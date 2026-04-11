#!/usr/bin/env python3
""" Low level velocity controller for unicycle-like robot (ROS 2).

Interfaces:
    Input:
        desired robot states from service path
        current robot states from odometry
        laser scan for CBF
    Output:
        velocity command via service response
"""
from __future__ import annotations

import os
import json
import numpy as np
import numpy.linalg as npla

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Use relative imports since we're in a package
from .utils import ClfCbfPreprocess, ClfCbfPostprocess
from .dynamic_controller import ClfCbfDrccp_dynamic_Controller
from dr_clf_cbf_interfaces.srv import ComputeTwist
from ament_index_python.packages import get_package_share_directory


class ClfCbfControllerNode(Node):
    """CLF-CBF-QP controller wrapper - service-based path following only."""

    def __init__(self) -> None:
        super().__init__('clf_cbf_controller')
        
        # -------------------- Config / Core ------------------
        self._load_controller_config()

        # -------------------- Modules -----------------------
        self.preprocessor = ClfCbfPreprocess(self)
        self.postprocessor = ClfCbfPostprocess(self, ctrl_limits=self.ctrl_limits)

        # For moving obstacles (dynamic env)
        self.mov_obs_flag = False
        self.mov_obs_data = []

        np.set_printoptions(formatter={'float': '{: 0.2f}'.format})

        # Service for controller plugin to call
        self._compute_twist_srv = self.create_service(
            ComputeTwist,
            '/dr_clf_cbf_controller/compute_twist',
            self._handle_compute_twist
        )
        self.get_logger().info("ComputeTwist service ready at '/dr_clf_cbf_controller/compute_twist'")
        self.get_logger().info(f"CLF-CBF CONTROL NODE INIT SUCCESSFUL for Agent!")

    # ------------------------ Helpers ------------------------

    def _load_controller_config(self) -> None:
        """Load controller configuration JSON and build controller core."""
        try:
            package_share = get_package_share_directory('dr_clf_cbf_controller')
            config_path = os.path.join(package_share, 'config', 'controller_config.json')
            with open(config_path, 'r') as f:
                cfg = json.load(f)
            params = cfg.get('parameters', {})
            bot_params = cfg.get('bot_params', {})
        except Exception as e:
            self.get_logger().warn(f"[clf_cbf_controller] Could not read controller_config.json; using defaults. ({e})")
            params = {}
            bot_params = {}

        self._wheel_offset = params.get('wheel_offset', 0.08)
        self._cbf_rate = params.get('cbf_rate', 0.4)
        self.noise_level = params.get('noise_level', 0.01)
        self.ctrl_limits = {'v_min': bot_params.get('v_min', -2.0),
                            'v_max': bot_params.get('v_max', 2.0),
                            'w_min': bot_params.get('w_min', -0.9),
                            'w_max': bot_params.get('w_max', 0.9)}
        self.core = ClfCbfDrccp_dynamic_Controller(**params)

    # ------------------------ Service Handler ------------------------

    def _handle_compute_twist(self, request, response):
        """
        Compute a Twist for the given robot pose and path.
        Request:
            request.current_pose (PoseStamped)
            request.path (nav_msgs/Path)
        Response:
            response.cmd_vel (Twist)
            response.success (bool)
            response.message (string)
            response.goal_reached (bool)
        """

        try:
            # Convert the incoming pose to controller state (z)
            px = request.current_pose.pose.position.x
            py = request.current_pose.pose.position.y
            q = request.current_pose.pose.orientation
            
            # Compute yaw from quaternion
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = np.arctan2(siny_cosp, cosy_cosp)

            # Build robot state z
            z = np.array([px, py, yaw])

            # If path empty, reply with zero twist
            if len(request.path.poses) == 0:
                response.cmd_vel = Twist()
                response.success = False
                response.message = "empty path"
                response.goal_reached = True
                return response

            # Find target pose from path (look-ahead logic)
            # Simple implementation: find closest point beyond a threshold distance
            look_ahead_dist = 0.5  # meters
            target_pose = None
            
            for p in request.path.poses:
                dx = p.pose.position.x - px
                dy = p.pose.position.y - py
                d = (dx*dx + dy*dy)**0.5
                if d >= look_ahead_dist:
                    target_pose = p.pose
                    break
            
            # If no point beyond look_ahead, use last point
            if target_pose is None:
                target_pose = request.path.poses[-1].pose

            # Construct gamma_s (2-vector) from target_pose
            gamma_x = target_pose.position.x
            gamma_y = target_pose.position.y
            gamma_s = np.array([gamma_x, gamma_y])

            # Update preprocessor with target
            self.preprocessor.set_target_from_path(gamma_x, gamma_y)

            # Prepare CBF samples from scan buffer
            num_samples = 5
            rbt_xy = np.array([[z[0]], [z[1]]])

            if len(self.preprocessor.scan_buffer) >= num_samples:
                cbf_value = np.zeros((num_samples,))
                cbf_grad = np.zeros((2, num_samples))
                partial_h_partial_t_static = np.zeros((num_samples,))

                for index, item in enumerate(self.preprocessor.scan_buffer[:num_samples]):
                    dist_rbt2surface = np.linalg.norm(rbt_xy - item, axis=0)
                    idx = int(np.argmin(dist_rbt2surface))
                    cbf_value[index] = dist_rbt2surface[idx]
                    gradient = rbt_xy[:, 0] - item[:, idx]
                    norm_gradient = gradient / np.linalg.norm(gradient)
                    cbf_grad[:, index] = norm_gradient

                partial_h_partial_t = partial_h_partial_t_static

                # Pick five smallest h samples
                select_idx = np.argsort((cbf_value - 0.31) * self._cbf_rate + partial_h_partial_t)[0:5]
                sorted_cbf_value = cbf_value[select_idx]
                sorted_cbf_grad = cbf_grad[:, select_idx]
                sorted_partial_hdt = partial_h_partial_t[select_idx]
            else:
                # Use placeholder values if scan buffer not ready
                sorted_cbf_value = np.ones(5) * 10.0  # Large safe distance
                sorted_cbf_grad = np.zeros((2, 5))
                sorted_partial_hdt = np.zeros(5)

            # Call controller to compute v, w
            v, w = self.core.generate_controller(
                rbt_pose=z,
                gamma_s=gamma_s,
                h_samples=sorted_cbf_value,
                h_grad_samples=sorted_cbf_grad,
                dh_dt_samples=sorted_partial_hdt
            )

            # Create Twist
            t = Twist()
            t.linear.x = float(v)
            t.angular.z = float(w)

            response.cmd_vel = t
            response.success = True
            response.message = "ok"

            # Determine if goal is reached: distance to last point on path
            last_pose = request.path.poses[-1].pose
            dx = last_pose.position.x - px
            dy = last_pose.position.y - py
            dist_to_goal = (dx*dx + dy*dy)**0.5
            response.goal_reached = (dist_to_goal < 0.3)  # 30cm threshold

            return response

        except Exception as e:
            self.get_logger().error(f"ComputeTwist handler error: {e}")
            response.cmd_vel = Twist()
            response.success = False
            response.message = str(e)
            response.goal_reached = False
            return response


def main(args=None):
    rclpy.init(args=args)
    node = ClfCbfControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()