#!/usr/bin/env python3
import time
import numpy as np

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
import math

import numpy as rnp

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Int32

import laser_geometry.laser_geometry as lg
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros
from tf_transformations import euler_from_quaternion


class ClfCbfPreprocess:
    """
    Topic subscribe and pre-processing (ROS 2).
    Exposes:
        _np_z: np.array([x, y, yaw])
        _np_z_dsr: np.array([x_d, y_d]) - Now set from service path
        scan_buffer: list of 2xN point arrays (map frame)
        _upstream_init_finish: bool when first data ready
    """

    # def __init__(self, node, agent_id=1):
    def __init__(self, node):
        self.node = node
        # self.agent_id = int(agent_id)

        # Flags/state
        self.mov_obs = False  # no moving obstacles initially
        self._upstream_data_ready = False
        self._upstream_init_finish = False
        self.localization_fail = False
        self.scan_buffer_ready = False
        self.mov_obs_ready = False

        # Data containers
        self._np_z = None
        self._np_z_dsr = None  # Will be set from service path
        self._prev_rbt_loc = None
        self._scan_frame = None
        self.scan_buffer = []
        self.mo_list = None

        # TF + laser projection
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node, spin_thread=True)
        self.laser_projection = lg.LaserProjection()

        # Topics
        odom_topic = f"/diff_drive_controller/odom"
        scan_topic = f"/scan"

        # Subscriptions (removed goal_pose subscription)
        self._odom_sub = self.node.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )
        self._scan_sub = self.node.create_subscription(
            LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data
        )

        self.node.get_logger().info("[clf_cbf_pre] Subscriptions created (path-only mode).")

    # ------------------- Callbacks -------------------

    def odom_callback(self, odom_msg: Odometry) -> None:
        pose = odom_msg.pose.pose
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(q)
        self._np_z = np.array([pose.position.x, pose.position.y, yaw])

        # Initialize _np_z_dsr to current position if not yet set
        if self._np_z_dsr is None:
            self._np_z_dsr = self._np_z[:2]  # Just x, y

        self._check_upstream_data()

    def scan_callback(self, scan_msg: LaserScan) -> None:
        if self._scan_frame is None:
            self._scan_frame = scan_msg.header.frame_id

        # Transform scan -> map and buffer the nearest-surface points
        try:
            map_frame = f"/map"
            tf = self.tf_buffer.lookup_transform(
                target_frame=map_frame,
                source_frame=self._scan_frame,
                time=Time(),  # latest available
                timeout=rclpy.duration.Duration(seconds=0.2).to_msg(),
            )
        except Exception as ex:
            # Early on, TF may not be available yet
            self.node.get_logger().debug(
                f"[clf_cbf_pre] TF {self._scan_frame} -> {map_frame} not ready: {ex}"
            )
            return

        pcl2_msg: PointCloud2 = self.laser_projection.projectLaser(scan_msg)
        pcl2_transformed = do_transform_cloud(pcl2_msg, tf)

        pcl = rnp.numpify(pcl2_transformed)
        points_x = pcl['x']
        points_y = pcl['y']
        surface_points = np.vstack((points_x, points_y))  # shape (2, N)

        self.scan_buffer.insert(0, surface_points)
        if len(self.scan_buffer) > 5:
            self.scan_buffer.pop()

        if len(self.scan_buffer) >= 5 and not self.scan_buffer_ready:
            self.scan_buffer_ready = True
            self.node.get_logger().info("[clf_cbf_pre] scan buffer ready.")

        self._check_upstream_data()

    def set_target_from_path(self, target_x: float, target_y: float) -> None:
        """Set the desired target position from service path."""
        self._np_z_dsr = np.array([target_x, target_y])

    # ------------------- Upstream readiness -------------------

    def _check_upstream_data(self) -> None:
        """Mark data readiness when odom and first scan are present."""
        status = True
        if self._np_z is None:
            status = False
        if self._np_z_dsr is None:
            status = False
        if self._scan_frame is None:
            status = False
        if not self.scan_buffer_ready:
            status = False

        if status and not self._upstream_init_finish:
            self._upstream_data_ready = True
            self._upstream_init_finish = True
            self.node.get_logger().info("[clf_cbf_pre] Upstream init complete: odom and scan ready.")

    # ------------------- Misc utils -------------------

    def check_de_localization(self) -> None:
        """
        Detect de-localization by monitoring motion perpendicular to heading.
        If detected, freeze pose at previous estimate.
        """
        if self._np_z is None:
            return

        current_rbt_loc = self._np_z[0:2]

        if self._prev_rbt_loc is None:
            self._prev_rbt_loc = self._np_z

        previous_rbt_loc = self._prev_rbt_loc[0:2]
        rbt_heading = self._np_z[2]
        dist_perp = np.abs(
            (current_rbt_loc - previous_rbt_loc)
            @ np.array([[-np.sin(rbt_heading)], [np.cos(rbt_heading)]])
        )

        if dist_perp <= 0.1 and not self.localization_fail:
            self._prev_rbt_loc = self._np_z
        else:
            self.localization_fail = True
            self._np_z = self._prev_rbt_loc


def clip(x: float, x_min: float, x_max: float) -> float:
    return max(x_min, min(x, x_max))


class ClfCbfPostprocess:
    """
    Downstream publisher interface (ROS 2).
    Clips control to hardware limits and publishes Twist and "reached" flags.
    """

    def __init__(self, node, ctrl_limits=None):
        self.node = node
        self.ctrl_limits = ctrl_limits or {}

        cmd_vel_topic = f"/diff_drive_controller/cmd_vel"
        reached_topic = f"/reached"

        self.cmd_vel_pub = self.node.create_publisher(Twist, cmd_vel_topic, 10)
        self.reached_pub = self.node.create_publisher(Int32, reached_topic, 10)

        self._body_twist = Twist()
        self.node.get_logger().info("[clf_cbf_post] Publishers created.")

    def send_cmd(self, v_dsr: float, w_dsr: float, clip_ctrl: bool = False, debug: bool = False) -> None:
        if clip_ctrl and self.ctrl_limits:
            v_dsr = clip(v_dsr, self.ctrl_limits['v_min'], self.ctrl_limits['v_max'])
            w_dsr = clip(w_dsr, self.ctrl_limits['w_min'], self.ctrl_limits['w_max'])

        self._body_twist.linear.x = float(v_dsr)
        self._body_twist.angular.z = float(w_dsr)
        self.cmd_vel_pub.publish(self._body_twist)

        if debug:
            self.node.get_logger().debug(f"[clf_cbf_post] cmd: v={v_dsr:.3f}, w={w_dsr:.3f}")

    def send_reached_waypoint(self) -> None:
        msg = Int32()
        msg.data = 1
        self.reached_pub.publish(msg)