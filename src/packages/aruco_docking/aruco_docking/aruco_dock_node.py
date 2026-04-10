#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist

IDLE = 'IDLE'; ALIGNING = 'ALIGNING'; DOCKING = 'DOCKING'; DOCKED = 'DOCKED'


class ArucoDockNode(Node):

    def __init__(self):
        super().__init__('aruco_dock_node')

        self.declare_parameter('marker_id',        0)
        self.declare_parameter('marker_length',    0.15)
        self.declare_parameter('target_distance',  0.05)
        self.declare_parameter('aruco_dict',       'DICT_4X4_50')
        self.declare_parameter('k_angular',        0.004)
        self.declare_parameter('k_linear',         0.6)
        self.declare_parameter('max_linear_vel',   0.15)
        self.declare_parameter('max_angular_vel',  0.4)
        self.declare_parameter('align_px_thresh',  15.0)
        self.declare_parameter('dist_tolerance',   0.008)
        self.declare_parameter('align_px_drive',   60.0)
        self.declare_parameter('debug_window',     False)

        p = self.get_parameter
        self.marker_id    = p('marker_id').value
        self.marker_len   = p('marker_length').value
        self.target_dist  = p('target_distance').value
        self.k_ang        = p('k_angular').value
        self.k_lin        = p('k_linear').value
        self.max_lin      = p('max_linear_vel').value
        self.max_ang      = p('max_angular_vel').value
        self.align_thresh = p('align_px_thresh').value
        self.dist_tol     = p('dist_tolerance').value
        self.align_drive  = p('align_px_drive').value
        self.debug_window = p('debug_window').value

        # Marker 3-D corner points in marker frame (OpenCV convention)
        h = self.marker_len / 2.0
        self.obj_pts = np.array([
            [-h,  h, 0],
            [ h,  h, 0],
            [ h, -h, 0],
            [-h, -h, 0],
        ], dtype=np.float32)

        dict_name  = p('aruco_dict').value
        aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        self._aruco_dict   = aruco_dict
        self._aruco_params = cv2.aruco.DetectorParameters_create()

        self.camera_matrix = None
        self.dist_coeffs   = None
        self.img_w = self.img_h = None
        self.state = IDLE
        self.bridge = CvBridge()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(CameraInfo, '/rear_camera/camera_info', self._info_cb, 10)
        self.create_subscription(Image, '/rear_camera/image_raw', self._img_cb, sensor_qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info(
            f'aruco_dock_node ready  id={self.marker_id}  target={self.target_dist*100:.1f}cm')

    def _info_cb(self, msg: CameraInfo):
        if self.camera_matrix is not None:
            return
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs   = np.array(msg.d, dtype=np.float64)
        self.img_w, self.img_h = msg.width, msg.height
        self.get_logger().info(f'Camera info received  {self.img_w}x{self.img_h}')

    def _img_cb(self, msg: Image):
        self.get_logger().info('CB fired', throttle_duration_sec=1.0)
        if self.camera_matrix is None:
            return
        if self.state == DOCKED:
            self._vel(0.0, 0.0)
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self._aruco_dict, parameters=self._aruco_params)
        self.get_logger().info(f'ids={ids}', throttle_duration_sec=1.0) 

        if ids is None:
            if self.state != IDLE:
                self.get_logger().warn('Marker lost', throttle_duration_sec=2.0)
                self.state = IDLE
            self._vel(0.0, 0.0)
            return

        target_idx = next((i for i, mid in enumerate(ids.flatten())
                           if mid == self.marker_id), None)
        if target_idx is None:
            self._vel(0.0, 0.0)
            return

        img_pts = corners[target_idx].reshape(4, 2).astype(np.float32)

        # ── Pose via solvePnP (replaces removed estimatePoseSingleMarkers) ──
        ok, rvec, tvec = cv2.solvePnP(
            self.obj_pts, img_pts,
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)

        if not ok:
            self._vel(0.0, 0.0)
            return

        tvec  = tvec.flatten()
        depth = float(tvec[2])

        err_x = float(np.mean(img_pts[:, 0])) - self.img_w / 2.0

        self.get_logger().info(
            f'[{self.state}] depth={depth:.3f}m err_x={err_x:+.1f}px',
            throttle_duration_sec=0.3)

        if self.debug_window:
            vis = frame.copy()
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)
            cv2.drawFrameAxes(vis, self.camera_matrix, self.dist_coeffs, rvec, tvec.reshape(3,1), 0.05)
            cv2.putText(vis, f'd={depth:.3f}m ex={err_x:+.0f}px [{self.state}]',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow('aruco_dock', vis)
            cv2.waitKey(1)

        aligned = abs(err_x) < self.align_thresh
        at_dist = abs(depth - self.target_dist) < self.dist_tol

        if aligned and at_dist:
            self.get_logger().info('*** DOCKED ***')
            self._vel(0.0, 0.0)
            self.state = DOCKED
            return

        if self.state == IDLE:
            self.state = ALIGNING

        # Rear camera: +err_x → marker right in image → robot's left → turn left (+angular_z)
        angular_z = float(np.clip(-self.k_ang * err_x, -self.max_ang, self.max_ang))

        linear_x = 0.0
        if abs(err_x) < self.align_drive:
            if depth > self.target_dist:
                linear_x = float(np.clip(-self.k_lin * (depth - self.target_dist),
                                         -self.max_lin, 0.0))
                self.state = DOCKING
            else:
                linear_x = float(np.clip(self.k_lin * (self.target_dist - depth) * 0.5,
                                         0.0, self.max_lin * 0.5))
        else:
            self.state = ALIGNING

        self._vel(linear_x, angular_z)

    def _vel(self, lx: float, az: float):
        msg = Twist()
        msg.linear.x = lx
        msg.angular.z = az
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._vel(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()