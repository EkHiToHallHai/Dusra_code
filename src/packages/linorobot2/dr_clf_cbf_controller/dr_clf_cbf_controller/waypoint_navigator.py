#!/usr/bin/env python3
"""GPS Waypoint Navigator — no Nav2, no SLAM.
Converts lat/lon waypoints → XY, calls CLF-CBF service for cmd_vel.
Sends all remaining waypoints as a full path to the controller.
The controller uses look-ahead logic to follow the path smoothly."""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from dr_clf_cbf_interfaces.srv import ComputeTwist


# ── Edit your waypoints here ──────────────────────────────────────────────────
WAYPOINTS_LATLON = [
    (37.4122, -121.9989),
    (37.4123, -121.9988),
    (37.4124, -121.9990),
]

# Must match <latitude_deg> and <longitude_deg> in your world SDF
# and must match the origin in gps_to_odom.py
ORIGIN_LAT =  37.412173071650805
ORIGIN_LON = -121.998878727967

# Distance threshold in metres to consider a waypoint reached
GOAL_RADIUS = 1.0
# ─────────────────────────────────────────────────────────────────────────────


def latlon_to_xy(lat, lon):
    """Convert lat/lon to local XY metres using equirectangular approximation.
    Uses the same formula as gps_to_odom.py to ensure consistency."""
    R = 6_371_000.0
    x = R * math.radians(lon - ORIGIN_LON) * math.cos(math.radians(ORIGIN_LAT))
    y = R * math.radians(lat - ORIGIN_LAT)
    return x, y


class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')

        # Convert all lat/lon waypoints to XY once at startup
        self.waypoints_xy = [latlon_to_xy(la, lo) for la, lo in WAYPOINTS_LATLON]
        self.current_wp   = 0  # index of the waypoint we are currently driving to

        self.pose = None  # Will be filled by odometry callback

        # Log all waypoints for debugging
        self.get_logger().info(f'Loaded {len(self.waypoints_xy)} waypoints:')
        for i, (x, y) in enumerate(self.waypoints_xy):
            self.get_logger().info(f'  WP{i}: x={x:.3f} m, y={y:.3f} m')

        # ── Subscribers ───────────────────────────────────────────────────────
        # Subscribe to the global EKF odometry which fuses GPS + IMU
        # This gives us the robot pose in the map frame
        self.create_subscription(
            Odometry,
            '/odometry/filtered/global',
            self._odom_cb,
            10
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── CLF-CBF Service Client ────────────────────────────────────────────
        self.cli = self.create_client(
            ComputeTwist,
            '/dr_clf_cbf_controller/compute_twist'
        )
        self.get_logger().info('Waiting for CLF-CBF compute_twist service...')
        self.cli.wait_for_service()
        self.get_logger().info('CLF-CBF compute_twist service is ready.')

        # Flag to prevent overlapping service calls
        self._service_call_pending = False

        # ── Control Loop at 10 Hz ─────────────────────────────────────────────
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info('Waypoint Navigator initialized and running.')

    # ── Odometry Callback ─────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        """Store the latest robot pose from the global EKF odometry."""
        self.pose = msg.pose.pose

    # ── Helper: Build Full Remaining Path ─────────────────────────────────────

    def _make_path(self) -> Path:
        """Build a Path message containing all remaining waypoints from the
        current waypoint index onwards. The CLF-CBF controller uses look-ahead
        logic to follow this path smoothly, picking the next point beyond
        look_ahead_dist. As waypoints are reached, the path shrinks."""
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp    = self.get_clock().now().to_msg()

        for tx, ty in self.waypoints_xy[self.current_wp:]:
            ps = PoseStamped()
            ps.header             = path.header
            ps.pose.position.x    = tx
            ps.pose.position.y    = ty
            ps.pose.position.z    = 0.0
            ps.pose.orientation.w = 1.0
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0
            path.poses.append(ps)

        return path

    # ── Helper: Build Current Pose Stamped ────────────────────────────────────

    def _current_pose_stamped(self) -> PoseStamped:
        """Wrap the current robot pose in a PoseStamped for the service request."""
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp    = self.get_clock().now().to_msg()
        ps.pose            = self.pose
        return ps

    # ── Helper: Distance to a Target XY ──────────────────────────────────────

    def _dist_to(self, tx: float, ty: float) -> float:
        """Euclidean distance from current robot position to target (tx, ty)."""
        px = self.pose.position.x
        py = self.pose.position.y
        return math.hypot(tx - px, ty - py)

    # ── Main Control Loop ─────────────────────────────────────────────────────

    def _control_loop(self):
        """Called at 10 Hz. Checks if current waypoint is reached, advances
        index if so, otherwise calls CLF-CBF service with remaining path."""

        # Wait until we have a valid pose from odometry
        if self.pose is None:
            self.get_logger().warn(
                'No odometry received yet. Waiting...',
                throttle_duration_sec=5.0
            )
            return

        # Check if all waypoints have been reached
        if self.current_wp >= len(self.waypoints_xy):
            self.get_logger().info(
                'All waypoints reached. Robot stopped.',
                once=True
            )
            self.cmd_pub.publish(Twist())  # Publish zero velocity to stop robot
            return

        # Skip if a service call is already in flight
        if self._service_call_pending:
            return

        # Get current target waypoint
        tx, ty = self.waypoints_xy[self.current_wp]

        # Check if current waypoint is reached within GOAL_RADIUS
        dist = self._dist_to(tx, ty)
        if dist < GOAL_RADIUS:
            self.get_logger().info(
                f'Waypoint {self.current_wp} reached: '
                f'x={tx:.3f} m, y={ty:.3f} m (dist={dist:.3f} m). '
                f'Advancing to waypoint {self.current_wp + 1}.'
            )
            self.current_wp += 1
            return

        self.get_logger().info(
            f'Navigating to WP{self.current_wp}: '
            f'x={tx:.3f} m, y={ty:.3f} m | dist={dist:.3f} m | '
            f'remaining waypoints={len(self.waypoints_xy) - self.current_wp}',
            throttle_duration_sec=2.0
        )

        # Build service request with full remaining path
        req = ComputeTwist.Request()
        req.current_pose = self._current_pose_stamped()
        req.path         = self._make_path()

        # Send async service call — result handled in _twist_cb
        self._service_call_pending = True
        future = self.cli.call_async(req)
        future.add_done_callback(self._twist_cb)

    # ── Service Response Callback ─────────────────────────────────────────────

    def _twist_cb(self, future):
        """Handle the CLF-CBF service response. Publish cmd_vel if successful."""
        self._service_call_pending = False
        try:
            res = future.result()
            if res.success:
                self.cmd_pub.publish(res.cmd_vel)
                self.get_logger().debug(
                    f'cmd_vel: v={res.cmd_vel.linear.x:.3f} m/s, '
                    f'w={res.cmd_vel.angular.z:.3f} rad/s'
                )
            else:
                self.get_logger().warn(
                    f'CLF-CBF returned failure: {res.message}. '
                    f'Publishing zero velocity.'
                )
                self.cmd_pub.publish(Twist())

        except Exception as e:
            self.get_logger().error(
                f'CLF-CBF service call failed with exception: {e}. '
                f'Publishing zero velocity.'
            )
            self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received. Shutting down.')
    finally:
        node.cmd_pub.publish(Twist())  # Stop robot on shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()