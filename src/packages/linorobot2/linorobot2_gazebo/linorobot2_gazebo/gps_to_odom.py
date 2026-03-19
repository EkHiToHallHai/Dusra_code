#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import math

class GpsToOdom(Node):
    def __init__(self):
        super().__init__('gps_to_odom')
        
        self.odom_pub = self.create_publisher(Odometry, '/odometry/gps', 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        
        self.origin_lat = 37.412173071650805
        self.origin_lon = -121.998878727967
        
        self.get_logger().info('GPS to Odom node started')

    def latlon_to_xy(self, lat, lon):
        R = 6371000.0
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        origin_lat_rad = math.radians(self.origin_lat)
        origin_lon_rad = math.radians(self.origin_lon)
        x = R * (lon_rad - origin_lon_rad) * math.cos(origin_lat_rad)
        y = R * (lat_rad - origin_lat_rad)
        return x, y

    def gps_callback(self, msg):
        if msg.status.status < 0:
            return
        x, y = self.latlon_to_xy(msg.latitude, msg.longitude)
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        odom.pose.covariance[0] = 0.1
        odom.pose.covariance[7] = 0.1
        self.odom_pub.publish(odom)
        
def main(args=None):
    rclpy.init(args=args)
    node = GpsToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()