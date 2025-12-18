#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class SimGPSBridge(Node):
    def __init__(self):
        super().__init__('sim_gps_bridge')

        # --- INPUT: Gazebo Topic ---
        # Remap this if your simulation uses /gps/fix
        self.sub_fix = self.create_subscription(NavSatFix, '/gps/fix', self.callback_gps, 10)
        
        # --- OUTPUT: EKF Topic ---
        self.pub_enu = self.create_publisher(PoseWithCovarianceStamped, '/gps/enu_pose', 10)

        # Local Tangent Plane Origin
        self.origin_lat = None
        self.origin_lon = None
        self.EARTH_RADIUS = 6378137.0
        self.has_origin = False

        self.get_logger().info("✅ SIM GPS BRIDGE STARTED")

    def callback_gps(self, msg):
        # Ignore invalid fixes
        if msg.status.status < 0: return

        lat = msg.latitude
        lon = msg.longitude

        # 1. Set Origin (First valid point becomes (0,0))
        if not self.has_origin:
            self.origin_lat = lat
            self.origin_lon = lon
            self.has_origin = True
            self.get_logger().info(f"🌍 SIM ORIGIN SET: {lat}, {lon}")
            return

        # 2. Convert WGS84 (Lat/Lon) -> ENU (Meters)
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        origin_lat_rad = math.radians(self.origin_lat)
        origin_lon_rad = math.radians(self.origin_lon)
        cos_lat = math.cos(origin_lat_rad)

        x_enu = (lon_rad - origin_lon_rad) * self.EARTH_RADIUS * cos_lat
        y_enu = (lat_rad - origin_lat_rad) * self.EARTH_RADIUS

        # 3. Publish to EKF
        pose = PoseWithCovarianceStamped()
        pose.header = msg.header
        pose.header.frame_id = "odom"
        
        pose.pose.pose.position.x = x_enu
        pose.pose.pose.position.y = y_enu
        
        # 4. Set Covariance (Trust)
        # Gazebo GPS is usually ~1.5m accurate unless configured otherwise
        variance = 1.5**2
        pose.pose.covariance[0] = variance  # X Variance
        pose.pose.covariance[7] = variance  # Y Variance
        pose.pose.covariance[35] = 999.0    # Yaw Variance (Unknown)
        
        self.pub_enu.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = SimGPSBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()