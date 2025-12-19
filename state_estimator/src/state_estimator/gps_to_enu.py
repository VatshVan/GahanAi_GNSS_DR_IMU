import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import numpy as np # Needed for array manipulation

class GpsToEnuNode(Node):
    def __init__(self):
        super().__init__('gps_to_enu_node')

        # --- CONFIGURATION ---
        self.USE_FIRST_POINT_AS_ORIGIN = True 
        
        self.origin_lat = 0.0
        self.origin_lon = 0.0
        self.origin_set = False

        # Earth Radius (meters)
        self.R = 6378137.0 

        # --- QOS PROFILES ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- SUBSCRIBER & PUBLISHER ---
        self.sub_gps = self.create_subscription(NavSatFix, 'gps/fix', self.cb_gps, qos)
        self.pub_enu = self.create_publisher(PoseWithCovarianceStamped, '/gps/enu_pose', 10)

        self.get_logger().info("🌍 GPS -> ENU Converter Started. Waiting for fix...")

    def cb_gps(self, msg):
        # 1. Reject invalid fixes
        if msg.status.status < 0:
            return

        lat = msg.latitude
        lon = msg.longitude

        # 2. Set Origin
        if not self.origin_set:
            if self.USE_FIRST_POINT_AS_ORIGIN:
                self.origin_lat = lat
                self.origin_lon = lon
                self.origin_set = True
                self.get_logger().info(f"📍 Origin set to: {lat}, {lon}")
            else:
                self.origin_set = True

        # 3. Convert to Meters (Local Tangent Plane)
        d_lat = math.radians(lat - self.origin_lat)
        d_lon = math.radians(lon - self.origin_lon)
        lat0_rad = math.radians(self.origin_lat)

        x = self.R * d_lon * math.cos(lat0_rad)
        y = self.R * d_lat

        # 4. Create Message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = "map" 
        
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0

        # --- COVARIANCE FIX ---
        # Initialize a 6x6 covariance matrix (flattened to 36)
        # We fill the diagonal for X, Y, Z using the GPS covariance
        # GPS Covariance is [xx, xy, xz, yx, yy, yz, zx, zy, zz] (Size 9)
        
        cov_36 = np.zeros(36, dtype=np.float64)
        
        # Map GPS covariance (3x3) to Pose covariance (6x6)
        # Index 0 (GPS xx) -> Index 0 (Pose xx)
        cov_36[0] = msg.position_covariance[0] 
        
        # Index 4 (GPS yy) -> Index 7 (Pose yy) (Row 1, Col 1)
        cov_36[7] = msg.position_covariance[4]
        
        # Index 8 (GPS zz) -> Index 14 (Pose zz) (Row 2, Col 2)
        cov_36[14] = msg.position_covariance[8]

        pose_msg.pose.covariance = cov_36

        self.pub_enu.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GpsToEnuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()