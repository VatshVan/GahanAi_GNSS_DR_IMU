#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import math
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseWithCovarianceStamped

class GNSSNode(Node):
    def __init__(self):
        super().__init__('gnss_node')

        # --- CONFIGURATION ---
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'gnss_link')
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # --- PUBLISHERS ---
        # 1. Standard Fix (For debugging/logging)
        self.fix_pub = self.create_publisher(NavSatFix, '/gnss/fix', 10)
        
        # 2. ENU Pose (For EKF Fusion)
        # Publishes position in meters relative to startup point
        self.enu_pub = self.create_publisher(PoseWithCovarianceStamped, '/gps/enu_pose', 10)

        # --- CARTESIAN CONVERSION STATE ---
        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None
        self.has_origin = False
        
        # Earth Radius (meters)
        self.EARTH_RADIUS = 6378137.0 

        # --- SERIAL CONNECTION ---
        self.ser = None
        self.connect_serial(port, baud)

        # --- TIMER ---
        self.create_timer(0.01, self.read_serial_data)
        
        self.get_logger().info(f"GNSS Node Started. Listening on {port}...")

    def connect_serial(self, port, baud):
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Connected to Arduino at {baud} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not connect to serial: {e}")

    def read_serial_data(self):
        if self.ser is None or not self.ser.is_open:
            return

        try:
            line = self.ser.readline().decode('utf-8', errors='replace').strip()
            
            if not line.startswith('$GNSS'):
                return

            parts = line.split(',')
            if len(parts) != 6:
                return

            # Extract Data
            lat = float(parts[1])
            lon = float(parts[2])
            alt = float(parts[3])
            fix_type = int(parts[4]) 
            hdop = float(parts[5])

            # Publish both the Raw Fix (Debug) and ENU Pose (EKF)
            self.publish_fix(lat, lon, alt, fix_type, hdop)

        except ValueError:
            pass
        except Exception as e:
            self.get_logger().error(f"Serial Read Error: {e}")

    def publish_fix(self, lat, lon, alt, fix_type, hdop):
        current_time = self.get_clock().now().to_msg()

        # --- 1. PREPARE RAW MSG ---
        msg = NavSatFix()
        msg.header.stamp = current_time
        msg.header.frame_id = self.frame_id
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt

        # Map Status
        if fix_type == 0: msg.status.status = NavSatStatus.STATUS_NO_FIX
        elif fix_type == 4: msg.status.status = NavSatStatus.STATUS_SBAS_FIX
        elif fix_type >= 5: msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        else: msg.status.status = NavSatStatus.STATUS_FIX
        
        msg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS

        # Covariance Calculation
        base_error = 2.5
        if fix_type == 6: base_error = 0.02 # RTK Fixed
        elif fix_type == 5: base_error = 0.5 # RTK Float
        
        variance = (hdop * base_error) ** 2
        
        msg.position_covariance = [
            variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, variance * 4.0
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        # Publish Raw
        self.fix_pub.publish(msg)

        # --- 2. CONVERT TO ENU & PUBLISH POSE ---
        if fix_type > 0: # Only convert if we have a fix
            self.publish_enu_pose(lat, lon, alt, variance, current_time)

    def publish_enu_pose(self, lat, lon, alt, variance, time_stamp):
        # A. Set Origin if not set
        if not self.has_origin:
            self.origin_lat = lat
            self.origin_lon = lon
            self.origin_alt = alt
            self.has_origin = True
            self.get_logger().warn(f"SET LOCAL ORIGIN: {lat}, {lon}")
            return # Skip first frame to avoid 0.0 jump

        # B. Convert Lat/Lon to meters (Local Tangent Plane)
        # dX = (lon_diff) * cos(lat) * Radius
        # dY = (lat_diff) * Radius
        
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        origin_lat_rad = math.radians(self.origin_lat)
        origin_lon_rad = math.radians(self.origin_lon)

        cos_lat = math.cos(origin_lat_rad)

        x_enu = (lon_rad - origin_lon_rad) * self.EARTH_RADIUS * cos_lat
        y_enu = (lat_rad - origin_lat_rad) * self.EARTH_RADIUS
        z_enu = alt - self.origin_alt

        # C. Create PoseWithCovarianceStamped
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = time_stamp
        pose_msg.header.frame_id = "odom" # EKF will fuse this into Odom frame

        # Set Position
        pose_msg.pose.pose.position.x = x_enu
        pose_msg.pose.pose.position.y = y_enu
        pose_msg.pose.pose.position.z = z_enu
        
        # Orientation (GNSS doesn't give orientation, set to identity)
        pose_msg.pose.pose.orientation.w = 1.0

        # Set Covariance (Reuse the logic from NavSatFix)
        # 6x6 Matrix: [x, y, z, roll, pitch, yaw]
        # We only populate x, y, z diagonals. Orientation is Unknown (High variance).
        
        cov = [0.0] * 36
        cov[0] = variance   # X variance
        cov[7] = variance   # Y variance
        cov[14] = variance * 4.0 # Z variance
        
        # Orientation variance (Huge value = "Don't trust my orientation")
        cov[21] = 9999.0
        cov[28] = 9999.0
        cov[35] = 9999.0

        pose_msg.pose.covariance = cov

        self.enu_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GNSSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()