"""
GPS/GNSS Bridge Node for ROS 2 - GNSS Data Acquisition and ENU Localization
This module implements a ROS 2 node that interfaces with GNSS receivers via serial communication,
parses NMEA GGA sentences, and publishes both raw geodetic coordinates and local ENU (East-North-Up)
frame positioning data. It provides RTK-aware covariance estimation and automatic local tangent plane
initialization.
Module: state_estimator.src.state_estimator.gps_bridge
Classes:
    GNSSNode: Main ROS 2 node for GNSS data acquisition and processing.
Functions:
    main(args=None): Entry point for the ROS 2 node execution.
Key Features:
    - Serial NMEA GGA sentence parsing with error handling
    - Real-time NavSatFix publication (WGS84 geodetic coordinates)
    - Local ENU frame conversion with automatic origin initialization
    - RTK-aware covariance estimation (Fixed: 0.02m, Float: 0.5m, Standard: 2.5m)
    - HDOP-scaled variance for adaptive position uncertainty
    - Configurable serial port, baud rate, and frame identifiers via ROS 2 parameters
    - Robust error handling for malformed NMEA sentences and serial communication failures
Published Topics:
    /gnss/fix (sensor_msgs/NavSatFix): Raw GNSS position in WGS84 geodetic coordinates
    /gps/enu_pose (geometry_msgs/PoseWithCovarianceStamped): Local ENU frame position relative to origin
Parameters:
    port (str): Serial device path (default: '/dev/ttyUSB1')
    baud (int): Serial baud rate (default: 115200)
    frame_id (str): TF frame identifier for NavSatFix messages (default: 'gnss_link')
Dependencies:
    - rclpy: ROS 2 Python client library
    - serial: PySerial for UART communication
    - sensor_msgs: ROS 2 sensor message definitions
    - geometry_msgs: ROS 2 geometry message definitions
    - math: Standard library mathematics module
Notes:
    - ENU conversion assumes a local tangent plane approximation with origin set at first valid fix
    - Covariance estimation is based on HDOP value and RTK quality indicator
    - Serial reading operates at 100 Hz with non-blocking I/O (0.1s timeout)
    - Z-component covariance set to 4x horizontal variance due to limited altitude reliability
"""
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
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'gnss_link')
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # --- PUBLISHERS ---
        self.fix_pub = self.create_publisher(NavSatFix, '/gnss/fix', 10)
        self.enu_pub = self.create_publisher(PoseWithCovarianceStamped, '/gps/enu_pose', 10)

        # --- CARTESIAN CONVERSION STATE ---
        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None
        self.has_origin = False
        self.EARTH_RADIUS = 6378137.0 

        self.declare_parameter('use_saved_origin', True)
        self.declare_parameter('origin_lat', 0.0)
        self.declare_parameter('origin_lon', 0.0)
        self.declare_parameter('origin_alt', 0.0)

        # --- SERIAL CONNECTION ---
        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Connected to GNSS on {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Serial Connection Failed: {e}")

                # --- LOAD SAVED ORIGIN IF AVAILABLE ---
        use_saved = self.get_parameter('use_saved_origin').value

        if use_saved:
            lat0 = self.get_parameter('origin_lat').value
            lon0 = self.get_parameter('origin_lon').value
            alt0 = self.get_parameter('origin_alt').value

            if lat0 != 0.0 and lon0 != 0.0:
                self.origin_lat = lat0
                self.origin_lon = lon0
                self.origin_alt = alt0
                self.has_origin = True

                self.get_logger().info(
                    f"Loaded saved ENU origin: "
                    f"{lat0:.6f}, {lon0:.6f}, {alt0:.2f}"
                )

        self.create_timer(0.01, self.read_serial_data)

    def read_serial_data(self):
        if self.ser is None or not self.ser.is_open: return

        try:
            line = self.ser.readline().decode('utf-8', errors='replace').strip()
            
            if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                self.parse_gga(line)
                
        except Exception: pass

    def parse_gga(self, line):
        # Format: $GNGGA,time,lat,NS,lon,EW,quality,num_sats,hdop,alt...
        parts = line.split(',')
        if len(parts) < 10: return

        try:
            # 1. Check Fix Quality (0 = Invalid)
            quality = int(parts[6]) 
            if quality == 0: return 

            # 2. Parse Lat/Lon (NMEA is DDMM.MMMM, we need DD.DDDD)
            raw_lat = parts[2]
            lat_dir = parts[3]
            raw_lon = parts[4]
            lon_dir = parts[5]
            
            if not raw_lat or not raw_lon: return

            lat = self.nmea_to_decimal(raw_lat)
            if lat_dir == 'S': lat = -lat
            
            lon = self.nmea_to_decimal(raw_lon)
            if lon_dir == 'W': lon = -lon
            
            alt = float(parts[9]) if parts[9] else 0.0
            hdop = float(parts[8]) if parts[8] else 10.0

            # 3. Publish
            self.publish_fix_and_enu(lat, lon, alt, quality, hdop)
        except ValueError: pass

    def nmea_to_decimal(self, dm):
        # Helper: Converts "1908.1116" (DDMM.MMMM) -> 19.1351 (Decimal Deg)
        if not dm: return 0.0
        # The last 2 digits of the integer part + decimals are the Minutes
        # The rest is Degrees.
        # Example: 1908.1116 -> Deg: 19, Min: 08.1116
        
        dot_idx = dm.find('.')
        if dot_idx == -1: return float(dm) # Should not happen in standard NMEA

        # Degrees is everything up to 2 chars before the dot
        deg_end_idx = dot_idx - 2
        degrees = float(dm[:deg_end_idx])
        minutes = float(dm[deg_end_idx:])
        
        return degrees + (minutes / 60.0)

    def publish_fix_and_enu(self, lat, lon, alt, quality, hdop):
        current_time = self.get_clock().now().to_msg()
        
        # --- CALC VARIANCE (Trust) ---
        base_error = 2.5
        if quality == 4: base_error = 0.02 # RTK Fixed
        elif quality == 5: base_error = 0.5 # RTK Float
        variance = (hdop * base_error) ** 2
        if quality < 4:  # non-RTK
            variance *= 3.0

        # --- 1. PUBLISH RAW FIX ---
        msg = NavSatFix()
        msg.header.stamp = current_time
        msg.header.frame_id = self.frame_id
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        
        if quality >= 4: msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        else: msg.status.status = NavSatStatus.STATUS_FIX
        
        msg.position_covariance = [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance*4]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.fix_pub.publish(msg)

        # --- 2. PUBLISH ENU (Meters) ---
        if not self.has_origin:
            self.origin_lat = lat
            self.origin_lon = lon
            self.origin_alt = alt
            self.has_origin = True
            self.get_logger().info(f"ORIGIN SET: {lat:.6f}, {lon:.6f}")
            return

        # Simple Local Tangent Plane Conversion
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        origin_lat_rad = math.radians(self.origin_lat)
        origin_lon_rad = math.radians(self.origin_lon)
        cos_lat = math.cos(origin_lat_rad)

        x_enu = (lon_rad - origin_lon_rad) * self.EARTH_RADIUS * cos_lat
        y_enu = (lat_rad - origin_lat_rad) * self.EARTH_RADIUS
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = "odom"
        pose_msg.pose.pose.position.x = x_enu
        pose_msg.pose.pose.position.y = y_enu
        
        # Covariance (Diagonal)
        pose_msg.pose.covariance[0] = variance # X
        pose_msg.pose.covariance[7] = variance # Y
        pose_msg.pose.covariance[35] = 999.0   # Yaw (Unknown)

        self.enu_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GNSSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()