#!/usr/bin/env python3
"""
GNSS Bridge Node (WiFi Version + Velocity)
==========================================
Reads NMEA data from an ESP32 TCP stream (default 192.168.4.1:8889).
Parses GGA (Position) and RMC (Speed + Course).
Publishes:
  - /gnss/fix (Raw Fix)
  - /gps/enu_pose (Odometry/Pose)
  - /gps/speed_kmph (Scalar Speed for EKF Main)
  - /gps/fix_velocity (Vector Velocity for EKF Validation) <--- NEW
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import socket
import math
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from std_msgs.msg import Float32

class GNSSNode(Node):
    def __init__(self):
        super().__init__('gnss_node')

        # --- CONFIGURATION ---
        self.declare_parameter('esp_ip', '192.168.4.1')
        self.declare_parameter('esp_port', 8889)
        self.declare_parameter('frame_id', 'gnss_link')
        
        self.esp_ip = self.get_parameter('esp_ip').value
        self.esp_port = self.get_parameter('esp_port').value
        self.frame_id = self.get_parameter('frame_id').value

        # --- QOS PROFILE ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- PUBLISHERS ---
        self.fix_pub = self.create_publisher(NavSatFix, '/gnss/fix', sensor_qos)
        self.enu_pub = self.create_publisher(PoseWithCovarianceStamped, '/gps/enu_pose', sensor_qos)
        self.speed_pub = self.create_publisher(Float32, '/gps/speed_kmph', sensor_qos)
        
        # NEW: Vector Velocity Publisher
        self.vel_pub = self.create_publisher(TwistWithCovarianceStamped, '/gps/fix_velocity', sensor_qos)

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

        # --- LOAD SAVED ORIGIN ---
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
                self.get_logger().info(f"Loaded saved ENU origin: {lat0:.6f}, {lon0:.6f}")

        # --- SOCKET CONNECTION ---
        self.sock = None
        self.sock_file = None
        self.connect_to_esp()

        self.create_timer(0.05, self.read_socket_data)
        self.get_logger().info(f"GNSS Bridge Started. Target: {self.esp_ip}:{self.esp_port}")

    def connect_to_esp(self):
        try:
            if self.sock: self.sock.close()
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(0.1)
            self.sock.connect((self.esp_ip, self.esp_port))
            self.sock_file = self.sock.makefile('r', encoding='utf-8', errors='replace')
            self.get_logger().info("Connected to ESP32 GPS Stream!")
        except Exception:
            self.sock = None

    def read_socket_data(self):
        if self.sock is None: 
            self.connect_to_esp()
            return

        try:
            line = self.sock_file.readline()
            if not line: raise ConnectionResetError("Empty read")
            line = line.strip()
            if not line: return

            if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                self.parse_gga(line)
            elif line.startswith('$GNRMC') or line.startswith('$GPRMC'):
                self.parse_rmc(line)

        except (socket.timeout, ConnectionResetError, BrokenPipeError, OSError):
            self.sock = None

    def parse_rmc(self, line):
        # Format: $GNRMC,time,status,lat,NS,lon,EW,speed_knots,course,date...
        parts = line.split(',')
        if len(parts) < 9: return # Need at least up to course
        
        try:
            # Check Status (A=Active)
            if parts[2] != 'A': return

            # 1. Parse Speed (Knots)
            speed_knots_str = parts[7]
            if not speed_knots_str: return
            speed_knots = float(speed_knots_str)
            
            # Conversions
            speed_kmph = speed_knots * 1.852
            speed_mps  = speed_knots * 0.514444

            # 2. Parse Course (Degrees True)
            course_deg_str = parts[8]
            course_deg = float(course_deg_str) if course_deg_str else 0.0
            
            # --- PUBLISH SCALAR SPEED (For EKF Main) ---
            msg_speed = Float32()
            msg_speed.data = speed_kmph
            self.speed_pub.publish(msg_speed)

            # --- PUBLISH VECTOR VELOCITY (For EKF Validation) ---
            # Convert Course (Clockwise from North) to ENU (East/North) components
            # Vx (East)  = Speed * sin(course)
            # Vy (North) = Speed * cos(course)
            course_rad = math.radians(course_deg)
            vx = speed_mps * math.sin(course_rad)
            vy = speed_mps * math.cos(course_rad)

            vel_msg = TwistWithCovarianceStamped()
            vel_msg.header.stamp = self.get_clock().now().to_msg()
            vel_msg.header.frame_id = self.frame_id
            
            vel_msg.twist.twist.linear.x = vx
            vel_msg.twist.twist.linear.y = vy
            vel_msg.twist.twist.linear.z = 0.0

            # Covariance (We trust GPS speed moderately)
            # Diagonal: [x, y, z, roll, pitch, yaw]
            # Speed from RMC is usually decent, assign variance ~0.25 (0.5m/s error)
            cov = [0.0] * 36
            cov[0] = 0.25 # Var X
            cov[7] = 0.25 # Var Y
            vel_msg.twist.covariance = cov

            self.vel_pub.publish(vel_msg)
            
        except ValueError: pass

    def parse_gga(self, line):
        parts = line.split(',')
        if len(parts) < 10: return

        try:
            quality = int(parts[6]) 
            if quality == 0: return 

            raw_lat = parts[2]; lat_dir = parts[3]
            raw_lon = parts[4]; lon_dir = parts[5]
            
            if not raw_lat or not raw_lon: return

            lat = self.nmea_to_decimal(raw_lat)
            if lat_dir == 'S': lat = -lat
            
            lon = self.nmea_to_decimal(raw_lon)
            if lon_dir == 'W': lon = -lon
            
            alt = float(parts[9]) if parts[9] else 0.0
            hdop = float(parts[8]) if parts[8] else 10.0

            self.publish_fix_and_enu(lat, lon, alt, quality, hdop)
        except ValueError: pass

    def nmea_to_decimal(self, dm):
        if not dm: return 0.0
        dot_idx = dm.find('.')
        if dot_idx == -1: return float(dm)
        deg_end_idx = dot_idx - 2
        degrees = float(dm[:deg_end_idx])
        minutes = float(dm[deg_end_idx:])
        return degrees + (minutes / 60.0)

    def publish_fix_and_enu(self, lat, lon, alt, quality, hdop):
        current_time = self.get_clock().now().to_msg()
        
        # Variance calc
        base_error = 2.5
        if quality == 4: base_error = 0.02
        elif quality == 5: base_error = 0.5
        variance = (hdop * base_error) ** 2
        if quality < 4: variance *= 3.0

        # 1. RAW FIX
        msg = NavSatFix()
        msg.header.stamp = current_time
        msg.header.frame_id = self.frame_id
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        if quality >= 4: msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        else: msg.status.status = NavSatStatus.STATUS_FIX
        msg.position_covariance = [variance, 0., 0., 0., variance, 0., 0., 0., variance*4]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.fix_pub.publish(msg)

        # 2. ENU POSE
        if not self.has_origin:
            self.origin_lat = lat
            self.origin_lon = lon
            self.origin_alt = alt
            self.has_origin = True
            self.get_logger().info(f"ORIGIN SET: {lat:.6f}, {lon:.6f}")
            return

        lat_rad = math.radians(lat); lon_rad = math.radians(lon)
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
        pose_msg.pose.covariance[0] = variance
        pose_msg.pose.covariance[7] = variance
        pose_msg.pose.covariance[35] = 999.0 

        self.enu_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GNSSNode())
    rclpy.shutdown()

if __name__ == '__main__': main()