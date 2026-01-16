#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import os
import math
from datetime import datetime
import numpy as np

from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion

class KittiBridge(Node):
    def __init__(self):
        super().__init__('kitti_bridge')

        # ==============================================================================
        # CONFIGURATION: UPDATE THIS PATH!
        # Point to the 'oxts' folder inside the extracted 'drive_0005_sync' folder.
        # Example: /home/user/Downloads/2011_09_26/2011_09_26_drive_0005_sync/oxts
        # ==============================================================================
        self.kitti_path = "/home/vatshvan/2011_09_26/2011_09_26_drive_0005_sync/oxts"
        
        # --- PUBLISHERS ---
        # "Best Effort" QoS to match your EKF settings
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', qos)
        self.pub_gps = self.create_publisher(PoseWithCovarianceStamped, '/gps/enu_pose', qos)
        self.pub_odom = self.create_publisher(Odometry, '/odom', qos) # Wheel speed
        self.pub_truth = self.create_publisher(Odometry, '/ground_truth', qos) 

        # --- LOAD DATA ---
        if not os.path.exists(self.kitti_path):
            self.get_logger().error(f"❌ Path not found: {self.kitti_path}")
            self.get_logger().error("Please update 'self.kitti_path' in the script!")
            return

        self.data_folder = os.path.join(self.kitti_path, 'data')
        self.data_files = sorted([f for f in os.listdir(self.data_folder) if f.endswith('.txt')])
        self.timestamps = self.load_timestamps()
        
        self.current_idx = 0
        self.origin_lat = None
        self.origin_lon = None

        self.get_logger().info(f"✅ Loaded {len(self.data_files)} frames from KITTI.")
        self.get_logger().info("🚀 Starting Playback at 10Hz...")
        
        # KITTI is recorded at 10Hz (0.1s)
        self.create_timer(0.1, self.update) 

    def load_timestamps(self):
        # Timestamps are typically one level up from 'data'
        ts_path = os.path.join(self.kitti_path, 'timestamps.txt')
        
        dates = []
        if os.path.exists(ts_path):
            with open(ts_path, 'r') as f:
                for line in f:
                    # Parse format: "2011-09-26 13:02:25.840245000"
                    # Truncate nanoseconds to microseconds for strptime
                    dt_str = line.strip()[:-3] 
                    dt = datetime.strptime(dt_str, "%Y-%m-%d %H:%M:%S.%f")
                    dates.append(dt)
        return dates

    def update(self):
        if self.current_idx >= len(self.data_files):
            self.get_logger().info("🏁 End of Dataset.")
            self.destroy_node()
            return

        # 1. Read Raw Line
        file_path = os.path.join(self.data_folder, self.data_files[self.current_idx])
        with open(file_path, 'r') as f:
            line = f.read().strip()
            vals = [float(x) for x in line.split()]

        # 2. Extract Values (Indices based on KITTI DevKit)
        lat, lon, alt = vals[0], vals[1], vals[2]
        roll, pitch, yaw = vals[3], vals[4], vals[5]
        vn, ve, vf = vals[6], vals[7], vals[8]       # Velocities (North, East, Forward)
        ax, ay, az = vals[11], vals[12], vals[13]    # Accel
        wx, wy, wz = vals[17], vals[18], vals[19]    # Gyro

        now = self.get_clock().now().to_msg()

        # 3. GPS -> ENU (Local X,Y)
        if self.origin_lat is None:
            self.origin_lat = lat
            self.origin_lon = lon
            self.get_logger().info(f"🌍 Origin set to Lat: {lat}, Lon: {lon}")
        
        x_enu, y_enu = self.latlon_to_enu(lat, lon, self.origin_lat, self.origin_lon)

        # --- A. PUBLISH GPS POSITION (/gps/enu_pose) ---
        msg_gps = PoseWithCovarianceStamped()
        msg_gps.header.stamp = now
        msg_gps.header.frame_id = "odom"
        msg_gps.pose.pose.position.x = x_enu
        msg_gps.pose.pose.position.y = y_enu
        msg_gps.pose.pose.position.z = alt
        # Fake Covariance (KITTI is very precise, let's say 0.5m accuracy)
        msg_gps.pose.covariance[0] = 0.5 # X Var
        msg_gps.pose.covariance[7] = 0.5 # Y Var
        self.pub_gps.publish(msg_gps)

        # --- B. PUBLISH WHEEL ODOMETRY (/odom) ---
        # Using 'vf' (velocity forward) to simulate wheel encoder speed
        msg_odom = Odometry()
        msg_odom.header.stamp = now
        msg_odom.header.frame_id = "odom"
        msg_odom.twist.twist.linear.x = vf
        self.pub_odom.publish(msg_odom)

        # --- C. PUBLISH IMU (/imu/data_raw) ---
        msg_imu = Imu()
        msg_imu.header.stamp = now
        msg_imu.header.frame_id = "imu_link"
        
        # Raw Data
        msg_imu.linear_acceleration.x = ax
        msg_imu.linear_acceleration.y = ay
        msg_imu.linear_acceleration.z = az
        msg_imu.angular_velocity.x = wx
        msg_imu.angular_velocity.y = wy
        msg_imu.angular_velocity.z = wz
        
        # Compass Injection (Using Truth Yaw as a perfect Magnetometer)
        # KITTI Yaw: 0 = East, CCW+
        q = self.euler_to_quaternion(0, 0, yaw) 
        msg_imu.orientation = q
        
        self.pub_imu.publish(msg_imu)

        # --- D. PUBLISH TRUTH (For plotting) ---
        msg_truth = Odometry()
        msg_truth.header.stamp = now
        msg_truth.header.frame_id = "odom"
        msg_truth.pose.pose.position.x = x_enu
        msg_truth.pose.pose.position.y = y_enu
        q_truth = self.euler_to_quaternion(roll, pitch, yaw)
        msg_truth.pose.pose.orientation = q_truth
        self.pub_truth.publish(msg_truth)

        self.current_idx += 1

    def latlon_to_enu(self, lat, lon, lat0, lon0):
        # Standard Haversine Projection for flat-earth assumption over small area
        R = 6378137.0 
        dLat = math.radians(lat - lat0)
        dLon = math.radians(lon - lon0)
        lat0_rad = math.radians(lat0)
        
        x = dLon * R * math.cos(lat0_rad)
        y = dLat * R
        return x, y

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(KittiBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()