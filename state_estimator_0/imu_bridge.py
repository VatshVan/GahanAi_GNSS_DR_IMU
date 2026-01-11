#!/usr/bin/env python3
"""
DiffIMUBridge (WiFi Version)
============================
Reads dual-IMU data from an ESP32 TCP stream (default 192.168.4.1:8888),
performs calibration, fusion, and filtering, and publishes standard ROS 2 Imu messages.
"""

import sys
import socket
import math
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion

class DiffIMUBridge(Node):
    def __init__(self):
        super().__init__('diff_imu_bridge')
        
        # --- CONFIGURATION ---
        self.declare_parameter('esp_ip', '192.168.4.1') # Default AP IP
        self.declare_parameter('esp_port', 8888)        # IMU Port (matches ESP32 code)
        
        self.esp_ip = self.get_parameter('esp_ip').value
        self.esp_port = self.get_parameter('esp_port').value

        # --- QOS PROFILE ---
        # Matches EKF's "Best Effort" expectation
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- PUBLISHERS ---
        self.pub_imu_diff = self.create_publisher(Imu, '/imu/data_raw', sensor_qos)
        self.pub_pitch    = self.create_publisher(Float32, '/imu/pitch', sensor_qos)
        self.pub_roll     = self.create_publisher(Float32, '/imu/roll', sensor_qos)
        
        # --- SOCKET CONNECTION ---
        self.sock = None
        self.sock_file = None
        self.connect_to_esp()

        # --- CALIBRATION STATE ---
        self.calibration_buffer = []
        self.is_calibrated = False
        
        # Bias Vectors
        self.bias_accel_left  = np.zeros(3)
        self.bias_gyro_left   = np.zeros(3)
        self.bias_accel_right = np.zeros(3)
        self.bias_gyro_right  = np.zeros(3)
        
        # --- FILTER STATE ---
        self.pitch_filtered = 0.0
        self.roll_filtered  = 0.0
        self.last_timestamp = self.get_clock().now()

        # --- COVARIANCE ---
        ACCEL_COV = 0.0025 
        GYRO_COV = 0.0004 
        ORIENT_COV = 0.001 

        self.linear_accel_cov = [ACCEL_COV, 0., 0., 0., ACCEL_COV, 0., 0., 0., ACCEL_COV]
        self.angular_vel_cov = [GYRO_COV, 0., 0., 0., GYRO_COV, 0., 0., 0., GYRO_COV]
        self.orientation_cov = [ORIENT_COV, 0., 0., 0., ORIENT_COV, 0., 0., 0., ORIENT_COV]

        # Timer calls the socket reader
        self.create_timer(0.01, self.read_socket_data)
        self.get_logger().info(f"Bridge Started. Target: {self.esp_ip}:{self.esp_port}")

    def connect_to_esp(self):
        """Establishes the TCP connection to the ESP32."""
        try:
            if self.sock: 
                self.sock.close()
            
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(1.0) # Short timeout for non-blocking feel
            self.sock.connect((self.esp_ip, self.esp_port))
            
            # Create a file-like object for easy readline() usage
            self.sock_file = self.sock.makefile('r', encoding='utf-8', errors='replace')
            
            self.get_logger().info("Connected to ESP32 IMU Stream!")
            
        except Exception as e:
            # We don't log errors constantly, only warnings if it fails
            self.get_logger().warn(f"Connection Failed: {e}. Retrying in loop...")
            self.sock = None

    def read_socket_data(self):
        """Reads one line from socket, parses, calibrates, filters, and publishes."""
        if self.sock is None:
            self.connect_to_esp()
            return

        try:
            # Read line from socket
            raw_line = self.sock_file.readline()
            
            # Check for empty string (socket closed)
            if not raw_line: 
                raise ConnectionResetError("Empty read")
                
            raw_line = raw_line.strip()
            if not raw_line: return 
            self.get_logger().info(f"Data: {raw_line}")
            data_parts = raw_line.split(',')
            if len(data_parts) != 12: return
            
            # --- PARSING ---
            values = [float(x) for x in data_parts]
            accel_left  = np.array(values[0:3])
            gyro_left   = np.array(values[3:6])
            accel_right = np.array(values[6:9])
            gyro_right  = np.array(values[9:12])

            # --- 1. CALIBRATION ---
            if not self.is_calibrated:
                self.calibration_buffer.append(values)
                if len(self.calibration_buffer) > 300:
                    calib_matrix = np.array(self.calibration_buffer)
                    self.bias_accel_left  = np.mean(calib_matrix[:, 0:3], axis=0)
                    self.bias_gyro_left   = np.mean(calib_matrix[:, 3:6], axis=0) 
                    self.bias_accel_right = np.mean(calib_matrix[:, 6:9], axis=0)
                    self.bias_gyro_right  = np.mean(calib_matrix[:, 9:12], axis=0) 
                    
                    # Assume flat ground during calib -> Z accel should NOT be zeroed out
                    self.bias_accel_left[2]  = 0.0
                    self.bias_accel_right[2] = 0.0
                    
                    self.is_calibrated = True
                    self.last_timestamp = self.get_clock().now()
                    self.get_logger().info("CALIBRATION COMPLETE.")
                return

            # --- 2. BIAS CORRECTION ---
            accel_left  -= self.bias_accel_left
            gyro_left   -= self.bias_gyro_left 
            accel_right -= self.bias_accel_right
            gyro_right  -= self.bias_gyro_right 
            
            # AVERAGE THE TWO SENSORS (Noise Reduction)
            accel_avg = (accel_left + accel_right) / 2.0
            gyro_avg  = (gyro_left + gyro_right) / 2.0

            # --- 3. FILTERING ---
            current_ros_time = self.get_clock().now()
            dt = (current_ros_time - self.last_timestamp).nanoseconds / 1e9
            self.last_timestamp = current_ros_time
            if dt <= 0: return

            # Calculate Tilt from Accelerometer
            pitch_accel = math.atan2(-accel_avg[0], math.sqrt(accel_avg[1]**2 + accel_avg[2]**2))
            roll_accel  = math.atan2(accel_avg[1], accel_avg[2])
            
            # Convert Gyro to Rad/s for integration
            pitch_rate = math.radians(gyro_avg[1])
            roll_rate  = math.radians(gyro_avg[0])

            # Complementary Filter
            alpha = 0.98
            self.pitch_filtered = alpha * (self.pitch_filtered + pitch_rate * dt) + (1.0 - alpha) * pitch_accel
            self.roll_filtered  = alpha * (self.roll_filtered  + roll_rate * dt)  + (1.0 - alpha) * roll_accel

            # --- 4. PUBLISH ---
            imu_msg = Imu()
            imu_msg.header.stamp = current_ros_time.to_msg()
            imu_msg.header.frame_id = "imu_link"
            
            # Data
            imu_msg.linear_acceleration.x = accel_avg[0]
            imu_msg.linear_acceleration.y = accel_avg[1]
            imu_msg.linear_acceleration.z = accel_avg[2]

            imu_msg.angular_velocity.x = math.radians(gyro_avg[0])
            imu_msg.angular_velocity.y = math.radians(gyro_avg[1])
            imu_msg.angular_velocity.z = math.radians(gyro_avg[2]) 

            # Orientation
            q = self.euler_to_quaternion(self.roll_filtered, self.pitch_filtered, 0.0)
            imu_msg.orientation = q

            # Covariance
            imu_msg.linear_acceleration_covariance = self.linear_accel_cov
            imu_msg.angular_velocity_covariance    = self.angular_vel_cov
            imu_msg.orientation_covariance         = self.orientation_cov
            
            self.pub_imu_diff.publish(imu_msg)

            # Debug
            p = Float32(); p.data = self.pitch_filtered
            r = Float32(); r.data = self.roll_filtered
            self.pub_pitch.publish(p)
            self.pub_roll.publish(r)
            
        except (socket.timeout, ConnectionResetError, BrokenPipeError, OSError):
            self.get_logger().warn("Socket connection lost/timed out. Reconnecting...")
            self.sock = None
        except ValueError:
            pass # Ignore malformed numbers

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    rclpy.init()
    node = DiffIMUBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()
