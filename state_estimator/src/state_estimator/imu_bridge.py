#!/usr/bin/env python3
import sys
import serial
import math
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion

class DiffIMUBridge(Node):
    def __init__(self, port='/dev/ttyUSB0', baud=115200):
        super().__init__('diff_imu_bridge')

        # --- PUBLISHERS ---
        self.pub_imu_diff = self.create_publisher(Imu, '/imu/diff', 10)
        self.pub_pitch    = self.create_publisher(Float32, '/imu/pitch', 10)
        self.pub_roll     = self.create_publisher(Float32, '/imu/roll', 10)

        # --- SERIAL CONNECTION ---
        self.serial_port = None
        try:
            self.serial_port = serial.Serial(port, baud, timeout=0.1)
        except Exception as e:
            self.get_logger().error(f"Serial Connection Error: {e}")
        
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

        # --- COVARIANCE CONSTANTS (The "Trust" Math) ---
        # Values are variances (standard deviation squared)
        
        # 1. Accelerometer Noise: ~0.05 m/s^2 error
        ACCEL_COV = 0.0025 
        
        # 2. Gyro Noise: ~0.02 rad/s error
        GYRO_COV = 0.0004 
        
        # 3. Orientation Noise: Filtered pitch/roll is fairly accurate
        ORIENT_COV = 0.001 

        self.linear_accel_cov = [
            ACCEL_COV, 0.0, 0.0,
            0.0, ACCEL_COV, 0.0,
            0.0, 0.0, ACCEL_COV
        ]

        self.angular_vel_cov = [
            GYRO_COV, 0.0, 0.0,
            0.0, GYRO_COV, 0.0,
            0.0, 0.0, GYRO_COV
        ]
        
        self.orientation_cov = [
            ORIENT_COV, 0.0, 0.0,
            0.0, ORIENT_COV, 0.0,
            0.0, 0.0, ORIENT_COV
        ]

        self.create_timer(0.01, self.read_serial_data)
        self.get_logger().info("⚠️  KEEP ROBOT STILL! Calibrating (3s)...")

    def read_serial_data(self):
        if self.serial_port is None: return
        try:
            raw_line = self.serial_port.readline().decode('utf-8', errors='replace').strip()
        except: return
        if not raw_line: return
        
        data_parts = raw_line.split(',')
        if len(data_parts) != 12: return

        try:
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
                    
                    self.bias_accel_left[2]  = 0.0
                    self.bias_accel_right[2] = 0.0
                    
                    self.is_calibrated = True
                    self.last_timestamp = self.get_clock().now()
                    self.get_logger().info("✅ CALIBRATION COMPLETE.")
                return

            # --- 2. BIAS CORRECTION ---
            accel_left  -= self.bias_accel_left
            gyro_left   -= self.bias_gyro_left 
            accel_right -= self.bias_accel_right
            gyro_right  -= self.bias_gyro_right 
            
            accel_avg = (accel_left + accel_right) / 2.0
            gyro_avg  = (gyro_left + gyro_right) / 2.0

            # --- 3. FILTERING ---
            current_ros_time = self.get_clock().now()
            dt = (current_ros_time - self.last_timestamp).nanoseconds / 1e9
            self.last_timestamp = current_ros_time
            if dt <= 0: return

            pitch_accel = math.atan2(-accel_avg[0], math.sqrt(accel_avg[1]**2 + accel_avg[2]**2))
            roll_accel  = math.atan2(accel_avg[1], accel_avg[2])
            
            pitch_rate = math.radians(gyro_avg[1])
            roll_rate  = math.radians(gyro_avg[0])

            alpha = 0.98
            self.pitch_filtered = alpha * (self.pitch_filtered + pitch_rate * dt) + (1.0 - alpha) * pitch_accel
            self.roll_filtered  = alpha * (self.roll_filtered  + roll_rate * dt)  + (1.0 - alpha) * roll_accel

            # --- 4. PUBLISH STANDARD IMU MESSAGE ---
            imu_msg = Imu()
            imu_msg.header.stamp = current_ros_time.to_msg()
            imu_msg.header.frame_id = "imu_link"
            
            # A. DATA
            imu_msg.linear_acceleration.x = accel_avg[0]
            imu_msg.linear_acceleration.y = accel_left[0] - accel_right[0] # Diff Accel
            imu_msg.linear_acceleration.z = accel_avg[2] # Gravity + Vertical

            imu_msg.angular_velocity.x = math.radians(gyro_avg[0])
            imu_msg.angular_velocity.y = math.radians(gyro_avg[1])
            imu_msg.angular_velocity.z = math.radians(gyro_avg[2]) # Yaw Rate

            # B. ORIENTATION (Quaternion from Pitch/Roll)
            # We assume Yaw = 0.0 here because Bridge doesn't track absolute heading (GNSS/EKF does that)
            q = self.euler_to_quaternion(self.roll_filtered, self.pitch_filtered, 0.0)
            imu_msg.orientation = q

            # C. COVARIANCE (Crucial for EKF)
            imu_msg.linear_acceleration_covariance = self.linear_accel_cov
            imu_msg.angular_velocity_covariance    = self.angular_vel_cov
            imu_msg.orientation_covariance         = self.orientation_cov
            
            self.pub_imu_diff.publish(imu_msg)

            # Publish Debug Topics
            p = Float32(); p.data = self.pitch_filtered
            r = Float32(); r.data = self.roll_filtered
            self.pub_pitch.publish(p)
            self.pub_roll.publish(r)

        except ValueError: pass

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