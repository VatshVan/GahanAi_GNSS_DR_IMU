#!/usr/bin/env python3
"""
ekf_stress_sim.py
-----------------
Extreme sensor + physics stress test for vehicle EKF.
Fixed: Now publishes True Velocity for proper Validation.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from math import sin, cos, tan, atan2
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from collections import deque

class EKFStressSim(Node):

    def __init__(self):
        super().__init__('ekf_stress_sim')

        # Vehicle
        self.L = 2.7
        self.dt = 0.01  # 100 Hz physics

        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.delta = 0.0

        # IMU error model
        self.gyro_bias = 0.01
        self.gyro_bias_tau = 300.0
        self.gyro_scale = 1.003

        # Wheel slip
        self.slip_factor = 1.0

        # GNSS latency buffer
        self.gps_buffer = deque(maxlen=50)

        # Publishers
        self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', 50)
        # Note: Topic name matches what EKF expects (/odom usually)
        self.pub_wheel = self.create_publisher(Odometry, '/odom', 20) 
        self.pub_lidar = self.create_publisher(Odometry, '/lidar_odom', 10)
        self.pub_gps = self.create_publisher(PoseWithCovarianceStamped, '/gps/enu_pose', 5)
        self.pub_truth = self.create_publisher(Odometry, '/ground_truth', 10)

        self.t = 0.0

        self.create_timer(self.dt, self.step)

        self.get_logger().info("🔥 EKF STRESS SIMULATOR STARTED")

    # ------------------------------

    def control_profile(self):
        if self.t < 10:
            if abs(self.t - 0.0) < 0.01: self.get_logger().info("Accelerating...")
            return 2.0, 0.0 # Slow start
        elif self.t < 30:
            if abs(self.t - 10.0) < 0.01: self.get_logger().info("Cruising...")
            return 8.0, 0.0
        elif self.t < 45:
            if abs(self.t - 30.0) < 0.01: self.get_logger().info("High Speed Turn...")
            return 10.0, 0.45
        elif self.t < 60:
            if abs(self.t - 45.0) < 0.01: self.get_logger().info("Decel + Sharp Turn...")
            return 3.0, -0.6
        elif self.t < 80:
            if abs(self.t - 60.0) < 0.01: self.get_logger().info("Steady Speed...")
            return 12.0, 0.0
        elif self.t < 100:
            if abs(self.t - 80.0) < 0.01: self.get_logger().info("Stopping...")
            return 0.0, 0.0
        else:
            if abs(self.t - 100.0) < 0.01: 
                self.get_logger().info("Simulation Complete.")
                rclpy.shutdown()
            return 0.0, 0.0

    # ------------------------------

    def step(self):

        v_cmd, steer_cmd = self.control_profile()

        # actuator lag
        self.v += 0.05 * (v_cmd - self.v)
        self.delta += 0.15 * (steer_cmd - self.delta)

        beta = atan2(tan(self.delta), 2.0)

        self.x += self.v * cos(self.yaw + beta) * self.dt
        self.y += self.v * sin(self.yaw + beta) * self.dt
        self.yaw += (self.v / self.L) * sin(beta) * self.dt * 2.0

        self.t += self.dt

        self.publish_truth()
        self.publish_imu()
        self.publish_wheel()
        self.publish_lidar()
        self.publish_gps()

    # ------------------------------

    def publish_truth(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        # 1. Position
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        
        # 2. Velocity (CRITICAL FIX: Validator needs this to compare!)
        msg.twist.twist.linear.x = self.v
        
        # 3. True Yaw Rate (optional but good for debugging)
        true_yaw_rate = (self.v / self.L) * tan(self.delta)
        msg.twist.twist.angular.z = true_yaw_rate
        
        self.pub_truth.publish(msg)

    # ------------------------------

    def publish_imu(self):
        true_yaw_rate = (self.v / self.L) * tan(self.delta)

        # Gauss-Markov bias drift
        self.gyro_bias += (-self.gyro_bias / self.gyro_bias_tau) * self.dt + np.random.normal(0, 0.0003)
        noise = np.random.normal(0, 0.02)
        meas = (true_yaw_rate * self.gyro_scale) + self.gyro_bias + noise

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angular_velocity.z = meas
        
        # Absolute Orientation (Perfect Magnetometer Simulation)
        # This helps check if your 'update_heading' logic works
        msg.orientation.w = cos(self.yaw / 2)
        msg.orientation.z = sin(self.yaw / 2)

        self.pub_imu.publish(msg)

    # ------------------------------

    def publish_wheel(self):
        # Simulate variable slip (e.g., hitting a wet patch)
        if int(self.t) % 15 == 0:
            self.slip_factor = np.random.uniform(0.7, 1.0)

        speed = self.v * self.slip_factor
        speed += np.random.normal(0, 0.15)

        speed = round(speed / 0.05) * 0.05  # quantization

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.twist.linear.x = speed

        self.pub_wheel.publish(msg)

    # ------------------------------

    def publish_lidar(self):
        # 10 Hz only
        if int(self.t / self.dt) % 10 != 0: return

        # 5% dropout (Sensor Failure)
        if np.random.rand() < 0.05: return

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.twist.twist.linear.x = self.v + np.random.normal(0, 0.03)
        msg.twist.twist.angular.z = (self.v / self.L) * tan(self.delta) + np.random.normal(0, 0.005)

        self.pub_lidar.publish(msg)

    # ------------------------------

    def publish_gps(self):
        # 5 Hz
        if int(self.t / self.dt) % 20 != 0: return

        # Outage (Tunnel)
        if np.random.rand() < 0.15: return

        x = self.x + np.random.normal(0, 1.5)
        y = self.y + np.random.normal(0, 1.5)

        # Multipath (Sudden Jump)
        if np.random.rand() < 0.03:
            x += np.random.choice([-8, 8])
            y += np.random.choice([-8, 8])

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y

        # Latency buffer (~0.3s delay)
        self.gps_buffer.append(msg)

        if len(self.gps_buffer) > 10:
            self.pub_gps.publish(self.gps_buffer.popleft())

def main(args=None):
    rclpy.init(args=args)
    node = EKFStressSim()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()