#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from math import sin, cos, pi
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped

class VirtualRobot(Node):
    def __init__(self):
        super().__init__('virtual_robot')

        # Publishers
        self.pub_imu = self.create_publisher(Imu, '/imu/diff', 10)
        self.pub_gps = self.create_publisher(PoseWithCovarianceStamped, '/gps/enu_pose', 10)
        self.pub_truth = self.create_publisher(Odometry, '/ground_truth', 10)
        self.pub_wheel = self.create_publisher(Float32, '/wheel_speed', 10)

        # Simulation Settings
        self.dt = 0.02 # 50 Hz
        self.t = 0.0
        self.steps = 0
        self.max_steps = 700000 # Stop after this

        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 2.0 

        self.create_timer(self.dt, self.update)
        self.get_logger().info(f"🚀 VIRTUAL ROBOT LIVE. Simulating {self.max_steps} steps...")

    def update(self):
        if self.steps >= self.max_steps:
            return

        if self.t < 10.0:
            true_yaw_rate = 0.0 
        else:
            # After 5s, start the sine wave curves
            true_yaw_rate = 0.5 * sin(0.2 * (self.t - 5.0))
            
        self.yaw += true_yaw_rate * self.dt
        self.x += self.v * cos(self.yaw) * self.dt
        self.y += self.v * sin(self.yaw) * self.dt
        self.t += self.dt
        self.steps += 1
        
        # ... (Rest of the file is unchanged) ...
        # 1. Physics (Figure 8)
        true_yaw_rate = 0.5 * sin(0.2 * self.t)
        self.yaw += true_yaw_rate * self.dt
        self.x += self.v * cos(self.yaw) * self.dt
        self.y += self.v * sin(self.yaw) * self.dt
        self.t += self.dt
        self.steps += 1

        now = self.get_clock().now().to_msg()

        # 2. Publish Ground Truth (For Plotting)
        truth_msg = Odometry()
        truth_msg.header.stamp = now
        truth_msg.header.frame_id = "odom"
        truth_msg.pose.pose.position.x = self.x
        truth_msg.pose.pose.position.y = self.y
        truth_msg.twist.twist.linear.x = self.v
        truth_msg.twist.twist.angular.z = true_yaw_rate
        # Yaw to Quaternion
        truth_msg.pose.pose.orientation.z = sin(self.yaw/2.0)
        truth_msg.pose.pose.orientation.w = cos(self.yaw/2.0)
        self.pub_truth.publish(truth_msg)

        # 3. Publish Noisy IMU (For EKF)
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "imu_link"
        imu.angular_velocity.z = true_yaw_rate + np.random.normal(0, 0.01)
        # Fake accel noise
        imu.linear_acceleration.x = np.random.normal(0, 0.1) 
        self.pub_imu.publish(imu)
    
        # 4. Publish Noisy GPS (2 Hz)
        if self.steps % 25 == 0:
            gps = PoseWithCovarianceStamped()
            gps.header.stamp = now
            gps.header.frame_id = "odom"
            gps.pose.pose.position.x = self.x + np.random.normal(0, 1.5)
            gps.pose.pose.position.y = self.y + np.random.normal(0, 1.5)
            # Covariance
            gps.pose.covariance[0] = 1.5**2
            gps.pose.covariance[7] = 1.5**2
            self.pub_gps.publish(gps)
        
        # 5. Publish Wheel Speed (50Hz)
        # True speed + small noise (0.1 m/s)
        wheel_msg = Float32()
        wheel_msg.data = self.v + np.random.normal(0, 0.1)
        self.pub_wheel.publish(wheel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VirtualRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()