import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import numpy as np
from math import cos, sin, pi

class SimGenerator(Node):
    def __init__(self):
        super().__init__('sim_generator')
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.gps_pub = self.create_publisher(PoseWithCovarianceStamped, '/gps/enu_pose', 10)
        self.true_pub = self.create_publisher(PoseStamped, '/ground_truth', 10)
        
        self.timer = self.create_timer(0.01, self.update) # 100Hz
        self.t = 0.0
        # Truth states
        self.x, self.y, self.yaw, self.v = 0.0, 0.0, 0.0, 0.0
        self.accel = 0.2  # 0.2 m/s^2 constant acceleration
        self.yaw_rate = 0.1 # 0.1 rad/s constant turn

    def update(self):
        dt = 0.01
        self.t += dt
        
        # 1. Physics Engine (Ground Truth)
        self.v += self.accel * dt
        self.yaw += self.yaw_rate * dt
        self.x += self.v * cos(self.yaw) * dt
        self.y += self.v * sin(self.yaw) * dt

        # 2. Publish Ground Truth
        tp = PoseStamped()
        tp.header.stamp = self.get_clock().now().to_msg()
        tp.header.frame_id = "odom"
        tp.pose.position.x, tp.pose.position.y = self.x, self.y
        self.true_pub.publish(tp)

        # 3. Publish Noisy BNO085 Data (100Hz)
        imu = Imu()
        imu.header = tp.header
        imu.header.frame_id = "imu_link"
        # Fused Quaternion with noise
        imu.orientation.z = sin(self.yaw / 2.0) + np.random.normal(0, 0.001)
        imu.orientation.w = cos(self.yaw / 2.0) + np.random.normal(0, 0.001)
        imu.angular_velocity.z = self.yaw_rate + np.random.normal(0, 0.005)
        imu.linear_acceleration.x = self.accel + np.random.normal(0, 0.05)
        imu.orientation_covariance[0] = 3.0 # High accuracy
        self.imu_pub.publish(imu)

        # 4. Publish Noisy GPS Data (10Hz)
        if int(self.t * 100) % 10 == 0:
            gps = PoseWithCovarianceStamped()
            gps.header = tp.header
            gps.pose.pose.position.x = self.x + np.random.normal(0, 0.5) # 0.5m noise
            gps.pose.pose.position.y = self.y + np.random.normal(0, 0.5)
            self.gps_pub.publish(gps)

def main():
    rclpy.init(); rclpy.spin(SimGenerator()); rclpy.shutdown()