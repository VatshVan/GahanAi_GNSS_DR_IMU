#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from math import atan2, sqrt

class SimIMUBridge(Node):
    def __init__(self):
        super().__init__('sim_imu_bridge')

        # --- INPUT: Gazebo Topic ---
        # Remap this in your launch command if your Gazebo topic is different (e.g., /demo/imu)
        self.sub_imu = self.create_subscription(Imu, '/imu/raw', self.callback_imu, 10)
        
        # --- OUTPUT: EKF Topics ---
        # 1. Main Data stream
        self.pub_imu_diff = self.create_publisher(Imu, '/imu/diff', 10)
        # 2. Gravity Compensation stream
        self.pub_pitch    = self.create_publisher(Float32, '/imu/pitch', 10)

        self.get_logger().info("✅ SIM IMU BRIDGE STARTED")

    def callback_imu(self, msg):
        # 1. Calculate Pitch
        # Your EKF needs this to subtract gravity from the forward acceleration.
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        
        # Standard Pitch calculation from Gravity Vector
        pitch = atan2(-ax, sqrt(ay*ay + az*az))

        # 2. Publish Pitch
        pitch_msg = Float32()
        pitch_msg.data = pitch
        self.pub_pitch.publish(pitch_msg)

        # 3. Republish IMU for EKF
        # We pass the Gazebo data through, but ensure the frame_id is correct.
        out_msg = msg
        out_msg.header.frame_id = "imu_link" 
        
        # Ensure covariance is present (Gazebo usually handles this, but safety first)
        if out_msg.linear_acceleration_covariance[0] == 0.0:
             # Standard noise params if missing
             out_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
             out_msg.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]

        self.pub_imu_diff.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimIMUBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()