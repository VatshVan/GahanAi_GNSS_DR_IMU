#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from math import sin, cos, pi, sqrt
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion

class VirtualRobot(Node):
    def __init__(self):
        super().__init__('virtual_robot')

        # --- PUBLISHERS ---
        # We publish to /imu/diff to mimic the output of 'imu_bridge.py'
        self.pub_imu = self.create_publisher(Imu, '/imu/diff', 10)
        self.pub_gps = self.create_publisher(PoseWithCovarianceStamped, '/gps/enu_pose', 10)
        self.pub_truth = self.create_publisher(Odometry, '/ground_truth', 10)
        self.pub_wheel = self.create_publisher(Float32, '/wheel_speed', 10)

        # --- SIMULATION STATE ---
        self.dt = 0.02  # 50 Hz
        self.t = 0.0
        self.steps = 0
        self.max_steps = 700000 

        # Robot Physics State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 2.0  # Constant speed 2.0 m/s

        self.create_timer(self.dt, self.update)
        self.get_logger().info(f"🚀 VIRTUAL ROBOT LIVE. Dual-IMU Mode.")

    def update(self):
        if self.steps >= self.max_steps:
            return

        # --- 1. PHYSICS ENGINE (Drive Straight -> Then Curve) ---
        # This mimics the "Operational Procedure" needed for EKF alignment
        if self.t < 10.0:
            true_yaw_rate = 0.0  # Drive Straight
        else:
            # Figure-8 Pattern
            true_yaw_rate = 0.5 * sin(0.2 * (self.t - 10.0))
            
        # Integrate Position
        self.yaw += true_yaw_rate * self.dt
        self.x += self.v * cos(self.yaw) * self.dt
        self.y += self.v * sin(self.yaw) * self.dt
        self.t += self.dt
        self.steps += 1

        now = self.get_clock().now().to_msg()

        # --- 2. GENERATE RAW DUAL-IMU DATA ---
        # We simulate what the hardware sends to the bridge
        
        # True Acceleration (Centripetal only, since speed is constant)
        # a_lat = v * omega
        true_accel_lat = self.v * true_yaw_rate
        true_accel_fwd = 0.0 # Constant speed
        
        # SIMULATE LEFT IMU (Independent Noise)
        # In reality, Left IMU might see different accel due to turning radius, 
        # but for EKF testing, independent noise is sufficient.
        ax_l = true_accel_fwd + np.random.normal(0, 0.005)
        ay_l = true_accel_lat + np.random.normal(0, 0.005)
        gz_l = true_yaw_rate  + np.random.normal(0, 0.001)

        # SIMULATE RIGHT IMU (Independent Noise)
        ax_r = true_accel_fwd + np.random.normal(0, 0.005)
        ay_r = true_accel_lat + np.random.normal(0, 0.005)
        gz_r = true_yaw_rate  + np.random.normal(0, 0.001)

        # --- 3. BRIDGE PROCESSING LOGIC ---
        # This matches exactly what your 'imu_bridge.py' does
        
        # A. Average the Gyros (Noise Reduction)
        gyro_z_avg = (gz_l + gz_r) / 2.0
        
        # B. Average the Accelerometers (Noise Reduction)
        accel_x_avg = (ax_l + ax_r) / 2.0
        
        # C. Calculate Differential (The Special Feature)
        # Your bridge stores (Left_X - Right_X) in the Y-channel
        accel_diff_x = ax_l - ax_r 

        # --- 4. PUBLISH /imu/diff ---
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "imu_link"
        
        # Orientation (Quaternions)
        # We assume Pitch/Roll = 0 for simulation, only Yaw matters here
        # Note: Your bridge uses a filter for P/R, we just send clean Identity + Yaw
        # Actually, standard IMU messages usually don't carry Yaw unless it's an AHRS.
        # But your EKF uses Gyro Z, so orientation field is less critical here.
        imu.orientation.w = 1.0 
        
        # Angular Velocity
        imu.angular_velocity.z = gyro_z_avg
        
        # Linear Acceleration
        imu.linear_acceleration.x = accel_x_avg
        imu.linear_acceleration.y = accel_diff_x  # <--- DIFFERENTIAL DATA
        imu.linear_acceleration.z = 9.81 + np.random.normal(0, 0.001)          # Gravity
        
        # Covariance (Matches your Bridge settings)
        imu.linear_acceleration_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
        imu.angular_velocity_covariance    = [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0004]
        
        self.pub_imu.publish(imu)

        # --- 5. PUBLISH GROUND TRUTH (For Plotting) ---
        truth_msg = Odometry()
        truth_msg.header.stamp = now
        truth_msg.header.frame_id = "odom"
        truth_msg.pose.pose.position.x = self.x
        truth_msg.pose.pose.position.y = self.y
        truth_msg.twist.twist.linear.x = self.v
        truth_msg.twist.twist.angular.z = true_yaw_rate
        self.pub_truth.publish(truth_msg)
    
        # --- 6. PUBLISH NOISY GPS (2 Hz) ---
        if self.steps % 25 == 0:
            gps = PoseWithCovarianceStamped()
            gps.header.stamp = now
            gps.header.frame_id = "odom"
            # Add significant noise (1.5m) to test EKF fusion
            gps.pose.pose.position.x = self.x + np.random.normal(0, 1.5)
            gps.pose.pose.position.y = self.y + np.random.normal(0, 1.5)
            
            cov = 1.5**2
            gps.pose.covariance[0] = cov
            gps.pose.covariance[7] = cov
            self.pub_gps.publish(gps)
        
        # --- 7. PUBLISH WHEEL SPEED (50Hz) ---
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