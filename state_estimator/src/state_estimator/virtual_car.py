#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from math import sin, cos, pi, radians
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped

class VirtualRobotRigorous(Node):
    def __init__(self):
        super().__init__('virtual_robot_rigorous')

        # Publishers
        self.pub_imu = self.create_publisher(Imu, '/imu/diff', 10)
        self.pub_gps = self.create_publisher(PoseWithCovarianceStamped, '/gps/enu_pose', 10)
        self.pub_truth = self.create_publisher(Odometry, '/ground_truth', 10)
        self.pub_wheel = self.create_publisher(Float32, '/wheel_speed', 10)
        self.pub_pitch = self.create_publisher(Float32, '/imu/pitch', 10)

        # Simulation Config
        self.dt = 0.02 # 50 Hz
        self.t = 0.0
        self.steps = 0
        self.max_steps = 30000 # Extended for longer track (~600s max)

        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.pitch_angle = 0.0 
        
        # Bias Drift (Simulating imperfect IMU)
        self.gyro_bias = 0.002 

        self.create_timer(self.dt, self.update)
        self.get_logger().info("🔥 RIGOROUS 'GRAND PRIX' TEST STARTED!")

    def update(self):
        if self.steps >= self.max_steps: return

        # ---------------------------------------------------------
        # 1. SCENARIO ORCHESTRATOR (The Grand Prix Track)
        # ---------------------------------------------------------
        
        # Phase 0: Calibration (0-5s) - Still
        if self.t < 5.0:
            target_v = 0.0
            yaw_rate_cmd = 0.0
            scenario = "Calibration (Still)"

        # Phase 1: The Drag Strip (5-20s) - High Speed Straight
        elif self.t < 20.0:
            target_v = 4.0 # Fast!
            yaw_rate_cmd = 0.0
            scenario = "Drag Strip (High Speed)"

        # Phase 2: The Mountain Pass (20-50s) - Long Curve + Hill
        # Constant turn while pitched up. Tests Gravity Comp + Centrifugal.
        elif self.t < 50.0:
            target_v = 2.5
            yaw_rate_cmd = 0.4 # Constant Left Turn
            self.pitch_angle = radians(12.0) # Steep Hill
            scenario = "Mountain Pass (Hill + Turn)"

        # Phase 3: The Underground Chicane (50-80s) - S-Turns + NO GPS
        # Complex maneuvering without satellites. Tests Dead Reckoning.
        elif self.t < 80.0:
            self.pitch_angle = 0.0
            target_v = 2.0
            # S-Turn Logic: Sine wave steering
            yaw_rate_cmd = 0.6 * sin(0.5 * (self.t - 50.0)) 
            scenario = "Tunnel Chicane (NO GPS)"

        # Phase 4: The Icy Bridge (80-95s) - Straight + Wheel Slip
        elif self.t < 95.0:
            target_v = 2.0
            yaw_rate_cmd = 0.0
            scenario = "Icy Bridge (Slip)"
            
        # Phase 5: The Figure-8 Finish (95s+) - Tight maneuvering
        elif self.t < 130.0:
            target_v = 1.5
            # Figure-8 steering
            yaw_rate_cmd = 0.8 * sin(0.3 * (self.t - 95.0))
            scenario = "Figure-8 Finish"

        else:
            target_v = 0.0
            yaw_rate_cmd = 0.0
            scenario = "Finished"

        # ---------------------------------------------------------
        # 2. PHYSICS ENGINE (Ground Truth)
        # ---------------------------------------------------------
        # Inertia (P-Controller)
        if self.v < target_v: self.v += 0.05
        elif self.v > target_v: self.v -= 0.05

        self.yaw += yaw_rate_cmd * self.dt
        
        # 3D Motion (Horizontal component reduced by pitch)
        v_horizontal = self.v * cos(self.pitch_angle)
        
        self.x += v_horizontal * cos(self.yaw) * self.dt
        self.y += v_horizontal * sin(self.yaw) * self.dt
        self.t += self.dt
        self.steps += 1

        now = self.get_clock().now().to_msg()

        # ---------------------------------------------------------
        # 3. SENSOR GENERATION (Fault Injection)
        # ---------------------------------------------------------

        # --- A. IMU (Diff + Pitch) ---
        # Gravity bleeds into X-axis when pitched
        gravity_x = 9.81 * sin(self.pitch_angle) 
        
        # Noisy readings
        ax_l = np.random.normal(0, 0.05) + gravity_x 
        ax_r = np.random.normal(0, 0.05) + gravity_x
        gz_l = yaw_rate_cmd + self.gyro_bias + np.random.normal(0, 0.01)
        gz_r = yaw_rate_cmd + self.gyro_bias + np.random.normal(0, 0.01)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "imu_link"
        imu.angular_velocity.z = (gz_l + gz_r) / 2.0
        imu.linear_acceleration.x = (ax_l + ax_r) / 2.0
        imu.linear_acceleration.y = ax_l - ax_r 
        imu.linear_acceleration.z = 9.81 * cos(self.pitch_angle)
        
        # Standard Covariances
        imu.linear_acceleration_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
        imu.angular_velocity_covariance    = [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0004]
        self.pub_imu.publish(imu)

        # Publish Pitch
        p_msg = Float32(); p_msg.data = self.pitch_angle
        self.pub_pitch.publish(p_msg)

        # --- B. WHEEL SPEED (Slip Injection) ---
        wheel_reading = self.v + np.random.normal(0, 0.005)
        # Inject huge slip during "Icy Bridge" phase
        if "Icy Bridge" in scenario: 
            wheel_reading = self.v * 3.0 # Spinning at 3x speed!
            
        w_msg = Float32(); w_msg.data = wheel_reading + np.random.normal(0, 0.1)
        self.pub_wheel.publish(w_msg)

        # --- C. GPS (Tunnel Outage) ---
        # GPS works everywhere EXCEPT in the Tunnel
        if "Tunnel" not in scenario:
            if self.steps % 25 == 0: # 2Hz
                gps = PoseWithCovarianceStamped()
                gps.header.stamp = now
                gps.header.frame_id = "odom"
                gps.pose.pose.position.x = self.x + np.random.normal(0, 1.5)
                gps.pose.pose.position.y = self.y + np.random.normal(0, 1.5)
                gps.pose.covariance[0] = 1.5**2
                gps.pose.covariance[7] = 1.5**2
                self.pub_gps.publish(gps)

        # --- D. TRUTH ---
        truth = Odometry()
        truth.header.stamp = now
        truth.header.frame_id = "odom"
        truth.pose.pose.position.x = self.x
        truth.pose.pose.position.y = self.y
        truth.twist.twist.linear.x = self.v
        self.pub_truth.publish(truth)
        
        if self.steps % 50 == 0:
            self.get_logger().info(f"[{self.t:.1f}s] {scenario}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VirtualRobotRigorous())
    rclpy.shutdown()

if __name__ == '__main__': main()