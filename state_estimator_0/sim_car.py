# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import numpy as np
# from math import sin, cos, pi, sqrt
# from sensor_msgs.msg import Imu
# from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
# from nav_msgs.msg import Odometry
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# class SimSensorFeeder(Node):
#     def __init__(self):
#         super().__init__('sim_sensor_feeder')

#         # --- QOS PROFILE (MUST MATCH EKF NODE) ---
#         # The EKF expects "Best Effort" data (like UDP/WiFi).
#         # We must publish with the same settings or they won't talk.
#         sensor_qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.VOLATILE,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=10
#         )

#         # --- PUBLISHERS ---
#         self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', sensor_qos)
#         self.pub_gps_pos = self.create_publisher(PoseWithCovarianceStamped, '/gps/enu_pose', sensor_qos)
#         self.pub_gps_vel = self.create_publisher(TwistWithCovarianceStamped, '/gps/fix_velocity', sensor_qos)
        
#         # Ground truth is for Rviz, so default QoS is fine, but let's use sensor_qos for consistency
#         self.pub_truth = self.create_publisher(Odometry, '/ground_truth', sensor_qos)

#         # --- SIMULATION STATE ---
#         self.dt = 0.05  # 20 Hz
#         self.t = 0.0
#         self.x = 0.0
#         self.y = 0.0
#         self.yaw = 0.0
#         self.v = 0.0
        
#         self.timer = self.create_timer(self.dt, self.update_physics_and_sensors)
#         self.get_logger().info("🚗 SIMULATOR STARTED (QoS: Best Effort)")
#         self.get_logger().info("    -> Publishing /imu/data_raw")
#         self.get_logger().info("    -> Publishing /gps/enu_pose")
#         self.get_logger().info("    -> Publishing /gps/fix_velocity")

#     def update_physics_and_sensors(self):
#         # 1. PHYSICS (Figure-8)
#         self.t += self.dt
#         target_v = 4.0 + 2.0 * sin(0.2 * self.t)
#         yaw_rate = 0.5 * sin(0.4 * self.t)
        
#         self.v = target_v
#         self.yaw += yaw_rate * self.dt
#         self.x += self.v * cos(self.yaw) * self.dt
#         self.y += self.v * sin(self.yaw) * self.dt

#         now = self.get_clock().now().to_msg()

#         # 2. SENSORS
#         # IMU
#         imu_msg = Imu()
#         imu_msg.header.stamp = now
#         imu_msg.header.frame_id = "imu_link"
#         imu_msg.angular_velocity.z = yaw_rate + np.random.normal(0, 0.01)
#         self.pub_imu.publish(imu_msg)

#         # GPS POS
#         gps_msg = PoseWithCovarianceStamped()
#         gps_msg.header.stamp = now
#         gps_msg.header.frame_id = "map"
#         gps_noise = np.random.normal(0, 0.5)
#         gps_msg.pose.pose.position.x = self.x + gps_noise
#         gps_msg.pose.pose.position.y = self.y + gps_noise
#         gps_msg.pose.covariance[0] = 0.5**2
#         gps_msg.pose.covariance[7] = 0.5**2
#         self.pub_gps_pos.publish(gps_msg)

#         # GPS VEL
#         vel_msg = TwistWithCovarianceStamped()
#         vel_msg.header.stamp = now
#         vel_msg.header.frame_id = "map"
#         noisy_v = self.v + np.random.normal(0, 0.1) 
#         vx = noisy_v * cos(self.yaw)
#         vy = noisy_v * sin(self.yaw)
#         vel_msg.twist.twist.linear.x = vx
#         vel_msg.twist.twist.linear.y = vy
#         self.pub_gps_vel.publish(vel_msg)

#         # TRUTH
#         truth_msg = Odometry()
#         truth_msg.header.stamp = now
#         truth_msg.header.frame_id = "map"
#         truth_msg.pose.pose.position.x = self.x
#         truth_msg.pose.pose.position.y = self.y
#         self.pub_truth.publish(truth_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     rclpy.spin(SimSensorFeeder())
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()









# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import numpy as np
# from math import sin, cos, pi, sqrt
# from sensor_msgs.msg import Imu
# from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
# from nav_msgs.msg import Odometry
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# class SimLongitudinal(Node):
#     def __init__(self):
#         super().__init__('sim_longitudinal')

#         sensor_qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.VOLATILE,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=10
#         )

#         self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', sensor_qos)
#         self.pub_gps_pos = self.create_publisher(PoseWithCovarianceStamped, '/gps/enu_pose', sensor_qos)
#         self.pub_gps_vel = self.create_publisher(TwistWithCovarianceStamped, '/gps/fix_velocity', sensor_qos)
#         self.pub_truth = self.create_publisher(Odometry, '/ground_truth', sensor_qos)

#         # CONFIGURATION
#         self.SENSOR_DIST = 1.0  # Meters (Distance between Front and Rear IMU)
#         self.dt = 0.05
        
#         self.t = 0.0
#         self.x = 0.0; self.y = 0.0; self.yaw = 0.0
#         self.v = 0.0; self.omega = 0.0
#         self.last_omega = 0.0
        
#         self.timer = self.create_timer(self.dt, self.update)
#         self.get_logger().info(f"🏎️ LONGITUDINAL SIM STARTED (Separation: {self.SENSOR_DIST}m)")

#     def update(self):
#         self.t += self.dt
        
#         # 1. PHYSICS (S-Curve)
#         target_v = 5.0 + 2.0 * sin(0.3 * self.t)
#         # Oscillating steering
#         self.omega = 1.0 * sin(0.5 * self.t)
        
#         # Calculate Angular Acceleration (alpha)
#         # alpha = (current_omega - last_omega) / dt
#         alpha = (self.omega - self.last_omega) / self.dt
#         self.last_omega = self.omega
        
#         # Update Pose
#         self.v = target_v
#         self.yaw += self.omega * self.dt
#         self.x += self.v * cos(self.yaw) * self.dt
#         self.y += self.v * sin(self.yaw) * self.dt

#         now = self.get_clock().now().to_msg()

#         # 2. SENSOR PHYSICS (Front/Rear)
#         # Center Acceleration (Centripetal) = v * omega
#         a_centripetal = self.v * self.omega
        
#         # Tangential Acceleration component due to rotation
#         # Front IMU feels EXTRA lateral kick when angular accel is positive
#         # Rear IMU feels LESS lateral kick
#         half_dist = self.SENSOR_DIST / 2.0
#         a_lat_front = a_centripetal + (alpha * half_dist)
#         a_lat_rear  = a_centripetal - (alpha * half_dist)
        
#         # Add Noise
#         a_lat_front += np.random.normal(0, 0.1)
#         a_lat_rear  += np.random.normal(0, 0.1)
        
#         # BRIDGE LOGIC (Averaging)
#         # Average cancels out the alpha term! ( +alpha - alpha = 0 )
#         # Result is pure center centripetal acceleration.
#         accel_fused = (a_lat_front + a_lat_rear) / 2.0

#         # PUBLISH IMU
#         imu = Imu()
#         imu.header.stamp = now
#         imu.header.frame_id = "imu_link"
#         imu.angular_velocity.z = self.omega + np.random.normal(0, 0.01)
#         imu.linear_acceleration.y = accel_fused
#         imu.linear_acceleration.x = 0.0 
#         self.pub_imu.publish(imu)

#         # PUBLISH GPS
#         gps = PoseWithCovarianceStamped()
#         gps.header.stamp = now
#         gps.header.frame_id = "map"
#         gps.pose.pose.position.x = self.x + np.random.normal(0, 0.3)
#         gps.pose.pose.position.y = self.y + np.random.normal(0, 0.3)
#         gps.pose.covariance = [0.09 if i in [0,7] else 0.0 for i in range(36)]
#         self.pub_gps_pos.publish(gps)

#         # PUBLISH VELOCITY
#         vel = TwistWithCovarianceStamped()
#         vel.header.stamp = now
#         vel.header.frame_id = "map"
#         nv = self.v + np.random.normal(0, 0.1)
#         vel.twist.twist.linear.x = nv * cos(self.yaw)
#         vel.twist.twist.linear.y = nv * sin(self.yaw)
#         self.pub_gps_vel.publish(vel)

#         # TRUTH
#         truth = Odometry()
#         truth.header.stamp = now
#         truth.header.frame_id = "map"
#         truth.pose.pose.position.x = self.x
#         truth.pose.pose.position.y = self.y
#         truth.pose.pose.orientation.z = sin(self.yaw/2.0)
#         truth.pose.pose.orientation.w = cos(self.yaw/2.0)
#         self.pub_truth.publish(truth)

# def main(args=None):
#     rclpy.init(args=args)
#     rclpy.spin(SimLongitudinal())
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
























#!/usr/bin/env python3
"""
SimCar Grand Prix (Flat Complex Circuit)
========================================
- Path: A 'Bean' or 'Helmet' shaped race track (not just a circle).
- Terrain: Completely Flat (Z=0).
- Update Rate: Strict 20 Hz.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from math import sin, cos, pi, sqrt, atan2
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class SimGrandPrix(Node):
    def __init__(self):
        super().__init__('sim_grand_prix')

        # --- QOS PROFILE ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- PUBLISHERS ---
        self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', sensor_qos)
        self.pub_gps_pos = self.create_publisher(PoseWithCovarianceStamped, '/gps/enu_pose', sensor_qos)
        self.pub_gps_vel = self.create_publisher(TwistWithCovarianceStamped, '/gps/fix_velocity', sensor_qos)
        self.pub_truth = self.create_publisher(Odometry, '/ground_truth', sensor_qos)

        # --- CONFIGURATION ---
        self.SENSOR_DIST = 0.9  
        self.dt = 0.05          # 20 Hz
        
        # --- STATE ---
        self.t = 0.0
        self.x = 0.0; self.y = 0.0; self.z = 0.0
        self.v = 0.0; self.yaw = 0.0; self.pitch = 0.0
        self.omega = 0.0; self.last_omega = 0.0
        
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info("GRAND PRIX SIM STARTED (Flat Track)")

    def get_track_position(self, t_val):
        """
        Generates a Grand Prix Style 'Bean' Curve.
        """
        # Slower time scaling for complex corners
        t = t_val * 0.15 
        
        scale = 20.0 
        
        # This formula creates a shape with a straight, a big curve, and a tight curve
        # x = cos(t) + 0.5 * cos(2t)
        # y = sin(t)
        
        x = scale * (cos(t) + 0.5 * cos(2*t))
        y = scale * (1.2 * sin(t))
        
        return x, y, 0.0

    def update(self):
        self.t += self.dt
        
        # 1. PATH GENERATION
        x_raw, y_raw, _ = self.get_track_position(self.t)
        x_next, y_next, _ = self.get_track_position(self.t + 0.1)
        
        # Derivatives
        dx = (x_next - x_raw) / 0.1
        dy = (y_next - y_raw) / 0.1
        
        # Update State
        self.x = x_raw
        self.y = y_raw
        self.z = 0.0
        self.v = sqrt(dx**2 + dy**2)
        self.yaw = atan2(dy, dx)
        self.pitch = 0.0

        # Yaw Rate (Omega)
        yaw_next = atan2(y_next - y_raw, x_next - x_raw)
        diff = yaw_next - self.yaw
        if diff > pi: diff -= 2*pi
        if diff < -pi: diff += 2*pi
        self.omega = diff / 0.1
        
        # Angular Accel
        alpha = (self.omega - self.last_omega) / self.dt
        self.last_omega = self.omega

        now = self.get_clock().now().to_msg()

        # 2. SENSOR PHYSICS
        a_centripetal = self.v * self.omega
        half_dist = self.SENSOR_DIST / 2.0
        a_lat_front = a_centripetal + (alpha * half_dist)
        a_lat_rear  = a_centripetal - (alpha * half_dist)
        
        a_lat_front += np.random.beta(0.1, 0.1)
        a_lat_rear  += np.random.beta(0.1, 0.1)
        accel_lat_fused = (a_lat_front + a_lat_rear) / 2.0
        
        # Publish IMU
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "imu_link"
        imu.angular_velocity.z = self.omega + np.random.normal(0, 0.01)
        imu.linear_acceleration.y = accel_lat_fused
        imu.linear_acceleration.x = 0.0 + np.random.normal(0, 0.1) 
        imu.linear_acceleration.z = 9.81
        self.pub_imu.publish(imu)

        # Publish GPS Pos
        gps = PoseWithCovarianceStamped()
        gps.header.stamp = now
        gps.header.frame_id = "map"
        gps.pose.pose.position.x = self.x + np.random.normal(0, 0.3)
        gps.pose.pose.position.y = self.y + np.random.normal(0, 0.3)
        gps.pose.pose.position.z = 0.0
        gps.pose.covariance[0] = 0.09; gps.pose.covariance[7] = 0.09
        self.pub_gps_pos.publish(gps)

        # Publish GPS Vel
        vel = TwistWithCovarianceStamped()
        vel.header.stamp = now
        vel.header.frame_id = "map"
        noisy_v = self.v + np.random.normal(0, 0.1)
        vel.twist.twist.linear.x = noisy_v * cos(self.yaw)
        vel.twist.twist.linear.y = noisy_v * sin(self.yaw)
        self.pub_gps_vel.publish(vel)

        # Publish Truth
        truth = Odometry()
        truth.header.stamp = now
        truth.header.frame_id = "map"
        truth.pose.pose.position.x = self.x
        truth.pose.pose.position.y = self.y
        truth.pose.pose.orientation.z = sin(self.yaw/2.0)
        truth.pose.pose.orientation.w = cos(self.yaw/2.0)
        self.pub_truth.publish(truth)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SimGrandPrix())
    rclpy.shutdown()

if __name__ == '__main__':
    main()





























# #!/usr/bin/env python3
# """
# SimCar Pivot (Stationary Rotation)
# ==================================
# - Position: Fixed at (0,0).
# - Action: Rotates Left/Right in place.
# - Purpose: Tests Yaw sensing without linear motion.
# """

# import rclpy
# from rclpy.node import Node
# import numpy as np
# from math import sin, cos, pi
# from sensor_msgs.msg import Imu
# from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
# from nav_msgs.msg import Odometry
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# class SimPivot(Node):
#     def __init__(self):
#         super().__init__('sim_pivot')

#         # --- QOS PROFILE ---
#         sensor_qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.VOLATILE,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=10
#         )

#         # --- PUBLISHERS ---
#         self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', sensor_qos)
#         self.pub_gps_pos = self.create_publisher(PoseWithCovarianceStamped, '/gps/enu_pose', sensor_qos)
#         self.pub_gps_vel = self.create_publisher(TwistWithCovarianceStamped, '/gps/fix_velocity', sensor_qos)
#         self.pub_truth = self.create_publisher(Odometry, '/ground_truth', sensor_qos)

#         # --- CONFIGURATION ---
#         self.SENSOR_DIST = 1.0  # Distance between Front/Rear IMU
#         self.dt = 0.05          # 20 Hz
        
#         # --- STATE ---
#         self.t = 0.0
#         self.yaw = 0.0
#         self.omega = 0.0
#         self.last_omega = 0.0
        
#         self.timer = self.create_timer(self.dt, self.update)
#         self.get_logger().info("🔄 PIVOT SIM STARTED (Stationary Rotation)")

#     def update(self):
#         self.t += self.dt
        
#         # 1. MOTION GENERATION (Pure Rotation)
#         # Scan back and forth +/- 90 degrees
#         self.yaw = (pi / 2.0) * sin(0.5 * self.t)
        
#         # Calculate Yaw Rate (Omega)
#         # derivative of sin(0.5t) is 0.5*cos(0.5t)
#         # omega = (pi/2) * 0.5 * cos(0.5*t)
#         current_omega = (pi / 2.0) * 0.5 * cos(0.5 * self.t)
        
#         # Calculate Angular Acceleration (Alpha)
#         alpha = (current_omega - self.omega) / self.dt
        
#         self.omega = current_omega
#         self.last_omega = current_omega

#         now = self.get_clock().now().to_msg()

#         # 2. SENSOR PHYSICS (Stationary)
#         # v = 0, so Centripetal (v*omega) is ZERO.
#         # The only acceleration is Tangential (alpha * radius)
        
#         half_dist = self.SENSOR_DIST / 2.0
        
#         # Front IMU: +Alpha pushes it Left
#         a_lat_front = (alpha * half_dist)
#         # Rear IMU: +Alpha pushes it Right (negative)
#         a_lat_rear  = -(alpha * half_dist)
        
#         # Add Noise
#         a_lat_front += np.random.normal(0, 0.05)
#         a_lat_rear  += np.random.normal(0, 0.05)
        
#         # Fuse (Should be near zero because they cancel out!)
#         accel_lat_fused = (a_lat_front + a_lat_rear) / 2.0
        
#         # PUBLISH IMU
#         imu = Imu()
#         imu.header.stamp = now
#         imu.header.frame_id = "imu_link"
#         imu.angular_velocity.z = self.omega + np.random.normal(0, 0.01)
#         imu.linear_acceleration.y = accel_lat_fused
#         imu.linear_acceleration.x = 0.0 + np.random.normal(0, 0.05) # Tiny engine rumble
#         imu.linear_acceleration.z = 9.81
#         self.pub_imu.publish(imu)

#         # PUBLISH GPS POS (Fixed at 0,0 with noise)
#         gps = PoseWithCovarianceStamped()
#         gps.header.stamp = now
#         gps.header.frame_id = "map"
#         gps.pose.pose.position.x = 0.0 + np.random.normal(0, 0.2)
#         gps.pose.pose.position.y = 0.0 + np.random.normal(0, 0.2)
#         gps.pose.pose.position.z = 0.0
#         gps.pose.covariance[0] = 0.04; gps.pose.covariance[7] = 0.04
#         self.pub_gps_pos.publish(gps)

#         # PUBLISH GPS VEL (Zero)
#         vel = TwistWithCovarianceStamped()
#         vel.header.stamp = now
#         vel.header.frame_id = "map"
#         vel.twist.twist.linear.x = 0.0 + np.random.normal(0, 0.05)
#         vel.twist.twist.linear.y = 0.0 + np.random.normal(0, 0.05)
#         self.pub_gps_vel.publish(vel)

#         # PUBLISH TRUTH
#         truth = Odometry()
#         truth.header.stamp = now
#         truth.header.frame_id = "map"
#         truth.pose.pose.position.x = 0.0
#         truth.pose.pose.position.y = 0.0
#         truth.pose.pose.orientation.z = sin(self.yaw/2.0)
#         truth.pose.pose.orientation.w = cos(self.yaw/2.0)
#         self.pub_truth.publish(truth)

# def main(args=None):
#     rclpy.init(args=args)
#     rclpy.spin(SimPivot())
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()