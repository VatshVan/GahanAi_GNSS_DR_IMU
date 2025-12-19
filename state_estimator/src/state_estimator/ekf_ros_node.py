# """
# Extended Kalman Filter (EKF) ROS Node for GNSS/IMU State Estimation.
# This module implements a state estimator node that fuses GPS (GNSS) and IMU sensor data
# using an Extended Kalman Filter to provide robust position, velocity, and orientation
# estimates for autonomous systems. The EKF performs prediction steps using IMU accelerometer
# and gyroscope measurements, and correction steps using GPS position fixes with dynamic
# covariance weighting.
# **State Vector (5-DOF):**
#     - x, y: ENU coordinates (meters)
#     - yaw: heading angle (radians)
#     - velocity: forward speed (m/s)
#     - omega: yaw rate (rad/s)
# **Key Features:**
#     - GPS initialization with heading alignment via vehicle motion
#     - Adaptive covariance rejection gate for unreliable GPS fixes
#     - IMU preprocessing: gravity compensation, centrifugal acceleration correction
#     - Dynamic friction model for velocity decay during stationary periods
#     - Real-time TF broadcasting and multi-endpoint odometry publishing
#     - Throttled logging for production environments
# **Subscribers:**
#     - /imu/diff (Imu): Differential IMU measurements at 50-100 Hz
#     - /imu/pitch (Float32): Vehicle pitch angle for gravity decomposition
#     - /gps/enu_pose (PoseWithCovarianceStamped): GPS position with uncertainty
# **Publishers:**
#     - /odometry/ekf (Odometry): Estimated state at IMU rate
#     - /pose/ekf (PoseWithCovarianceStamped): State with diagonal covariance
#     - /position/ekf (PointStamped): Position-only output for diagnostics
#     - tf: base_link frame relative to odom
# **Dependencies:**
#     - rclpy: ROS 2 Python client library
#     - numpy: Numerical computation
#     - ekf_core.EKF: External EKF implementation module
# **Performance Characteristics:**
#     - Prediction latency: <10 ms per IMU sample
#     - GPS update processing: <5 ms
#     - Covariance matrix: 5×5 symmetric positive definite
#     - Numerical stability: Matrix inversion safeguarded with exception handling
# **Author:** VatshVan
# **License:** Proprietary - GahanAI
# """

# import rclpy
# from rclpy.node import Node
# import numpy as np
# from math import atan2, sin, cos, degrees, sqrt
# from sensor_msgs.msg import Imu
# from std_msgs.msg import Float32
# from nav_msgs.msg import Odometry
# # from vehicle_msgs.msg import VehicleMsg
# from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, PointStamped
# from tf2_ros import TransformBroadcaster
# from .ekf_core import EKF

# class EKFNode(Node):
#     def __init__(self):
#         super().__init__('ekf_node')
        
#         self.ROTATION_RADIUS = 0.15 
        
#         self.ekf = EKF()
#         self.last_msg_time = None
#         self.current_pitch = 0.0

#         # Subscribers
#         self.create_subscription(Imu, '/imu/diff', self.callback_imu_diff, 10)
#         self.create_subscription(Float32, '/imu/pitch', self.callback_pitch, 10)
#         self.create_subscription(PoseWithCovarianceStamped, '/gps/enu_pose', self.callback_gnss, 10)
        
#         # Publishers
#         self.pub_odom = self.create_publisher(Odometry, '/odometry/ekf', 50)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         self.pub_pose = self.create_publisher(
#             PoseWithCovarianceStamped, '/pose/ekf', 20
#         )

#         self.create_subscription(Float32, '/wheel_speed', self.callback_wheel, 10)
#         # self.create_subscription(
#         #     VehicleMsg,
#         #     '/vehicle',
#         #     self.callback_wheel,
#         #     10
#         # )

#         self.pub_position = self.create_publisher(
#             PointStamped, '/position/ekf', 20
#         )

#         self.get_logger().info("--- EKF STARTED ---")

#     def callback_wheel(self, msg):
#         speed = msg.data
#         # Variance for wheel speed (0.1 m/s noise -> 0.01 variance)
#         # We trust this A LOT more than GPS speed.
#         var_speed = 0.1 ** 2 
#         self.ekf.update_wheel_speed(speed, var_speed)
    
#     def callback_pitch(self, msg): 
#         self.current_pitch = msg.data

#     def callback_gnss(self, msg):
#         x_gps = msg.pose.pose.position.x
#         y_gps = msg.pose.pose.position.y
        
#         # Get Time in seconds
#         current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

#         # ... (Variance extraction remains the same) ...
#         var_x = msg.pose.covariance[0]
#         var_y = msg.pose.covariance[7]
#         if var_x > 20.0: return

#         # --- 1. INITIALIZATION LOGIC ---
#         if not self.ekf.is_yaw_initialized:
#             # PASS TIME HERE
#             if self.ekf.check_alignment(x_gps, y_gps, current_time):
#                 self.get_logger().info(f"ALIGNED! Speed: {self.ekf.state[3]:.2f} m/s")
#             else:
#                 self.get_logger().info(f"CALIBRATING... {self.ekf.start_gps_x} to {x_gps}")
#             return

#         # --- 2. UPDATE STEP ---
#         # Note: Your new class takes a 2x2 R matrix directly
#         R_gps = np.array([
#             [var_x, 0.0],
#             [0.0, var_y]
#         ])
        
#         # Call the specific GPS helper in your new class
#         # --- Position update ---
#         self.ekf.update_gps(x_gps, y_gps, R_gps)

#         # --- GPS-derived heading update ---
#         now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

#         if self.ekf.last_gps_x is not None:
#             dx = x_gps - self.ekf.last_gps_x
#             dy = y_gps - self.ekf.last_gps_y
#             dt = now - self.ekf.last_gps_time

#             if dt > 0:
#                 speed = sqrt(dx*dx + dy*dy) / dt

#                 # --- EXISTING HEADING LOGIC ---
#                 if speed > self.ekf.min_gps_heading_speed:
#                     yaw_gps = atan2(dy, dx)
#                     var_yaw = np.deg2rad(10.0) ** 2
#                     self.ekf.update_gps_heading(yaw_gps, var_yaw)

#                 # --- 🔥 NEW: GPS VELOCITY FUSION 🔥 ---
#                 # This fixes the "Stuck at 0" bug.
#                 # We calculate speed from GPS and feed it to the filter.
#                 # Variance is high (16.0) because GPS speed is noisy, 
#                 # but it's enough to pull the EKF up to 2.0 m/s.
#                 var_speed_gps = 18.0 
#                 self.ekf.update_wheel_speed(speed, var_speed_gps)

#         # Store GPS history
#         self.ekf.last_gps_x = x_gps
#         self.ekf.last_gps_y = y_gps
#         self.ekf.last_gps_time = now
        
#         status = "RTK FIXED" if var_x < 0.01 else ("RTK FLOAT" if var_x < 1.0 else "3D FIX")
#         self.get_logger().info(f"UPDATE [{status}] | Pos: {x_gps:.1f}, {y_gps:.1f}", throttle_duration_sec=2.0)

#     def callback_imu_diff(self, msg: Imu):
#         current_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
#         if self.last_msg_time is None:
#             self.last_msg_time = current_time_sec; return
#         dt = current_time_sec - self.last_msg_time
#         self.last_msg_time = current_time_sec
#         if dt <= 0: return

#         accel_fwd_raw = msg.linear_acceleration.x
#         yaw_rate      = msg.angular_velocity.z
        
#         # Physics Correction
#         gravity_vector_x = 9.81 * sin(self.current_pitch) 
#         accel_gravity_corrected = accel_fwd_raw + gravity_vector_x 
#         accel_centrifugal = (yaw_rate ** 2) * self.ROTATION_RADIUS
#         accel_final = accel_gravity_corrected - accel_centrifugal

#         if abs(accel_final) < 0.05:
#             accel_final = 0.0
#             # Friction decay on velocity state
#             # self.ekf.state[3] *= 0.8 
#             if abs(self.ekf.state[3]) < 0.05: self.ekf.state[3] = 0.0

#         # --- PREDICT STEP ---
#         # Note: Method name changed from 'predict_state' to 'predict'
#         self.ekf.predict(accel_final, yaw_rate, dt)
        
#         self.publish_odometry(msg.header.stamp)

#     # def callback_wheel(self, msg: VehicleMsg):
#     #     v_wheel = msg.vehicle_speed_mps

#     #     # Reject nonsense
#     #     if v_wheel < 0.0 or v_wheel > 15.0:
#     #         return

#     #     # Wheel slip → moderate trust
#     #     var_v = 0.2 ** 2   # (m/s)^2

#     #     self.ekf.update_wheel_speed(v_wheel, var_v)

#     def publish_odometry(self, stamp):
#         state = self.ekf.get_current_state()
#         pos_x, pos_y, yaw, vel, omega = state
        
#         # TF Broadcasting
#         t = TransformStamped()
#         t.header.stamp = stamp
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_link'
#         t.transform.translation.x = float(pos_x)
#         t.transform.translation.y = float(pos_y)
#         t.transform.rotation.z = sin(yaw / 2.0)
#         t.transform.rotation.w = cos(yaw / 2.0)
#         self.tf_broadcaster.sendTransform(t)
        
#         # Odometry Publishing
#         odom = Odometry()
#         odom.header.stamp = stamp
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id = 'base_link'
#         odom.pose.pose.position.x = float(pos_x)
#         odom.pose.pose.position.y = float(pos_y)
#         odom.pose.pose.orientation = t.transform.rotation
#         odom.twist.twist.linear.x = float(vel)
#         odom.twist.twist.angular.z = float(omega)
#         self.pub_odom.publish(odom)

#         pose_msg = PoseWithCovarianceStamped()
#         pose_msg.header.stamp = stamp
#         pose_msg.header.frame_id = 'odom'

#         pose_msg.pose.pose.position.x = float(pos_x)
#         pose_msg.pose.pose.position.y = float(pos_y)
#         pose_msg.pose.pose.orientation.z = sin(yaw / 2.0)
#         pose_msg.pose.pose.orientation.w = cos(yaw / 2.0)

#         pose_msg.pose.covariance[0]  = self.ekf.P[0, 0]   # x
#         pose_msg.pose.covariance[7]  = self.ekf.P[1, 1]   # y
#         pose_msg.pose.covariance[35] = self.ekf.P[2, 2]   # yaw

#         self.pub_pose.publish(pose_msg)

#         position_msg = PointStamped()
#         position_msg.header.stamp = stamp
#         position_msg.header.frame_id = 'odom'
#         position_msg.point.x = float(pos_x)
#         position_msg.point.y = float(pos_y)
#         position_msg.point.z = 0.0
#         self.pub_position.publish(position_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = EKFNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__': main()



# _______________________________________________________________________________-
# _______________________________________________________________________________-
# _______________________________________________________________________________-
# _______________________________________________________________________________-
# _______________________________________________________________________________-
# _______________________________________________________________________________-
# _______________________________________________________________________________-
# _______________________________________________________________________________-
# _______________________________________________________________________________-
# _______________________________________________________________________________-
# _______________________________________________________________________________-













# # import rclpy
# # from rclpy.node import Node
# # import numpy as np
# # from math import sin, cos, degrees
# # from sensor_msgs.msg import Imu
# # from std_msgs.msg import Float32
# # from nav_msgs.msg import Odometry
# # from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
# # from tf2_ros import TransformBroadcaster

# # # --- MATH HELPER CLASS ---
# # class EKF:
# #     def __init__(self):
# #         # State Vector: [pos_x, pos_y, yaw, velocity, yaw_rate]
# #         self.state = np.zeros(5)
        
# #         # --- STEP 3: COVARIANCE INITIALIZATION ---
# #         # P: Initial Uncertainty (Trust mechanism)
# #         self.P = np.diag([
# #             1.0,   # x
# #             1.0,   # y
# #             0.1,   # yaw
# #             1.0,   # velocity
# #             0.1    # yaw_rate
# #         ])

# #         # Q: Process Noise (Model Uncertainty per second)
# #         # We assume Velocity is noisy (0.5) so EKF looks to GPS to fix it.
# #         self.Q = np.diag([
# #             0.05,  # x
# #             0.05,  # y
# #             0.01,  # yaw
# #             0.5,   # velocity
# #             0.05   # yaw_rate
# #         ])

# #     def predict_state(self, accel_fwd, yaw_rate, dt):
# #         x, y, yaw, v, omega = self.state
        
# #         # --- 1. STATE PREDICTION (Physics) ---
# #         new_yaw = yaw + yaw_rate * dt
# #         new_v = v + accel_fwd * dt
# #         new_v *= 0.99 # Drag factor
        
# #         # Safety Clamping
# #         new_v = max(min(new_v, 5.0), -5.0)

# #         new_x = x + new_v * np.cos(yaw) * dt
# #         new_y = y + new_v * np.sin(yaw) * dt
        
# #         self.state = np.array([new_x, new_y, new_yaw, new_v, yaw_rate])

# #         # --- 2. COVARIANCE PREDICTION (Uncertainty Growth) ---
# #         # Jacobian F (Partial derivatives of state w.r.t state)
# #         F = np.eye(5)
        
# #         # Derivatives for Position vs Yaw/Velocity
# #         F[0, 2] = -v * np.sin(yaw) * dt  # dx/dyaw
# #         F[0, 3] = np.cos(yaw) * dt       # dx/dv
# #         F[1, 2] = v * np.cos(yaw) * dt   # dy/dyaw
# #         F[1, 3] = np.sin(yaw) * dt       # dy/dv
# #         F[2, 4] = dt                     # dyaw/domega

# #         # P_new = F * P * F_transpose + (Q * dt)
# #         # We scale Q by dt because noise accumulates with time.
# #         self.P = F @ self.P @ F.T + (self.Q * dt)

# #     def update_gps(self, z, R):
# #         # Observation Matrix H: We measure x (idx 0) and y (idx 1)
# #         H = np.array([
# #             [1, 0, 0, 0, 0],
# #             [0, 1, 0, 0, 0]
# #         ])

# #         # Innovation (Error between GPS and Prediction)
# #         y = z - H @ self.state
        
# #         # Innovation Covariance
# #         S = H @ self.P @ H.T + R
        
# #         # Kalman Gain
# #         try:
# #             K = self.P @ H.T @ np.linalg.inv(S)
# #         except np.linalg.LinAlgError:
# #             # Fallback if matrix singular (rare)
# #             return

# #         # Update State
# #         self.state = self.state + K @ y
        
# #         # Update Uncertainty
# #         self.P = (np.eye(5) - K @ H) @ self.P
    
# #     def get_current_state(self): return self.state

# # # --- ROS NODE ---
# # class EKFNode(Node):
# #     def __init__(self):
# #         super().__init__('ekf_node_diff_imu')
        
# #         self.ROTATION_RADIUS = 0.15 
        
# #         self.ekf = EKF()
# #         self.last_msg_time = None
# #         self.current_pitch = 0.0

# #         # --- SUBSCRIBERS ---
# #         # 1. IMU (Prediction)
# #         self.create_subscription(Imu, '/imu/diff', self.callback_imu_diff, 10)
# #         self.create_subscription(Float32, '/imu/pitch', self.callback_pitch, 10)
        
# #         # 2. GNSS (Correction) - STEP 4 Implemented Here
# #         self.create_subscription(PoseWithCovarianceStamped, '/gps/enu_pose', self.callback_gnss, 10)
        
# #         # --- PUBLISHERS ---
# #         self.pub_odom = self.create_publisher(Odometry, '/odometry/ekf', 50)
# #         self.tf_broadcaster = TransformBroadcaster(self)

# #         self.get_logger().info("--- EKF ACTIVE: IMU PREDICT + GNSS UPDATE ---")

# #     def callback_pitch(self, msg): 
# #         self.current_pitch = msg.data

# #     # --- STEP 4: GNSS CALLBACK ---
# #     def callback_gnss(self, msg):
# #         # 1. Extract Measurement Vector z = [x, y]
# #         x_gps = msg.pose.pose.position.x
# #         y_gps = msg.pose.pose.position.y
# #         z = np.array([x_gps, y_gps])
        
# #         # 2. Extract Covariance R (2x2)
# #         # msg.pose.covariance is a 36-element array.
# #         # Index 0 is Var(X), Index 7 is Var(Y).
# #         cov = msg.pose.covariance
# #         R = np.array([
# #             [cov[0], 0.0],
# #             [0.0, cov[7]]
# #         ])
        
# #         # 3. Call EKF Update
# #         self.ekf.update_gps(z, R)
        
# #         # Debug Log (Low throttle)
# #         self.get_logger().info(f"🛰️ GPS CORRECTION | Pos: {x_gps:.2f}, {y_gps:.2f}", throttle_duration_sec=2.0)

# #     # --- IMU CALLBACK ---
# #     def callback_imu_diff(self, msg: Imu):
# #         current_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
# #         if self.last_msg_time is None:
# #             self.last_msg_time = current_time_sec; return
# #         dt = current_time_sec - self.last_msg_time
# #         self.last_msg_time = current_time_sec
# #         if dt <= 0: return

# #         # Unpack Data
# #         accel_fwd_raw = msg.linear_acceleration.x
# #         yaw_rate      = msg.angular_velocity.z
        
# #         # Physics Correction
# #         gravity_vector_x = 9.81 * sin(self.current_pitch) 
# #         accel_gravity_corrected = accel_fwd_raw + gravity_vector_x 
# #         accel_centrifugal = (yaw_rate ** 2) * self.ROTATION_RADIUS
# #         accel_linear_net = accel_gravity_corrected - accel_centrifugal

# #         # Motion Logic (ZUPT)
# #         accel_final = accel_linear_net
# #         if abs(accel_final) < 0.15: 
# #             accel_final = 0.0
# #             self.ekf.state[3] *= 0.8 # Friction
# #             if abs(self.ekf.state[3]) < 0.05: self.ekf.state[3] = 0.0

# #         # EKF Prediction
# #         self.ekf.predict_state(accel_final, yaw_rate, dt)
# #         self.publish_odometry(msg.header.stamp)

# #     def publish_odometry(self, stamp):
# #         state = self.ekf.get_current_state()
# #         pos_x, pos_y, yaw, vel, omega = state
        
# #         t = TransformStamped()
# #         t.header.stamp = stamp
# #         t.header.frame_id = 'odom'
# #         t.child_frame_id = 'base_link'
# #         t.transform.translation.x = float(pos_x)
# #         t.transform.translation.y = float(pos_y)
# #         t.transform.rotation.z = sin(yaw / 2.0)
# #         t.transform.rotation.w = cos(yaw / 2.0)
# #         self.tf_broadcaster.sendTransform(t)
        
# #         odom = Odometry()
# #         odom.header.stamp = stamp
# #         odom.header.frame_id = 'odom'
# #         odom.child_frame_id = 'base_link'
# #         odom.pose.pose.position.x = float(pos_x)
# #         odom.pose.pose.position.y = float(pos_y)
# #         odom.pose.pose.orientation = t.transform.rotation
# #         odom.twist.twist.linear.x = float(vel)
# #         odom.twist.twist.angular.z = float(omega)
# #         self.pub_odom.publish(odom)

# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = EKFNode()
# #     rclpy.spin(node)
# #     node.destroy_node()
# #     rclpy.shutdown()

# # if __name__ == '__main__': main()




















# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# from std_msgs.msg import Float32
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
# from tf2_ros import TransformBroadcaster
# from math import sin, cos, sqrt, degrees, atan2
# import numpy as np
# from .ekf_core import EKF

# class EKFNode(Node):
#     def __init__(self):
#         super().__init__('ekf_node')
#         self.ekf = EKF()
#         self.last_msg_time = None
#         self.current_pitch = 0.0
#         self.ROTATION_RADIUS = 0.15 

#         # --- SUBSCRIBERS ---
#         self.create_subscription(Imu, '/imu/diff', self.callback_imu_diff, 10)
#         self.create_subscription(Float32, '/imu/pitch', self.callback_pitch, 10)
#         self.create_subscription(PoseWithCovarianceStamped, '/gps/enu_pose', self.callback_gnss, 10)
#         self.create_subscription(Float32, '/wheel_speed', self.callback_wheel, 10)
        
#         # --- PUBLISHERS ---
#         self.pub_odom = self.create_publisher(Odometry, '/odometry/ekf', 50)
#         self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/pose/ekf', 20)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         self.last_imu_time   = None
#         self.last_gps_time   = None
#         self.last_wheel_time = None

#         self.IMU_TIMEOUT   = 0.1 
#         self.GPS_TIMEOUT   = 1.0
#         self.WHEEL_TIMEOUT = 0.2

#         self.filter_ready = False
#         self.last_pitch_time = None

#         self.get_logger().info("--- EKF STARTED: WHEEL + GPS + IMU FUSION ---")

#     def callback_pitch(self, msg): 
#         self.current_pitch = msg.data
#         self.last_pitch_time = self.get_clock().now().nanoseconds * 1e-9

#     # def callback_wheel(self, msg):
#     #     self.last_wheel_time = self.get_clock().now().nanoseconds * 1e-9
#     #     speed = msg.data
#     #     now = self.get_clock().now().nanoseconds * 1e-9
        
#     #     # Check IMU timeout
#     #     if self.last_imu_time and (now - self.last_imu_time) > self.IMU_TIMEOUT:
#     #         return

#     #     var_speed = 0.1 ** 2 
#     #     self.ekf.update_wheel_speed(speed, var_speed)
#     def callback_wheel(self, msg):
#         self.last_wheel_time = self.get_clock().now().nanoseconds * 1e-9
#         speed = msg.data
#         now = self.get_clock().now().nanoseconds * 1e-9
        
#         # Check IMU timeout
#         if self.last_imu_time and (now - self.last_imu_time) > self.IMU_TIMEOUT:
#             return

#         # --- SLIP DETECTION START ---
#         # 1. Compare Wheel Speed vs. EKF Predicted Speed
#         predicted_speed = self.ekf.state[3]
        
#         # 2. Threshold: 1.0 m/s difference is impossible in 0.02s (That's 50m/s^2 accel)
#         if abs(speed - predicted_speed) > 1.0:
#             # self.get_logger().warn(f"SLIP REJECTED: Wheel={speed:.1f}, EKF={predicted_speed:.1f}", throttle_duration_sec=1.0)
#             return
#         # --- SLIP DETECTION END ---

#         var_speed = 0.1 ** 2 
#         self.ekf.update_wheel_speed(speed, var_speed)

#     def callback_gnss(self, msg):
#         # 1. FIX: Calculate 'now' FIRST
#         now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
#         # 2. FIX: Update last_gps_time AFTER using 'now'
#         self.last_gps_time = now

#         x_gps = msg.pose.pose.position.x
#         y_gps = msg.pose.pose.position.y
#         var_x = msg.pose.covariance[0]
#         var_y = msg.pose.covariance[7]
        
#         # Initialization
#         if not self.ekf.is_yaw_initialized:
#             if self.ekf.check_alignment(x_gps, y_gps, now):
#                 self.filter_ready = True
#                 self.get_logger().info(f"ALIGNED! Heading: {degrees(self.ekf.state[2]):.1f}")
#             else:
#                 self.get_logger().info(f"CALIBRATING... Drive 5m", throttle_duration_sec=1.0)
#             return

#         # Position Update
#         R_gps = np.diag([var_x, var_y]) * 3.0
#         self.ekf.update_gps(x_gps, y_gps, R_gps)

#         if self.last_imu_time is None:
#             return

#         # Heading Drift Correction
#         if self.ekf.last_gps_x is not None:
#             dx = x_gps - self.ekf.last_gps_x
#             dy = y_gps - self.ekf.last_gps_y
#             dt = now - self.ekf.last_gps_time
#             dist = sqrt(dx*dx + dy*dy)

#             if dt > 0 and dist > 0.5:
#                 speed_gps = dist / dt
#                 if speed_gps > 1.0:
#                     yaw_gps = atan2(dy, dx)
#                     self.ekf.update_gps_heading(yaw_gps, 5.0)

#         self.ekf.last_gps_x = x_gps
#         self.ekf.last_gps_y = y_gps
#         self.ekf.last_gps_time = now

#     def callback_imu_diff(self, msg):
#         now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
#         self.last_imu_time = now

#         if self.last_msg_time is None: self.last_msg_time = now; return
#         dt = now - self.last_msg_time
#         self.last_msg_time = now
#         if dt <= 0: return

#         if not self.filter_ready:
#             return
        
#         # 3. FIX: Define accel_raw BEFORE checking pitch time
#         accel_raw = msg.linear_acceleration.x
#         yaw_rate  = msg.angular_velocity.z

#         # Check for stale pitch data
#         if self.last_pitch_time is None or (now - self.last_pitch_time) > 0.2:
#             accel_net = accel_raw
#         else:
#             accel_net = accel_raw + (9.81 * sin(self.current_pitch)) - ((yaw_rate**2) * self.ROTATION_RADIUS)
        
#         if abs(accel_net) < 0.05: accel_net = 0.0
        
#         self.ekf.predict(accel_net, yaw_rate, dt)
#         self.publish_odometry(msg.header.stamp)

#     def publish_odometry(self, stamp):
#         s = self.ekf.state
#         t = TransformStamped()
#         t.header.stamp = stamp; t.header.frame_id = 'odom'; t.child_frame_id = 'base_link'
#         t.transform.translation.x, t.transform.translation.y = s[0], s[1]
#         t.transform.rotation.z = sin(s[2]/2.0); t.transform.rotation.w = cos(s[2]/2.0)
#         self.tf_broadcaster.sendTransform(t)
        
#         o = Odometry()
#         o.header = t.header; o.child_frame_id = t.child_frame_id
#         o.pose.pose.position.x, o.pose.pose.position.y = s[0], s[1]
#         o.pose.pose.orientation = t.transform.rotation
#         o.twist.twist.linear.x, o.twist.twist.angular.z = s[3], s[4]
#         self.pub_odom.publish(o)
        
#         p = PoseWithCovarianceStamped()
#         p.header = o.header; p.pose.pose = o.pose.pose
        
#         for i in range(36): p.pose.covariance[i] = 0.0

#         p.pose.covariance[0]  = self.ekf.P[0,0]   # x
#         p.pose.covariance[7]  = self.ekf.P[1,1]   # y
#         p.pose.covariance[35] = self.ekf.P[2,2]   # yaw
#         p.pose.covariance[14] = 999.0  # z
#         p.pose.covariance[21] = 999.0  # roll
#         p.pose.covariance[28] = 999.0  # pitch

#         self.pub_pose.publish(p)

# def main(args=None):
#     rclpy.init(args=args)
#     rclpy.spin(EKFNode())
#     rclpy.shutdown()

# if __name__ == '__main__': main()














import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
from math import sin, cos, sqrt, degrees, radians, atan2, asin, pi
import numpy as np
import math

# Make sure this import matches your file structure
from .ekf_core import EKF 

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        self.ekf = EKF()
        
        # --- CONFIGURATION ---
        self.MAX_SPEED_MPS = 10.0 
        self.ENABLE_MOTION_ALIGNMENT = True # If True, Yaw resets to match GPS track on first move
        
        # --- STATE VARIABLES ---
        self.last_msg_time = None
        self.gps_initialized = False
        self.filter_ready = False
        self.motion_aligned = False
        
        self.current_gps_speed = 0.0

        # --- QOS PROFILES ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- SUBSCRIBERS ---
        self.create_subscription(Imu, '/imu/data_raw', self.callback_imu, sensor_qos)
        self.create_subscription(Float32, '/gps/speed_kmph', self.callback_velocity, sensor_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/gps/enu_pose', self.callback_gnss, sensor_qos)
        # Note: We removed the mag_heading subscriber. We will use Motion Alignment instead.

        # --- PUBLISHERS ---
        self.pub_odom = self.create_publisher(Odometry, '/odometry/ekf', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("🚀 EKF STARTED. Waiting for GPS...")

    # 1. GPS HANDLER (Position Update + Motion Alignment)
    def callback_gnss(self, msg):
        x_gps = msg.pose.pose.position.x
        y_gps = msg.pose.pose.position.y
        var_x = msg.pose.covariance[0]
        var_y = msg.pose.covariance[7]

        # A. Initialization
        if not self.gps_initialized:
            self.ekf.state[0] = x_gps
            self.ekf.state[1] = y_gps
            self.gps_initialized = True
            self.filter_ready = True
            self.get_logger().info(f"✅ GPS INIT: Position set to X:{x_gps:.1f}, Y:{y_gps:.1f}")
            return

        if not self.filter_ready: return

        # B. Motion Alignment (The "Sawtooth" Fix)
        # If we are moving (>0.5 m/s) and haven't aligned yet, 
        # SNAP the Yaw to match the GPS vector.
        if self.ENABLE_MOTION_ALIGNMENT and not self.motion_aligned and self.current_gps_speed > 0.5:
            dx = x_gps - self.ekf.state[0]
            dy = y_gps - self.ekf.state[1]
            dist = sqrt(dx**2 + dy**2)
            
            if dist > 0.2: # Ensure we actually moved between frames
                track_yaw = atan2(dy, dx)
                self.ekf.state[2] = track_yaw # Force Yaw
                self.ekf.is_yaw_initialized = True
                self.motion_aligned = True
                self.get_logger().info(f"📐 MOTION ALIGNMENT: Yaw snapped to {degrees(track_yaw):.1f}°")

        # C. Standard Update
        # Trust GPS Position heavily (Low variance)
        R_gps = np.diag([0.5, 0.5]) 
        self.ekf.update_gps(x_gps, y_gps, R_gps)

    # 2. VELOCITY HANDLER
    def callback_velocity(self, msg):
        speed_mps = msg.data / 3.6
        self.current_gps_speed = speed_mps

        if not self.filter_ready: return

        # Trust GPS Speed ALMOST PERFECTLY.
        # This forces the Blue line to stick to the Green dots.
        var_speed = 0.01 
        self.ekf.update_wheel_speed(speed_mps, var_speed)

    # 3. PREDICTION STEP
    def callback_imu(self, msg: Imu):
        if not self.filter_ready: return

        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.last_msg_time is None: 
            self.last_msg_time = now
            return
            
        dt = now - self.last_msg_time
        self.last_msg_time = now
        
        if dt <= 0: return

        # --- A. YAW RATE ---
        # We only use the Gyro. We IGNORE the accelerometer for speed.
        yaw_rate  = msg.angular_velocity.z
        
        # --- B. ACCEL IGNORE ---
        # Force acceleration to 0. This stops the velocity spikes.
        # We rely 100% on GPS updates for speed changes.
        accel_net = 0.0

        # Predict
        self.ekf.predict(accel_net, yaw_rate, dt)

        # Publish
        self.publish_odometry(msg.header.stamp)

    def publish_odometry(self, stamp):
        s = self.ekf.state
        
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(s[0])
        t.transform.translation.y = float(s[1])
        t.transform.translation.z = 0.0
        
        t.transform.rotation.z = sin(s[2]/2.0)
        t.transform.rotation.w = cos(s[2]/2.0)
        
        self.tf_broadcaster.sendTransform(t)
        
        o = Odometry()
        o.header = t.header
        o.child_frame_id = t.child_frame_id
        o.pose.pose.position.x = float(s[0])
        o.pose.pose.position.y = float(s[1])
        o.pose.pose.orientation = t.transform.rotation
        
        o.twist.twist.linear.x = float(s[3])
        o.twist.twist.angular.z = float(s[4])
        
        self.pub_odom.publish(o)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(EKFNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()