# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import numpy as np
# from math import sin, cos
# from sensor_msgs.msg import Imu
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster
# from collections import deque
# import math

# # --- MATH CLASS ---
# class EKF:
#     def __init__(self):
#         self.x = np.zeros(5)
#         self.P = np.eye(5) * 0.1
#         self.Q = np.diag([0.0, 0.0, 0.001, 0.01, 0.01])
#         self.accel_history = deque(maxlen=20) 
#         self.gyro_history = deque(maxlen=20)

#     def predict(self, alpha, omega, dt):
#         x, y, yaw, v, _ = self.x
#         new_yaw = yaw + omega * dt
#         new_x = x + v * np.cos(yaw) * dt
#         new_y = y + v * np.sin(yaw) * dt
#         self.x = np.array([new_x, new_y, new_yaw, v, omega])
        
#         F = np.eye(5)
#         F[0, 2] = -v * np.sin(yaw) * dt
#         F[0, 3] = np.cos(yaw) * dt
#         F[1, 2] = v * np.cos(yaw) * dt
#         F[1, 3] = np.sin(yaw) * dt
#         self.P = F @ self.P @ F.T + self.Q
    
#     def get_state(self): return self.x

#     def apply_velocity_constraint(self, sigma=0.2):
#         # pseudo-measurement: v ≈ current v
#         H = np.zeros((1, 5))
#         H[0, 3] = 1.0

#         R = np.array([[sigma**2]])

#         z = np.array([[self.x[3]]])
#         y = z - H @ self.x.reshape(-1,1)

#         S = H @ self.P @ H.T + R
#         K = self.P @ H.T @ np.linalg.inv(S)

#         self.x = self.x + (K @ y).flatten()
#         self.P = (np.eye(5) - K @ H) @ self.P

# # --- ROS NODE ---
# class EKFNode(Node):
#     def __init__(self):
#         super().__init__('ekf_node_diff_imu')
#         self.accel_history = deque(maxlen=20) 
#         self.gyro_history = deque(maxlen=20)

#         # 1. HARDCODED TOPIC (No params, no YAML confusion)
#         target_topic = '/imu/diff'

#         self.ekf = EKF()
#         self.last_imu_time = None
#         self.IMU_BASELINE = 0.30 
#         self.msg_count = 0

#         # 2. SUBSCRIBER
#         self.create_subscription(Imu, target_topic, self.diff_cb, 10)

#         # 3. PUBLISHER
#         self.odom_pub = self.create_publisher(Odometry, '/odometry/ekf', 50)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # 4. HEARTBEAT TIMER (To prove the node is alive)
#         self.create_timer(1.0, self.check_status)

#         self.get_logger().info(f"HARDCODED MODE: Listening strictly to '{target_topic}'")

#     def check_status(self):
#         # This will print every second if the node is alive but getting no data
#         if self.msg_count == 0:
#             self.get_logger().warn("Waiting for IMU data... (Check Bridge!)")
#         else:
#             # If we are getting data, we don't need to spam, just show total count
#             pass
    
#     def apply_velocity_constraint(self, sigma=0.2):
#         # pseudo-measurement: v ≈ current v
#         H = np.zeros((1, 5))
#         H[0, 3] = 1.0

#         R = np.array([[sigma**2]])

#         z = np.array([[self.x[3]]])
#         y = z - H @ self.x.reshape(-1,1)

#         S = H @ self.P @ H.T + R
#         K = self.P @ H.T @ np.linalg.inv(S)

#         self.x = self.x + (K @ y).flatten()
#         self.P = (np.eye(5) - K @ H) @ self.P

#     def diff_cb(self, msg: Imu):
#         # Time calc
#         current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
#         if self.last_imu_time is None:
#             self.last_imu_time = current_time
#             self.current_pitch = 0.0
#             return
#         dt = current_time - self.last_imu_time
#         self.last_imu_time = current_time
#         if dt <= 0: return

#         # Unpack Data
#         omega_z = msg.orientation.x          
#         omega_y = msg.orientation.y          
#         raw_accel_x = msg.orientation_covariance[0] 
#         raw_accel_z = msg.orientation_covariance[1] 
        
#         delta_ax = msg.linear_acceleration.x 
#         alpha_z = delta_ax / self.IMU_BASELINE

#         # --- 1. POPULATE HISTORY (For Jitter Check) ---
#         # We need this data to calculate variance later
#         self.accel_history.append(raw_accel_x)
#         self.gyro_history.append(omega_z)

#         # --- 2. COMPLEMENTARY FILTER (Pitch) ---
#         pitch_accel = np.arctan2(raw_accel_x, raw_accel_z)
        
#         # Adaptive Gain: If G-Force is weird (turning/accelerating), trust Gyro more.
#         # This prevents the "Slow Tilt" bug you mentioned earlier.
#         g_force = math.sqrt(raw_accel_x**2 + raw_accel_z**2)
#         alpha = 0.02
#         if abs(g_force - 9.81) > 0.5: alpha = 0.005 

#         if not hasattr(self, 'current_pitch'): self.current_pitch = 0.0
#         self.current_pitch = (1.0 - alpha) * (self.current_pitch + omega_y * dt) + \
#                              (alpha) * (pitch_accel)

#         # --- 3. GRAVITY COMPENSATION ---
#         gravity_effect = 9.81 * sin(self.current_pitch)
#         corrected_accel_x = raw_accel_x - gravity_effect 

#         # --- 4. STATIONARY CHECK (Protected) ---
#         is_stationary = False
#         current_vel = self.ekf.x[3]
        
#         # [CRITICAL GUARD]: Only check variance if we are barely moving (< 1.0 m/s)
#         # This protects against the "Highway Freeze" bug.
#         if abs(current_vel) < 1.0 and len(self.accel_history) == 20:
            
#             acc_var = np.var(self.accel_history)
#             gyro_var = np.var(self.gyro_history)
            
#             # If variance is low, we are physically stopped (engine idle/parked)
#             if acc_var < 0.005 and gyro_var < 0.0005:
#                 is_stationary = True

#         # --- 5. MOTION LOGIC ---
#         if is_stationary:
#             # We are definitely stopped. Lock everything to prevent drift.
#             self.ekf.x[3] = 0.0
#             omega_z = 0.0        # Lock Yaw
#             corrected_accel_x = 0.0
#         else:
#             # We are moving (Cruising OR Accelerating)
            
#             # Simple noise gate
#             if abs(corrected_accel_x) < 0.05:
#                 corrected_accel_x = 0.0
            
#             # --- MOTION CONSTRAINT: ACCELERATION CONSISTENCY ---
#             expected_accel = abs(self.ekf.x[3] * omega_z)

#             # If accel exists but no turning or prior velocity → reject it
#             if abs(corrected_accel_x) > 0.2 and expected_accel < 0.05:
#                 corrected_accel_x *= 0.2   # soft reject (DON'T zero hard)

#             # If yaw rate is near zero, acceleration must be small
#             if abs(omega_z) < 0.02 and abs(corrected_accel_x) > 0.1:
#                 corrected_accel_x *= 0.3

#             # Integrate
#             self.ekf.x[3] += corrected_accel_x * dt

#             if abs(corrected_accel_x) < 0.1 and abs(omega_z) < 0.05:
#                 self.ekf.apply_velocity_constraint(sigma=0.4)
            
#             # Very light air resistance (prevent infinite runaway)
#             self.ekf.x[3] *= 0.999

#         # --- DIAGNOSIS PRINTS ---
#         if self.accel_history: var_debug = np.var(self.accel_history)
#         else: var_debug = 0.0

#         self.get_logger().info(
#             f"Var: {var_debug:.4f} | "
#             f"Pitch: {np.degrees(self.current_pitch):.1f}° | "
#             f"Net: {corrected_accel_x:.2f} | "
#             f"Vel: {self.ekf.x[3]:.2f}", 
#             throttle_duration_sec=0.2
#         )

#         self.ekf.predict(alpha=alpha_z, omega=omega_z, dt=dt)
#         self.publish_odom(msg.header.stamp)

#     def publish_odom(self, stamp):
#         state = self.ekf.get_state()
#         x, y, yaw, v, omega = state

#         t = TransformStamped()
#         t.header.stamp = stamp
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_link'
#         t.transform.translation.x = float(x)
#         t.transform.translation.y = float(y)
#         t.transform.rotation.z = sin(yaw / 2.0)
#         t.transform.rotation.w = cos(yaw / 2.0)
#         self.tf_broadcaster.sendTransform(t)

#         odom = Odometry()
#         odom.header.stamp = stamp
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id = 'base_link'
#         odom.pose.pose.position.x = float(x)
#         odom.pose.pose.position.y = float(y)
#         odom.pose.pose.orientation = t.transform.rotation
#         odom.twist.twist.linear.x = float(v)
#         odom.twist.twist.angular.z = float(omega)
#         self.odom_pub.publish(odom)

# def main(args=None):
#     rclpy.init(args=args)
#     node = EKFNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
















# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import numpy as np
# from math import sin, cos
# from sensor_msgs.msg import Imu
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster
# from collections import deque
# import math

# # --- MATH CLASS (Updated) ---
# class EKF:
#     def __init__(self):
#         self.x = np.zeros(5)
#         self.P = np.eye(5) * 0.1
#         self.Q = np.diag([0.0, 0.0, 0.001, 0.01, 0.01])
#         self.accel_history = deque(maxlen=20) 
#         self.gyro_history = deque(maxlen=20)

#     def predict(self, alpha, omega, dt):
#         x, y, yaw, v, _ = self.x
#         new_yaw = yaw + omega * dt
#         new_x = x + v * np.cos(yaw) * dt
#         new_y = y + v * np.sin(yaw) * dt
#         self.x = np.array([new_x, new_y, new_yaw, v, omega])
        
#         F = np.eye(5)
#         F[0, 2] = -v * np.sin(yaw) * dt
#         F[0, 3] = np.cos(yaw) * dt
#         F[1, 2] = v * np.cos(yaw) * dt
#         F[1, 3] = np.sin(yaw) * dt
#         self.P = F @ self.P @ F.T + self.Q
    
#     def get_state(self): return self.x

#     def apply_velocity_constraint(self, sigma=0.2):
#         H = np.zeros((1, 5))
#         H[0, 3] = 1.0

#         R = np.array([[sigma**2]])

#         z = np.array([[self.x[3]]])
#         y = z - H @ self.x.reshape(-1,1)

#         S = H @ self.P @ H.T + R
#         K = self.P @ H.T @ np.linalg.inv(S)

#         self.x = self.x + (K @ y).flatten()
#         self.P = (np.eye(5) - K @ H) @ self.P

# # --- ROS NODE ---
# class EKFNode(Node):
#     def __init__(self):
#         super().__init__('ekf_node_diff_imu')
#         self.accel_history = deque(maxlen=20) 
#         self.gyro_history = deque(maxlen=20)

#         # 1. HARDCODED TOPIC
#         target_topic = '/imu/diff'

#         self.ekf = EKF()
#         self.last_imu_time = None
#         self.IMU_BASELINE = 0.30 
#         self.msg_count = 0

#         # 2. SUBSCRIBER
#         self.create_subscription(Imu, target_topic, self.diff_cb, 10)

#         # 3. PUBLISHER
#         self.odom_pub = self.create_publisher(Odometry, '/odometry/ekf', 50)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # 4. HEARTBEAT TIMER
#         self.create_timer(1.0, self.check_status)

#         self.get_logger().info(f"HARDCODED MODE: Listening strictly to '{target_topic}'")

#     def check_status(self):
#         if self.msg_count == 0:
#             self.get_logger().warn("Waiting for IMU data... (Check Bridge!)")
#         else:
#             pass

#     def diff_cb(self, msg: Imu):
#         # Time calc
#         current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
#         if self.last_imu_time is None:
#             self.last_imu_time = current_time
#             self.current_pitch = 0.0
#             return
#         dt = current_time - self.last_imu_time
#         self.last_imu_time = current_time
#         if dt <= 0: return

#         # Unpack Data
#         omega_z = msg.orientation.x          
#         omega_y = msg.orientation.y          
#         raw_accel_x = msg.orientation_covariance[0] 
#         raw_accel_z = msg.orientation_covariance[1] 
        
#         delta_ax = msg.linear_acceleration.x 
#         alpha_z = delta_ax / self.IMU_BASELINE

#         # --- 1. POPULATE HISTORY ---
#         self.accel_history.append(raw_accel_x)
#         self.gyro_history.append(omega_z)

#         # --- 2. COMPLEMENTARY FILTER (Pitch) ---
#         pitch_accel = np.arctan2(raw_accel_x, raw_accel_z)
        
#         g_force = math.sqrt(raw_accel_x**2 + raw_accel_z**2)
#         alpha = 0.02
#         if abs(g_force - 9.81) > 0.5: alpha = 0.005 

#         if not hasattr(self, 'current_pitch'): self.current_pitch = 0.0
#         self.current_pitch = (1.0 - alpha) * (self.current_pitch + omega_y * dt) + \
#                              (alpha) * (pitch_accel)

#         # --- 3. GRAVITY COMPENSATION ---
#         gravity_effect = 9.81 * sin(self.current_pitch)
#         corrected_accel_x = raw_accel_x - gravity_effect 

#         # --- 4. STATIONARY CHECK ---
#         is_stationary = False
#         current_vel = self.ekf.x[3]
        
#         if abs(current_vel) < 1.0 and len(self.accel_history) == 20:
#             acc_var = np.var(self.accel_history)
#             gyro_var = np.var(self.gyro_history)
#             if acc_var < 0.005 and gyro_var < 0.0005:
#                 is_stationary = True

#         # --- 5. MOTION LOGIC ---
#         if is_stationary:
#             self.ekf.x[3] = 0.0
#             omega_z = 0.0        
#             corrected_accel_x = 0.0
#         else:
#             # Simple noise gate
#             if abs(corrected_accel_x) < 0.05:
#                 corrected_accel_x = 0.0
            
#             # --- MOTION CONSTRAINT: ACCELERATION CONSISTENCY ---
#             expected_accel = abs(self.ekf.x[3] * omega_z)

#             # If accel exists but no turning or prior velocity → reject it
#             if abs(corrected_accel_x) > 0.2 and expected_accel < 0.05:
#                 corrected_accel_x *= 0.2   

#             # If yaw rate is near zero, acceleration must be small
#             if abs(omega_z) < 0.02 and abs(corrected_accel_x) > 0.1:
#                 corrected_accel_x *= 0.3

#             # Integrate
#             self.ekf.x[3] += corrected_accel_x * dt

#             if abs(corrected_accel_x) < 0.1 and abs(omega_z) < 0.05:
#                 # Now this works because the function is inside EKF class
#                 self.ekf.apply_velocity_constraint(sigma=0.4)
            
#             # Very light air resistance
#             self.ekf.x[3] *= 0.999

#         # --- DIAGNOSIS PRINTS ---
#         if self.accel_history: var_debug = np.var(self.accel_history)
#         else: var_debug = 0.0

#         self.get_logger().info(
#             f"Var: {var_debug:.4f} | "
#             f"Pitch: {np.degrees(self.current_pitch):.1f}° | "
#             f"Net: {corrected_accel_x:.2f} | "
#             f"Vel: {self.ekf.x[3]:.2f}", 
#             throttle_duration_sec=0.2
#         )

#         self.ekf.predict(alpha=alpha_z, omega=omega_z, dt=dt)
#         self.publish_odom(msg.header.stamp)

#     def publish_odom(self, stamp):
#         state = self.ekf.get_state()
#         x, y, yaw, v, omega = state

#         t = TransformStamped()
#         t.header.stamp = stamp
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_link'
#         t.transform.translation.x = float(x)
#         t.transform.translation.y = float(y)
#         t.transform.rotation.z = sin(yaw / 2.0)
#         t.transform.rotation.w = cos(yaw / 2.0)
#         self.tf_broadcaster.sendTransform(t)

#         odom = Odometry()
#         odom.header.stamp = stamp
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id = 'base_link'
#         odom.pose.pose.position.x = float(x)
#         odom.pose.pose.position.y = float(y)
#         odom.pose.pose.orientation = t.transform.rotation
#         odom.twist.twist.linear.x = float(v)
#         odom.twist.twist.angular.z = float(omega)
#         self.odom_pub.publish(odom)

# def main(args=None):
#     rclpy.init(args=args)
#     node = EKFNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




# Final till now Dec 15

# import rclpy
# from rclpy.node import Node
# import numpy as np
# from math import sin, cos, degrees
# from sensor_msgs.msg import Imu
# from std_msgs.msg import Float32  
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster

# # --- MATH HELPER CLASS ---
# class EKF:
#     def __init__(self):
#         # State Vector: [pos_x, pos_y, yaw_angle, velocity, yaw_rate]
#         self.state = np.zeros(5) 

#     def predict_state(self, accel_fwd, yaw_rate, dt):
#         pos_x, pos_y, yaw, velocity, _ = self.state
        
#         # 1. Integrate Yaw
#         new_yaw = yaw + yaw_rate * dt
        
#         # 2. Integrate Velocity (with Drag Factor)
#         new_velocity = velocity + accel_fwd * dt
#         new_velocity *= 0.99 # Simple Air Drag
        
#         # 3. Safety Clamping (Max Speed 5 m/s)
#         if new_velocity > 5.0: new_velocity = 5.0
#         if new_velocity < -5.0: new_velocity = -5.0

#         # 4. Integrate Position (Dead Reckoning)
#         new_pos_x = pos_x + new_velocity * np.cos(new_yaw) * dt
#         new_pos_y = pos_y + new_velocity * np.sin(new_yaw) * dt
        
#         self.state = np.array([new_pos_x, new_pos_y, new_yaw, new_velocity, yaw_rate])
    
#     def get_current_state(self): return self.state

# # --- ROS NODE ---
# class EKFNode(Node):
#     def __init__(self):
#         super().__init__('ekf_node_diff_imu')
        
#         # --- CONFIGURATION ---
#         self.ROTATION_RADIUS = 0.15  # Meters (Distance from Pivot to Sensor)
        
#         # --- STATE ---
#         self.ekf = EKF()
#         self.last_msg_time = None
#         self.current_pitch = 0.0

#         # --- SUBSCRIBERS ---
#         self.sub_imu_diff = self.create_subscription(Imu, '/imu/diff', self.callback_imu_diff, 10)
#         # Note: We still subscribe to Pitch separately because the standard IMU msg 
#         # uses Quaternions, and decoding them back to Euler here is extra work.
#         # Since the Bridge publishes both, this is fine for now.
#         self.sub_pitch    = self.create_subscription(Float32, '/imu/pitch', self.callback_pitch, 10)
        
#         # --- PUBLISHERS ---
#         self.pub_odom = self.create_publisher(Odometry, '/odometry/ekf', 50)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         self.get_logger().info("--- EKF NODE STARTED [Standardized Inputs] ---")

#     def callback_pitch(self, msg): 
#         self.current_pitch = msg.data

#     def callback_imu_diff(self, msg: Imu):
#         # 1. Time Delta Calculation
#         current_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
#         if self.last_msg_time is None:
#             self.last_msg_time = current_time_sec; return
#         dt = current_time_sec - self.last_msg_time
#         self.last_msg_time = current_time_sec
#         if dt <= 0: return

#         # 2. Unpack Data (UPDATED TO STANDARD ROS FIELDS)
#         # linear_acceleration.x -> Forward Accel (Average)
#         # angular_velocity.z    -> Yaw Rate
        
#         accel_fwd_raw = msg.linear_acceleration.x  # <--- CHANGED FROM covariance[0]
#         yaw_rate      = msg.angular_velocity.z     # <--- CHANGED FROM orientation.x
        
#         # 3. PHYSICS CORRECTION: Gravity Compensation
#         # Bridge uses atan2(-Ax), effectively flipping sign. We ADD gravity to compensate.
#         gravity_vector_x = 9.81 * sin(self.current_pitch) 
#         accel_gravity_corrected = accel_fwd_raw + gravity_vector_x 

#         # 4. PHYSICS CORRECTION: Centrifugal Force
#         # Subtracting fake forward force caused by rotation: F = w^2 * r
#         accel_centrifugal = (yaw_rate ** 2) * self.ROTATION_RADIUS
#         accel_linear_net = accel_gravity_corrected - accel_centrifugal

#         # 5. MOTION LOGIC (Zero Velocity Update & Friction)
#         accel_final = accel_linear_net
        
#         # Noise Gate: Ignore tiny accelerations
#         if abs(accel_final) < 0.15: 
#             accel_final = 0.0
            
#             # Apply Friction to decay velocity when coasting
#             current_vel = self.ekf.state[3]
#             self.ekf.state[3] = current_vel * 0.8 

#             if abs(self.ekf.state[3]) < 0.05:
#                 self.ekf.state[3] = 0.0

#         # 6. UPDATE EKF STATE
#         self.ekf.predict_state(accel_final, yaw_rate, dt)
#         self.publish_odometry(msg.header.stamp)

#         # --- DIAGNOSTIC LOGGING ---
#         self.get_logger().info(
#             f"PITCH: {degrees(self.current_pitch):.1f} deg | "
#             f"RAW_ACC: {accel_fwd_raw:.2f} | "
#             f"GRAV_COMP: {gravity_vector_x:.2f} | "
#             f"NET_ACC: {accel_final:.2f}",
#             throttle_duration_sec=0.2
#         )

#     def publish_odometry(self, stamp):
#         state = self.ekf.get_current_state()
#         pos_x, pos_y, yaw, vel, omega = state
        
#         # 1. Broadcast Transform
#         t = TransformStamped()
#         t.header.stamp = stamp
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_link'
#         t.transform.translation.x = float(pos_x)
#         t.transform.translation.y = float(pos_y)
#         t.transform.rotation.z = sin(yaw / 2.0)
#         t.transform.rotation.w = cos(yaw / 2.0)
#         self.tf_broadcaster.sendTransform(t)
        
#         # 2. Publish Odometry Message
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

# def main(args=None):
#     rclpy.init(args=args)
#     node = EKFNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__': main()



















import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
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
        self.ENABLE_MOTION_ALIGNMENT = True 
        
        # --- STATE VARIABLES ---
        self.last_msg_time = None
        self.gps_initialized = False
        self.filter_ready = False
        self.motion_aligned = False
        
        # Speed storage
        self.current_fix_velocity = 0.0

        # --- QOS PROFILES ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- SUBSCRIBERS ---
        # 1. IMU (Prediction)
        self.create_subscription(Imu, '/imu/data_raw', self.callback_imu, sensor_qos)
        
        # 2. GPS Position (Correction + Alignment)
        self.create_subscription(PoseWithCovarianceStamped, '/gps/enu_pose', self.callback_gnss, sensor_qos)
        
        # 3. Primary Speed Source (Vector Velocity acting as Wheel Odom)
        # We now trust this topic as our main speed source.
        self.create_subscription(TwistWithCovarianceStamped, '/gps/fix_velocity', self.callback_fix_velocity, sensor_qos)

        # --- PUBLISHERS ---
        self.pub_odom = self.create_publisher(Odometry, '/odometry/ekf', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("EKF STARTED. Waiting for GPS...")

    # 1. GPS HANDLER (Position Update + Motion Alignment)
    def callback_gnss(self, msg):
        x_gps = msg.pose.pose.position.x
        y_gps = msg.pose.pose.position.y

        # A. Initialization
        if not self.gps_initialized:
            self.ekf.state[0] = x_gps
            self.ekf.state[1] = y_gps
            self.gps_initialized = True
            self.filter_ready = True
            self.get_logger().info(f"GPS INIT: Position set to X:{x_gps:.1f}, Y:{y_gps:.1f}")
            return

        if not self.filter_ready: return

        # B. Motion Alignment (The "Sawtooth" Fix)
        # We check self.current_fix_velocity which is updated by the new primary callback
        if self.ENABLE_MOTION_ALIGNMENT and not self.motion_aligned and self.current_fix_velocity > 0.5:
            dx = x_gps - self.ekf.state[0]
            dy = y_gps - self.ekf.state[1]
            dist = sqrt(dx**2 + dy**2)
            
            if dist > 0.2: # Ensure we actually moved between frames
                track_yaw = atan2(dy, dx)
                self.ekf.state[2] = track_yaw # Force Yaw
                self.ekf.is_yaw_initialized = True
                self.motion_aligned = True
                self.get_logger().info(f"MOTION ALIGNMENT: Yaw snapped to {degrees(track_yaw):.1f}°")

        # C. Standard Update
        # Trust GPS Position heavily (Low variance)
        R_gps = np.diag([0.5, 0.5]) 
        self.ekf.update_gps(x_gps, y_gps, R_gps)

    # 2. PRIMARY SPEED HANDLER (Acts as Wheel Odom)
    def callback_fix_velocity(self, msg):
        """
        Calculates speed magnitude from GPS Velocity Vector and feeds it to EKF 
        as if it were wheel odometry.
        """
        if not self.filter_ready: return

        # Extract Vector Components (East/North)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        
        # Calculate Magnitude (Scalar Speed)
        speed_mps = sqrt(vx**2 + vy**2)
        self.current_fix_velocity = speed_mps

        # Feed to EKF
        # Trust this heavily (0.01 variance) to force EKF to follow GPS speed
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
        # Force acceleration to 0. We rely 100% on GPS updates for speed changes.
        accel_net = 0.0

        # Predict
        self.ekf.predict(accel_net, yaw_rate, dt)

        # Publish
        self.publish_odometry(msg.header.stamp)

    def publish_odometry(self, stamp):
        s = self.ekf.state
        
        # --- 1. TF BROADCAST ---
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
        
        # --- 2. ODOMETRY MESSAGE ---
        o = Odometry()
        o.header = t.header
        o.child_frame_id = t.child_frame_id
        
        # Pose
        o.pose.pose.position.x = float(s[0])
        o.pose.pose.position.y = float(s[1])
        o.pose.pose.position.z = 0.0
        o.pose.pose.orientation = t.transform.rotation
        
        # Pose Covariance (6x6)
        pose_cov = np.zeros(36, dtype=np.float64)
        pose_cov[0]  = self.ekf.P[0,0] # Var X
        pose_cov[7]  = self.ekf.P[1,1] # Var Y
        pose_cov[14] = 99999.0         # Var Z 
        pose_cov[21] = 99999.0         # Var Roll
        pose_cov[28] = 99999.0         # Var Pitch
        pose_cov[35] = self.ekf.P[2,2] # Var Yaw
        o.pose.covariance = pose_cov

        # Twist (Velocity)
        o.twist.twist.linear.x = float(s[3])
        o.twist.twist.linear.y = 0.0 
        o.twist.twist.linear.z = 0.0
        o.twist.twist.angular.x = 0.0
        o.twist.twist.angular.y = 0.0
        o.twist.twist.angular.z = float(s[4])
        
        # Twist Covariance (6x6)
        twist_cov = np.zeros(36, dtype=np.float64)
        twist_cov[0]  = self.ekf.P[3,3] # Var Vx
        twist_cov[7]  = 1e-6            # Var Vy 
        twist_cov[14] = 1e-6            # Var Vz
        twist_cov[21] = 1e-6            # Var Roll Rate
        twist_cov[28] = 1e-6            # Var Pitch Rate
        twist_cov[35] = self.ekf.P[4,4] # Var Yaw Rate
        o.twist.covariance = twist_cov
        
        self.pub_odom.publish(o)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(EKFNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()