"""
Extended Kalman Filter (EKF) ROS Node for State Estimation.
This module implements a ROS 2 node that performs state estimation using an Extended Kalman Filter,
fusing inertial measurement unit (IMU) data for prediction with GNSS/GPS data for correction.
Module Overview:
    The EKFNode class subscribes to IMU and GNSS topics, maintains an internal EKF instance,
    and publishes estimated odometry and transform information at regular intervals.
Key Features:
    - Sensor Fusion: Combines IMU-based dead reckoning with GNSS corrections
    - Yaw Initialization: Automatic heading calibration through forward motion detection
    - Physics Corrections: Gravity and centrifugal acceleration compensation
    - ZUPT Detection: Zero Velocity Update (standstill detection) with friction modeling
    - GPS Quality Filtering: Variance-based rejection of unreliable GNSS measurements
    - Transform Broadcasting: Real-time TF2 frame publication for visualization
Subscriptions:
    /imu/diff (sensor_msgs/Imu):
        Differential IMU measurements (linear acceleration, angular velocity)
    /imu/pitch (std_msgs/Float32):
        Pitch angle used for gravity vector decomposition
    /gps/enu_pose (geometry_msgs/PoseWithCovarianceStamped):
        GNSS position estimates in ENU (East-North-Up) frame with covariance
Publications:
    /odometry/ekf (nav_msgs/Odometry):
        Fused odometry estimates (position, velocity, orientation)
    tf (geometry_msgs/TransformStamped) via TF2:
        Dynamic transform from 'odom' to 'base_link' frame
Parameters:
    ROTATION_RADIUS (float): Wheelbase radius for centrifugal acceleration calculation (default: 0.15m)
State Variables:
    ekf (EKF): Internal EKF instance managing state prediction and update
    last_msg_time (float): Timestamp of previous IMU message for delta-time calculation
    current_pitch (float): Latest pitch angle reading from IMU
    is_yaw_initialized (bool): Flag indicating successful heading calibration
Note:
    The node enters a calibration mode on startup where it estimates initial heading by
    detecting forward motion over a minimum distance (1.5m). Normal EKF updates are
    suspended during this phase.
Example:
    >>> rclpy.init()
    >>> node = EKFNode()
    >>> rclpy.spin(node)
"""

import rclpy
from rclpy.node import Node
import numpy as np
from math import sin, cos, degrees
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
from .ekf_core import EKF

# --- IMPORT YOUR NEW EKF CLASS HERE ---
# (Paste your new EKF class code here or import it if in a separate file)
# from state_estimator.ekf_core import EKF 

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        
        self.ROTATION_RADIUS = 0.15 
        
        self.ekf = EKF()  # <--- Uses your new robust class
        self.last_msg_time = None
        self.current_pitch = 0.0

        # Subscribers
        self.create_subscription(Imu, '/imu/diff', self.callback_imu_diff, 10)
        self.create_subscription(Float32, '/imu/pitch', self.callback_pitch, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/gps/enu_pose', self.callback_gnss, 10)
        
        # Publishers
        self.pub_odom = self.create_publisher(Odometry, '/odometry/ekf', 50)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("--- EKF STARTED (Joseph Form) ---")

    def callback_pitch(self, msg): 
        self.current_pitch = msg.data

    def callback_gnss(self, msg):
        x_gps = msg.pose.pose.position.x
        y_gps = msg.pose.pose.position.y
        
        # Extract Variance
        var_x = msg.pose.covariance[0]
        var_y = msg.pose.covariance[7]

        # REJECTION GATE
        if var_x > 20.0:
            self.get_logger().warn(f"⚠️ REJECTING GPS (Var: {var_x:.2f})")
            return

        # --- 1. INITIALIZATION LOGIC ---
        if not self.ekf.is_yaw_initialized:
            if self.ekf.check_alignment(x_gps, y_gps):
                self.get_logger().info(f"✅ YAW ALIGNED! Heading: {degrees(self.ekf.state[2]):.1f} deg")
            else:
                self.get_logger().info(f"⏳ CALIBRATING... Drive Forward", throttle_duration_sec=1.0)
            return

        # --- 2. UPDATE STEP ---
        # Note: Your new class takes a 2x2 R matrix directly
        R_gps = np.array([
            [var_x, 0.0],
            [0.0, var_y]
        ])
        
        # Call the specific GPS helper in your new class
        self.ekf.update_gps(x_gps, y_gps, R_gps)
        
        status = "RTK FIXED" if var_x < 0.01 else ("RTK FLOAT" if var_x < 1.0 else "3D FIX")
        self.get_logger().info(f"🛰️ UPDATE [{status}] | Pos: {x_gps:.1f}, {y_gps:.1f}", throttle_duration_sec=2.0)

    def callback_imu_diff(self, msg: Imu):
        current_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_msg_time is None:
            self.last_msg_time = current_time_sec; return
        dt = current_time_sec - self.last_msg_time
        self.last_msg_time = current_time_sec
        if dt <= 0: return

        accel_fwd_raw = msg.linear_acceleration.x
        yaw_rate      = msg.angular_velocity.z
        
        # Physics Correction
        gravity_vector_x = 9.81 * sin(self.current_pitch) 
        accel_gravity_corrected = accel_fwd_raw + gravity_vector_x 
        accel_centrifugal = (yaw_rate ** 2) * self.ROTATION_RADIUS
        accel_final = accel_gravity_corrected - accel_centrifugal

        if abs(accel_final) < 0.15: 
            accel_final = 0.0
            # Friction decay on velocity state
            self.ekf.state[3] *= 0.8 
            if abs(self.ekf.state[3]) < 0.05: self.ekf.state[3] = 0.0

        # --- PREDICT STEP ---
        # Note: Method name changed from 'predict_state' to 'predict'
        self.ekf.predict(accel_final, yaw_rate, dt)
        
        self.publish_odometry(msg.header.stamp)

    def publish_odometry(self, stamp):
        state = self.ekf.get_current_state()
        pos_x, pos_y, yaw, vel, omega = state
        
        # TF Broadcasting
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(pos_x)
        t.transform.translation.y = float(pos_y)
        t.transform.rotation.z = sin(yaw / 2.0)
        t.transform.rotation.w = cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)
        
        # Odometry Publishing
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(pos_x)
        odom.pose.pose.position.y = float(pos_y)
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = float(vel)
        odom.twist.twist.angular.z = float(omega)
        self.pub_odom.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()
















# import rclpy
# from rclpy.node import Node
# import numpy as np
# from math import sin, cos, degrees
# from sensor_msgs.msg import Imu
# from std_msgs.msg import Float32
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
# from tf2_ros import TransformBroadcaster

# # --- MATH HELPER CLASS ---
# class EKF:
#     def __init__(self):
#         # State Vector: [pos_x, pos_y, yaw, velocity, yaw_rate]
#         self.state = np.zeros(5)
        
#         # --- STEP 3: COVARIANCE INITIALIZATION ---
#         # P: Initial Uncertainty (Trust mechanism)
#         self.P = np.diag([
#             1.0,   # x
#             1.0,   # y
#             0.1,   # yaw
#             1.0,   # velocity
#             0.1    # yaw_rate
#         ])

#         # Q: Process Noise (Model Uncertainty per second)
#         # We assume Velocity is noisy (0.5) so EKF looks to GPS to fix it.
#         self.Q = np.diag([
#             0.05,  # x
#             0.05,  # y
#             0.01,  # yaw
#             0.5,   # velocity
#             0.05   # yaw_rate
#         ])

#     def predict_state(self, accel_fwd, yaw_rate, dt):
#         x, y, yaw, v, omega = self.state
        
#         # --- 1. STATE PREDICTION (Physics) ---
#         new_yaw = yaw + yaw_rate * dt
#         new_v = v + accel_fwd * dt
#         new_v *= 0.99 # Drag factor
        
#         # Safety Clamping
#         new_v = max(min(new_v, 5.0), -5.0)

#         new_x = x + new_v * np.cos(yaw) * dt
#         new_y = y + new_v * np.sin(yaw) * dt
        
#         self.state = np.array([new_x, new_y, new_yaw, new_v, yaw_rate])

#         # --- 2. COVARIANCE PREDICTION (Uncertainty Growth) ---
#         # Jacobian F (Partial derivatives of state w.r.t state)
#         F = np.eye(5)
        
#         # Derivatives for Position vs Yaw/Velocity
#         F[0, 2] = -v * np.sin(yaw) * dt  # dx/dyaw
#         F[0, 3] = np.cos(yaw) * dt       # dx/dv
#         F[1, 2] = v * np.cos(yaw) * dt   # dy/dyaw
#         F[1, 3] = np.sin(yaw) * dt       # dy/dv
#         F[2, 4] = dt                     # dyaw/domega

#         # P_new = F * P * F_transpose + (Q * dt)
#         # We scale Q by dt because noise accumulates with time.
#         self.P = F @ self.P @ F.T + (self.Q * dt)

#     def update_gps(self, z, R):
#         # Observation Matrix H: We measure x (idx 0) and y (idx 1)
#         H = np.array([
#             [1, 0, 0, 0, 0],
#             [0, 1, 0, 0, 0]
#         ])

#         # Innovation (Error between GPS and Prediction)
#         y = z - H @ self.state
        
#         # Innovation Covariance
#         S = H @ self.P @ H.T + R
        
#         # Kalman Gain
#         try:
#             K = self.P @ H.T @ np.linalg.inv(S)
#         except np.linalg.LinAlgError:
#             # Fallback if matrix singular (rare)
#             return

#         # Update State
#         self.state = self.state + K @ y
        
#         # Update Uncertainty
#         self.P = (np.eye(5) - K @ H) @ self.P
    
#     def get_current_state(self): return self.state

# # --- ROS NODE ---
# class EKFNode(Node):
#     def __init__(self):
#         super().__init__('ekf_node_diff_imu')
        
#         self.ROTATION_RADIUS = 0.15 
        
#         self.ekf = EKF()
#         self.last_msg_time = None
#         self.current_pitch = 0.0

#         # --- SUBSCRIBERS ---
#         # 1. IMU (Prediction)
#         self.create_subscription(Imu, '/imu/diff', self.callback_imu_diff, 10)
#         self.create_subscription(Float32, '/imu/pitch', self.callback_pitch, 10)
        
#         # 2. GNSS (Correction) - STEP 4 Implemented Here
#         self.create_subscription(PoseWithCovarianceStamped, '/gps/enu_pose', self.callback_gnss, 10)
        
#         # --- PUBLISHERS ---
#         self.pub_odom = self.create_publisher(Odometry, '/odometry/ekf', 50)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         self.get_logger().info("--- EKF ACTIVE: IMU PREDICT + GNSS UPDATE ---")

#     def callback_pitch(self, msg): 
#         self.current_pitch = msg.data

#     # --- STEP 4: GNSS CALLBACK ---
#     def callback_gnss(self, msg):
#         # 1. Extract Measurement Vector z = [x, y]
#         x_gps = msg.pose.pose.position.x
#         y_gps = msg.pose.pose.position.y
#         z = np.array([x_gps, y_gps])
        
#         # 2. Extract Covariance R (2x2)
#         # msg.pose.covariance is a 36-element array.
#         # Index 0 is Var(X), Index 7 is Var(Y).
#         cov = msg.pose.covariance
#         R = np.array([
#             [cov[0], 0.0],
#             [0.0, cov[7]]
#         ])
        
#         # 3. Call EKF Update
#         self.ekf.update_gps(z, R)
        
#         # Debug Log (Low throttle)
#         self.get_logger().info(f"🛰️ GPS CORRECTION | Pos: {x_gps:.2f}, {y_gps:.2f}", throttle_duration_sec=2.0)

#     # --- IMU CALLBACK ---
#     def callback_imu_diff(self, msg: Imu):
#         current_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
#         if self.last_msg_time is None:
#             self.last_msg_time = current_time_sec; return
#         dt = current_time_sec - self.last_msg_time
#         self.last_msg_time = current_time_sec
#         if dt <= 0: return

#         # Unpack Data
#         accel_fwd_raw = msg.linear_acceleration.x
#         yaw_rate      = msg.angular_velocity.z
        
#         # Physics Correction
#         gravity_vector_x = 9.81 * sin(self.current_pitch) 
#         accel_gravity_corrected = accel_fwd_raw + gravity_vector_x 
#         accel_centrifugal = (yaw_rate ** 2) * self.ROTATION_RADIUS
#         accel_linear_net = accel_gravity_corrected - accel_centrifugal

#         # Motion Logic (ZUPT)
#         accel_final = accel_linear_net
#         if abs(accel_final) < 0.15: 
#             accel_final = 0.0
#             self.ekf.state[3] *= 0.8 # Friction
#             if abs(self.ekf.state[3]) < 0.05: self.ekf.state[3] = 0.0

#         # EKF Prediction
#         self.ekf.predict_state(accel_final, yaw_rate, dt)
#         self.publish_odometry(msg.header.stamp)

#     def publish_odometry(self, stamp):
#         state = self.ekf.get_current_state()
#         pos_x, pos_y, yaw, vel, omega = state
        
#         t = TransformStamped()
#         t.header.stamp = stamp
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_link'
#         t.transform.translation.x = float(pos_x)
#         t.transform.translation.y = float(pos_y)
#         t.transform.rotation.z = sin(yaw / 2.0)
#         t.transform.rotation.w = cos(yaw / 2.0)
#         self.tf_broadcaster.sendTransform(t)
        
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
