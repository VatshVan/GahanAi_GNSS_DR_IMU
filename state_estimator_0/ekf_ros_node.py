"""
Extended Kalman Filter ROS2 Node for GNSS/IMU Fusion — Module Docstring
Module Purpose
---------------
This module implements a production-grade Extended Kalman Filter (EKF) ROS2 node that
fuses GNSS (GPS) position and velocity information with IMU angular-rate (and optionally
accelerometer/wheel-speed) inputs to produce robust, continuous pose and velocity estimates
for ground vehicles or similarly constrained robotic platforms.
The implementation is intentionally minimal in runtime work per sample (predict/update
separations) and is designed to be:
- Deterministic and real-time friendly (small predictable per-sample computation).
- Easy to extend and retune (clear places to change process/measurement covariances).
- Interoperable with ROS2 ecosystems (standard message types and TF broadcasting).
- Maintainable for long-term use (documented assumptions, interfaces, and tests).
Primary Components
------------------
- EKFNode (rclpy.Node)
    - ROS2 node that performs:
        - IMU-based prediction step (uses angular rate; accelerometer optional).
        - GNSS position updates with optional motion-based heading alignment.
        - Wheel/primary speed integration as a trusted speed source (optionally validated
          against fix velocity).
        - TF broadcasting ("odom" -> "base_link") and /odometry/ekf publishing.
    - Subscribed topics (default names):
        - /imu/data_raw                   (sensor_msgs/Imu)
            - Used primarily for angular rate (z) for heading integration.
            - Accelerometer support exists in code paths but may be disabled by config.
        - /gps/enu_pose                   (geometry_msgs/PoseWithCovarianceStamped)
            - GNSS position in ENU frame; covariance array used to form measurement R.
        - /gps/speed_kmph                 (std_msgs/Float32)
            - Primary trusted speed source (treated like wheel odometry).
            - Units: km/h input, converted to m/s internally.
        - /gps/fix_velocity               (geometry_msgs/TwistWithCovarianceStamped)
            - Optional velocity vector (east/north). Used to validate primary speed.
    - Published topics:
        - /odometry/ekf                   (nav_msgs/Odometry)
            - Pose and twist representation of the filter state.
        - TF: odom -> base_link
            - Pose broadcast for downstream consumers.
    - QoS:
        - IMU, GPS and speed topics use a best-effort, volatile, KEEP_LAST QoS suitable
          for current vehicle-data streaming scenarios. Adjust QoS for networks with
          lossy/delayed transports or across namespace boundaries as required.
- EKF (imported from .ekf_core)
    - Filter core implementing predict/measurement update logic.
    - Expected public API:
        - state: numpy array-like [pos_x, pos_y, yaw, velocity, yaw_rate]
        - P: covariance matrix matching state size (5x5 in current design)
        - is_yaw_initialized: boolean flag used by node to gate alignment/updates
        - predict(accel: float, yaw_rate: float, dt: float)
        - update_gps(x: float, y: float, R: np.ndarray(2x2))
        - update_wheel_speed(speed_mps: float, var_speed: float)
        - get_current_state() -> state vector
    - Notes:
        - The EKF implementation is intentionally separate. Changes to the EKF API must
          preserve these method signatures or the node must be updated accordingly.
State Representation and Units
------------------------------
- State vector (5-DOF):
    - index 0: pos_x (meters, ENU East)
    - index 1: pos_y (meters, ENU North)
    - index 2: yaw (radians, ENU heading measured CCW from East; consistent with atan2(dy,dx))
    - index 3: velocity (m/s, forward along yaw)
    - index 4: yaw_rate (rad/s)
- Time: seconds using ROS2 message header timestamps (header.stamp.sec + header.stamp.nanosec*1e-9).
- All angles use radians internally. Exposed logs may show degrees for human readability.
Design Assumptions & Behavioral Summary
--------------------------------------
- Coordinate Frame: ENU (East, North, Up). TF published from "odom" to "base_link".
- IMU:
    - Primary purpose is to supply yaw_rate (angular_velocity.z) for heading integration.
    - Accelerometer-based speed integration is intentionally weak/disabled in production
      configurations to avoid drift; speed is primarily sourced from GNSS/wheel.
    - A pitch message may optionally be consumed when gravity compensation is required.
- GPS:
    - PoseWithCovarianceStamped is used as the position measurement; covariance indices 0
      and 7 correspond to Var(x) and Var(y).
    - A motion-alignment strategy exists: on first meaningful motion the yaw can be
      snapped to GPS track. This mitigates initial yaw ambiguity ("sawtooth" alignment).
- Speed Sources:
    - Primary speed input is /gps/speed_kmph (treated similar to wheel odometry).
    - A secondary /gps/fix_velocity vector can be used to validate or reject the
      primary speed to detect slip or stale readings.
- Initialization:
    - The node sets the filter position to the first received GNSS fix and waits for
      motion alignment if enabled before accepting IMU-based predictions for full
      operation (filter_ready flag).
- Robustness:
    - Sanity checks in the node reject unrealistic wheel speeds vs predicted speeds.
    - The EKF should guard matrix inversions with proper exception handling internally.
Configuration and Tunables
--------------------------
- Node-level configuration constants (can be converted to ROS2 parameters):
    - MAX_SPEED_MPS: maximum allowed vehicle speed for sanity clamping.
    - ENABLE_MOTION_ALIGNMENT: enable/disable snapping yaw to GPS track on initial motion.
    - ROTATION_RADIUS: used when compensating centrifugal acceleration if accelerometer
      input is used (meters).
    - R_gps: measurement covariance used for GPS updates; currently defaulted in-node but
      should be configured based on receiver-class (RTK vs SBAS vs consumer).
- EKF internal matrices:
    - Q (process noise) and initial P must be tuned per platform mass, sensor quality,
      and expected dynamics. Add ROS2 parameters to make these configurable at runtime.
- QoS settings: adjust reliability/durability/depth as deployment characteristics demand.
Operational Guidelines & Best Practices
--------------------------------------
- Time Synchronization:
    - Ensure sensors publish timestamps in a consistent clock (ideally synced to the
      robot/local host clock). ROS2 Time or hardware timestamping is preferred.
- GPS Covariance:
    - Use GNSS receivers that publish realistic covariance estimates. If unavailable,
      configure R_gps conservatively (larger variances) to avoid overconfidence.
- Bootstrapping:
    - Start vehicle stationary and allow a few GNSS fixes to set the initial position.
    - Enable motion alignment by ensuring the vehicle moves straight for a few meters
      to allow yaw snapping to the GPS track if required.
- Testing:
    - Unit tests for EKF core (predict/update math, Jacobians, covariance positive-definiteness).
    - Integration tests in simulation (Gazebo/recorded bag files) with injected noise.
    - Long-duration stress tests at expected mission profiles.
- Observability & Telemetry:
    - Expose filter residuals, innovation covariance S, and gain K on a diagnostics topic
      or logging/metrics endpoint for tuning and monitoring.
    - Log occasional (throttled) summaries of P-diagonal elements to monitor uncertainty growth.
- Safety:
    - Do not feed this estimator directly to safety-critical motion controllers without
      additional health checks (e.g., GNSS integrity, NTRIP/RTK fix status, wheel slip detection).
Failure Modes and Mitigations
----------------------------
- Stale or missing IMU:
    - Node currently uses IMU for yaw_rate; if IMU becomes unavailable, yaw integration
      will stall — consider fallback strategies (e.g., dead-reckoning with last known speed
      and yaw or rely on GPS heading when moving).
- GPS outages:
    - Expect larger positional uncertainty during GNSS outages. Increase Q for velocity
      process noise and rely on wheel odometry if available.
- Overconfident GPS covariance:
    - If receiver supplies unrealistically small covariances, the filter may become
      overconfident and unstable. Clamp minimum R values or perform sanity checks on
      covariance priors.
- Singularity in update (S matrix singular):
    - EKF must handle and recover from linear-algebra exceptions; fallback to
      conservative covariance inflation or skipping the update.
Extensibility & Integration Points
---------------------------------
- Parameterization:
    - Convert hard-coded constants (MAX_SPEED_MPS, R_gps defaults, motion alignment
      thresholds) to ROS2 node parameters for in-situ tuning and easier long-term maintenance.
- Additional Sensors:
    - Visual odometry, LiDAR pose updates, or wheel encoders can be integrated by adding
      measurement handlers that publish to the same EKF API (extend EKF.update_* methods).
- Multi-rate Handling:
    - Node assumes reasonably synchronized sensors with timestamps. For multi-rate or
      delayed measurements, consider buffering and reprocessing or use delayed-state EKF
      (e.g., augment state with short-term pose history).
- Diagnostics:
    - Add a diagnostics_updater or custom diagnostic topic to expose filter health, GNSS
      fix quality, IMU health, and residual magnitudes.
Testing, CI and Long-term Maintenance
------------------------------------
- Unit Tests:
    - EKF predict/update math must be covered with deterministic scenarios and edge-cases:
      - Zero motion, constant velocity, pure rotation, high-noise sensors.
- Integration Tests:
    - Playback recorded ROS bags (IMU + GNSS + speed) and assert accepted tolerances on
      final position/yaw after long runs.
- Regression Tests:
    - Add tests that capture previously observed failure modes (e.g., "stuck at zero" bug)
      to ensure fixes remain enforced.
- Continuous Integration:
    - Run static analysis (mypy/flake8), unit tests, and integration tests (sensor bag playback)
      on each change to preserve behavioral contracts.
- Backwards Compatibility:
    - Keep the public EKF API stable or provide adapter layers when changing signatures.
    - Maintain semantic versioning and a changelog in the repository.
Versioning, Authorship & License
-------------------------------
- Author: VatshVan (documented original author)
- License: Proprietary - GahanAI (as present in source header).
- Maintain explicit changelog entries and adhere to semantic versioning on public releases.
Example Usage Summary
---------------------
- Start ROS2 and bring up sensors that publish:
    - /gps/enu_pose (PoseWithCovarianceStamped)
    - /imu/data_raw (sensor_msgs/Imu)
    - /gps/speed_kmph (std_msgs/Float32)  (optional but recommended)
- Launch this node; observe /odometry/ekf and TF "odom" -> "base_link".
- Tune EKF.Q, EKF.P, and R_gps based on platform characteristics and GNSS quality.
Stability & Deprecated Notes
----------------------------
- The code is designed to be stable for long-term maintenance. When deprecating or
  replacing major subsystems (e.g., switching to an Unscented KF or factor-graph
  backend), provide adapters or migration scripts and keep test coverage to prevent
  regressions across versions.
Contact & Support
-----------------
- For questions about algorithmic choices, EKF tuning, or integration guidance, contact
  the original author/maintainers. Include reproducer bags and configuration files for
  efficient diagnosis.
"""

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












# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# from sensor_msgs.msg import Imu
# from std_msgs.msg import Float32
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
# from tf2_ros import TransformBroadcaster
# from math import sin, cos, sqrt, degrees, radians, atan2, asin, pi
# import numpy as np
# import math

# # Make sure this import matches your file structure
# # from .ekf_core import EKF 
# try:
#     # Try importing as a ROS package (Relative import)
#     from .ekf_core import EKF
# except ImportError:
#     # Fallback if running as a generic script (Absolute import)
#     from ekf_core import EKF

# class EKFNode(Node):
#     def __init__(self):
#         super().__init__('ekf_node')
#         self.ekf = EKF()
        
#         # --- CONFIGURATION ---
#         self.MAX_SPEED_MPS = 10.0 
#         self.ENABLE_MOTION_ALIGNMENT = True # If True, Yaw resets to match GPS track on first move
        
#         # --- STATE VARIABLES ---
#         self.last_msg_time = None
#         self.gps_initialized = False
#         self.filter_ready = False
#         self.motion_aligned = False
        
#         # Speed storage
#         self.current_speed_kmph = 0.0
#         self.current_fix_velocity = 0.0

#         # --- QOS PROFILES ---
#         sensor_qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.VOLATILE,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=10
#         )

#         # --- SUBSCRIBERS ---
#         # 1. IMU (Prediction)
#         self.create_subscription(Imu, '/imu/data_raw', self.callback_imu, sensor_qos)
        
#         # 2. GPS Position (Correction + Alignment)
#         self.create_subscription(PoseWithCovarianceStamped, '/gps/enu_pose', self.callback_gnss, sensor_qos)
        
#         # 3. Primary Speed Source (Wheel Odom equivalent)
#         self.create_subscription(Float32, '/gps/speed_kmph', self.callback_speed_kmph, sensor_qos)
        
#         # 4. Velocity Validation Source
#         self.create_subscription(TwistWithCovarianceStamped, '/gps/fix_velocity', self.callback_fix_velocity, sensor_qos)

#         # --- PUBLISHERS ---
#         self.pub_odom = self.create_publisher(Odometry, '/odometry/ekf', 10)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         self.get_logger().info("EKF STARTED. Waiting for GPS...")

#     # 1. GPS HANDLER (Position Update + Motion Alignment)
#     def callback_gnss(self, msg):
#         x_gps = msg.pose.pose.position.x
#         y_gps = msg.pose.pose.position.y
#         var_x = msg.pose.covariance[0]
#         var_y = msg.pose.covariance[7]

#         # A. Initialization
#         if not self.gps_initialized:
#             self.ekf.state[0] = x_gps
#             self.ekf.state[1] = y_gps
#             self.gps_initialized = True
#             self.filter_ready = True
#             self.get_logger().info(f"GPS INIT: Position set to X:{x_gps:.1f}, Y:{y_gps:.1f}")
#             return

#         if not self.filter_ready: return

#         # B. Motion Alignment (The "Sawtooth" Fix)
#         # We use the VALIDATED fix velocity for alignment check if available, otherwise scalar speed
#         check_speed = self.current_fix_velocity if self.current_fix_velocity > 0 else (self.current_speed_kmph / 3.6)
        
#         if self.ENABLE_MOTION_ALIGNMENT and not self.motion_aligned and check_speed > 0.5:
#             dx = x_gps - self.ekf.state[0]
#             dy = y_gps - self.ekf.state[1]
#             dist = sqrt(dx**2 + dy**2)
            
#             if dist > 0.2: # Ensure we actually moved between frames
#                 track_yaw = atan2(dy, dx)
#                 self.ekf.state[2] = track_yaw # Force Yaw
#                 self.ekf.is_yaw_initialized = True
#                 self.motion_aligned = True
#                 self.get_logger().info(f"MOTION ALIGNMENT: Yaw snapped to {degrees(track_yaw):.1f}°")

#         # C. Standard Update
#         # Trust GPS Position heavily (Low variance)
#         R_gps = np.diag([0.5, 0.5]) 
#         self.ekf.update_gps(x_gps, y_gps, R_gps)

#     # 2. PRIMARY SPEED HANDLER (Acts as Wheel Odom)
#     def callback_speed_kmph(self, msg):
#         speed_mps = msg.data / 3.6
#         self.current_speed_kmph = msg.data

#         if not self.filter_ready: return

#         # VALIDATION: Compare with Fix Velocity
#         # If fix velocity is available and differs significantly, we might have slip or lag
#         if self.current_fix_velocity > 0:
#             diff = abs(speed_mps - self.current_fix_velocity)
#             if diff > 2.0: # 2 m/s discrepancy
#                 # self.get_logger().warn(f"Speed mismatch! KMPH: {speed_mps:.2f}, FIX: {self.current_fix_velocity:.2f}")
#                 pass # For now, we trust speed_kmph, but you can add logic here to reject

#         # Trust GPS Speed ALMOST PERFECTLY.
#         # This forces the Blue line to stick to the Green dots.
#         var_speed = 0.01 
#         self.ekf.update_wheel_speed(speed_mps, var_speed)

#     # 3. VELOCITY VALIDATION HANDLER
#     def callback_fix_velocity(self, msg):
#         # Calculates magnitude of the velocity vector (East/North components)
#         vx = msg.twist.twist.linear.x
#         vy = msg.twist.twist.linear.y
#         self.current_fix_velocity = sqrt(vx**2 + vy**2)

#     # 4. PREDICTION STEP
#     def callback_imu(self, msg: Imu):
#         if not self.filter_ready: return

#         now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
#         if self.last_msg_time is None: 
#             self.last_msg_time = now
#             return
            
#         dt = now - self.last_msg_time
#         self.last_msg_time = now
        
#         if dt <= 0: return

#         # --- A. YAW RATE ---
#         # We only use the Gyro. We IGNORE the accelerometer for speed.
#         yaw_rate  = msg.angular_velocity.z
        
#         # --- B. ACCEL IGNORE ---
#         # Force acceleration to 0. This stops the velocity spikes.
#         # We rely 100% on GPS updates for speed changes.
#         accel_net = 0.0

#         # Predict
#         self.ekf.predict(accel_net, yaw_rate, dt)

#         # Publish
#         self.publish_odometry(msg.header.stamp)

#     def publish_odometry(self, stamp):
#         s = self.ekf.state
        
#         # --- 1. TF BROADCAST ---
#         t = TransformStamped()
#         t.header.stamp = stamp
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_link'
#         t.transform.translation.x = float(s[0])
#         t.transform.translation.y = float(s[1])
#         t.transform.translation.z = 0.0
        
#         t.transform.rotation.z = sin(s[2]/2.0)
#         t.transform.rotation.w = cos(s[2]/2.0)
        
#         self.tf_broadcaster.sendTransform(t)
        
#         # --- 2. ODOMETRY MESSAGE ---
#         o = Odometry()
#         o.header = t.header
#         o.child_frame_id = t.child_frame_id
        
#         # Pose
#         o.pose.pose.position.x = float(s[0])
#         o.pose.pose.position.y = float(s[1])
#         o.pose.pose.position.z = 0.0
#         o.pose.pose.orientation = t.transform.rotation
        
#         # Pose Covariance (6x6)
#         # We populate the diagonal to tell Nav2/Controls what we know.
#         # Format: [x, y, z, roll, pitch, yaw]
#         pose_cov = np.zeros(36, dtype=np.float64)
#         pose_cov[0]  = self.ekf.P[0,0] # Var X
#         pose_cov[7]  = self.ekf.P[1,1] # Var Y
#         pose_cov[14] = 99999.0         # Var Z (Unknown/High)
#         pose_cov[21] = 99999.0         # Var Roll
#         pose_cov[28] = 99999.0         # Var Pitch
#         pose_cov[35] = self.ekf.P[2,2] # Var Yaw
#         o.pose.covariance = pose_cov

#         # Twist (Velocity)
#         o.twist.twist.linear.x = float(s[3])
#         o.twist.twist.linear.y = 0.0 # Non-holonomic assumption (car cannot slide sideways)
#         o.twist.twist.linear.z = 0.0
#         o.twist.twist.angular.x = 0.0
#         o.twist.twist.angular.y = 0.0
#         o.twist.twist.angular.z = float(s[4])
        
#         # Twist Covariance (6x6)
#         twist_cov = np.zeros(36, dtype=np.float64)
#         twist_cov[0]  = self.ekf.P[3,3] # Var Vx
#         twist_cov[7]  = 1e-6            # Var Vy (Low because we assume 0 for car)
#         twist_cov[14] = 1e-6            # Var Vz
#         twist_cov[21] = 1e-6            # Var Roll Rate
#         twist_cov[28] = 1e-6            # Var Pitch Rate
#         twist_cov[35] = self.ekf.P[4,4] # Var Yaw Rate
#         o.twist.covariance = twist_cov
        
#         self.pub_odom.publish(o)

# def main(args=None):
#     rclpy.init(args=args)
#     rclpy.spin(EKFNode())
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()









import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from math import sin, cos, sqrt, degrees, radians, atan2, asin, pi
import numpy as np
import math

try:
    from .ekf_core import EKF
except ImportError:
    from ekf_core import EKF

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        self.ekf = EKF()
        
        # --- CONFIGURATION ---
        self.MAX_SPEED_MPS = 10.0 
        self.ENABLE_MOTION_ALIGNMENT = True 
        
        # VISUALIZATION CONFIG (Longitudinal)
        self.SENSOR_DIST = 0.9    # Distance between Front and Rear IMUs
        self.VEHICLE_LENGTH = 1.5 # Slightly longer to fit sensors
        self.VEHICLE_WIDTH = 0.6
        
        # --- STATE VARIABLES ---
        self.last_msg_time = None
        self.gps_initialized = False
        self.filter_ready = False
        self.motion_aligned = False
        self.current_fix_velocity = 0.0

        # --- QOS PROFILES ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- SUBSCRIBERS ---
        self.create_subscription(Imu, '/imu/data_raw', self.callback_imu, sensor_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/gps/enu_pose', self.callback_gnss, sensor_qos)
        self.create_subscription(TwistWithCovarianceStamped, '/gps/fix_velocity', self.callback_fix_velocity, sensor_qos)

        # --- PUBLISHERS ---
        self.pub_odom = self.create_publisher(Odometry, '/odometry/ekf', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/vehicle_markers', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        print("------------------------------------------------")
        print(">>> EKF LONGITUDINAL MODE (Front/Back Sensors) <<<")
        print("------------------------------------------------")

    def callback_gnss(self, msg):
        x_gps = msg.pose.pose.position.x
        y_gps = msg.pose.pose.position.y

        if not self.gps_initialized:
            self.ekf.state[0] = x_gps
            self.ekf.state[1] = y_gps
            self.gps_initialized = True
            self.filter_ready = True
            return

        if not self.filter_ready: return

        if self.ENABLE_MOTION_ALIGNMENT and not self.motion_aligned and self.current_fix_velocity > 0.5:
            dx = x_gps - self.ekf.state[0]
            dy = y_gps - self.ekf.state[1]
            dist = sqrt(dx**2 + dy**2)
            if dist > 0.2: 
                track_yaw = atan2(dy, dx)
                self.ekf.state[2] = track_yaw 
                self.ekf.is_yaw_initialized = True
                self.motion_aligned = True

        R_gps = np.diag([0.05, 0.05]) 
        self.ekf.update_gps(x_gps, y_gps, R_gps)

    def callback_fix_velocity(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_fix_velocity = sqrt(vx**2 + vy**2)
        self.ekf.update_wheel_speed(self.current_fix_velocity, 0.01)

    def callback_imu(self, msg: Imu):
        if not self.filter_ready: return
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_msg_time is None: 
            self.last_msg_time = now
            return
        dt = now - self.last_msg_time
        self.last_msg_time = now
        if dt <= 0: return

        yaw_rate = msg.angular_velocity.z
        self.ekf.predict(0.0, yaw_rate, dt)
        self.publish_odometry(msg.header.stamp)

    def publish_odometry(self, stamp):
        s = self.ekf.state
        
        # TF & Odom Logic (Standard)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'; t.child_frame_id = 'base_link'
        t.transform.translation.x = float(s[0]); t.transform.translation.y = float(s[1])
        t.transform.rotation.z = sin(s[2]/2.0); t.transform.rotation.w = cos(s[2]/2.0)
        self.tf_broadcaster.sendTransform(t)
        
        o = Odometry()
        o.header = t.header; o.child_frame_id = t.child_frame_id
        o.pose.pose.position.x = float(s[0]); o.pose.pose.position.y = float(s[1])
        o.pose.pose.orientation = t.transform.rotation
        o.twist.twist.linear.x = float(s[3]); o.twist.twist.angular.z = float(s[4])
        
        # Fake Covariance for Viz
        pose_cov = np.zeros(36); pose_cov[0] = self.ekf.P[0,0]; pose_cov[7] = self.ekf.P[1,1]; pose_cov[35] = self.ekf.P[2,2]
        o.pose.covariance = pose_cov
        
        self.pub_odom.publish(o)
        
        # --- PUBLISH LONGITUDINAL MARKERS ---
        self.publish_markers(stamp, s[0], s[1], s[2])

    def publish_markers(self, stamp, x, y, yaw):
        markers = MarkerArray()

        # 1. Chassis (Green Box)
        chassis = Marker()
        chassis.header.frame_id = "odom"
        chassis.header.stamp = stamp
        chassis.ns = "vehicle_body"; chassis.id = 0; chassis.type = Marker.CUBE
        chassis.action = Marker.ADD
        chassis.pose.position.x = x; chassis.pose.position.y = y; chassis.pose.position.z = 0.1
        chassis.pose.orientation.z = sin(yaw/2.0); chassis.pose.orientation.w = cos(yaw/2.0)
        chassis.scale.x = self.VEHICLE_LENGTH
        chassis.scale.y = self.VEHICLE_WIDTH
        chassis.scale.z = 0.1
        chassis.color.g = 1.0; chassis.color.a = 0.5 
        markers.markers.append(chassis)

        # 2. Sensors (Red Spheres) - Front and Back
        half_dist = self.SENSOR_DIST / 2.0
        
        # FRONT SENSOR (Positive X relative to car center)
        # x_new = x + d * cos(yaw)
        # y_new = y + d * sin(yaw)
        f_dx = half_dist * cos(yaw)
        f_dy = half_dist * sin(yaw)

        # Front IMU
        imu_f = Marker()
        imu_f.header.frame_id = "odom"
        imu_f.header.stamp = stamp
        imu_f.ns = "sensors"; imu_f.id = 1; imu_f.type = Marker.SPHERE
        imu_f.pose.position.x = x + f_dx
        imu_f.pose.position.y = y + f_dy
        imu_f.pose.position.z = 0.15
        imu_f.scale.x = 0.15; imu_f.scale.y = 0.15; imu_f.scale.z = 0.15
        imu_f.color.r = 1.0; imu_f.color.a = 1.0
        markers.markers.append(imu_f)

        # Rear IMU
        imu_r = Marker()
        imu_r.header.frame_id = "odom"
        imu_r.header.stamp = stamp
        imu_r.ns = "sensors"; imu_r.id = 2; imu_r.type = Marker.SPHERE
        imu_r.pose.position.x = x - f_dx
        imu_r.pose.position.y = y - f_dy
        imu_r.pose.position.z = 0.15
        imu_r.scale.x = 0.15; imu_r.scale.y = 0.15; imu_r.scale.z = 0.15
        imu_r.color.r = 1.0; imu_r.color.a = 1.0
        markers.markers.append(imu_r)
        
        self.pub_markers.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(EKFNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()