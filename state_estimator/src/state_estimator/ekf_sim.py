# """
# ekf_sim.py

# Simulation and plotting for the updated EKF (5-state vector).
# Compatible with the new ekf_core.py structure.

# Simulates:
#  - IMU Yaw Rate + Forward Acceleration (Inputs for Prediction)
#  - Wheel Velocity (Update step via generic correct)
#  - GPS Position (Update step via update_gps)
# """

# import numpy as np
# import matplotlib.pyplot as plt
# import sys

# np.random.seed(42)
# # Try importing the EKF class
# try:
#     from ekf_core import EKF
# except ImportError:
#     print("Error: Could not import 'EKF' from 'ekf_core.py'.")
#     print("Make sure ekf_core.py is in the same directory.")
#     sys.exit(1)

# def wrap_angle(a):
#     """Normalize angle to [-pi, pi]"""
#     return (a + np.pi) % (2*np.pi) - np.pi

# def angle_diff(a, b):
#     """Calculate smallest difference between two angles"""
#     d = a - b
#     return wrap_angle(d)

# def simulate():
#     # ---------------------------------------------------------
#     # 1. Setup Simulation Parameters
#     # ---------------------------------------------------------
#     np.random.seed(42)
#     sim_time = 80.0
#     dt = 0.02  # 50 Hz
#     t_steps = int(sim_time / dt)
#     t = np.arange(t_steps) * dt

#     # Sensor Rates
#     gps_dt = 0.5  # 2 Hz
#     gps_step_interval = int(gps_dt / dt)

#     # ---------------------------------------------------------
#     # 2. Generate True Trajectory (Ground Truth)
#     # ---------------------------------------------------------
#     # We create a varying yaw rate and acceleration profile
#     true_omega = 0.08 * np.sin(0.1 * t) + 0.04 * np.sin(0.3 * t)
#     true_acc   = 0.05 * np.cos(0.05 * t)

#     # State: [x, y, yaw, v] (Note: Simulation uses 4 states for truth, EKF uses 5)
#     x_true = np.zeros((t_steps, 4))
#     x_true[0] = [0.0, 0.0, 0.0, 2.0]  # Start at (0,0), 2 m/s

#     for k in range(1, t_steps):
#         px, py, yaw, v = x_true[k-1]
        
#         # Physics Integration
#         yaw_new = wrap_angle(yaw + true_omega[k-1] * dt)
#         v_new   = max(0.0, v + true_acc[k-1] * dt)
#         px_new  = px + v_new * np.cos(yaw_new) * dt
#         py_new  = py + v_new * np.sin(yaw_new) * dt
        
#         x_true[k] = [px_new, py_new, yaw_new, v_new]

#     # ---------------------------------------------------------
#     # 3. Generate Noisy Measurements
#     # ---------------------------------------------------------
#     # Noise Standard Deviations
#     std_imu_omega = np.deg2rad(1.0)  # Gyro noise
#     std_imu_accel = 0.1              # Accelerometer noise (New!)
#     std_gps_xy    = 1.5              # GPS Position noise
#     std_wheel_v   = 0.2              # Wheel Encoder noise

#     # Add Gaussian noise
#     meas_imu_omega = true_omega + np.random.randn(t_steps) * std_imu_omega
#     meas_imu_accel = true_acc   + np.random.randn(t_steps) * std_imu_accel
#     meas_gps_xy    = x_true[:, 0:2] + np.random.randn(t_steps, 2) * std_gps_xy
#     meas_wheel_v   = x_true[:, 3] + np.random.randn(t_steps) * std_wheel_v

#     # ---------------------------------------------------------
#     # 4. Initialize EKF
#     # ---------------------------------------------------------
#     ekf = EKF()
    
#     # Initial Guess (Slightly wrong to test convergence)
#     # State Vector: [x, y, yaw, v, yaw_rate]
#     ekf.state = np.array([-2.0, 1.0, np.deg2rad(5.0), 1.0, 0.0])
    
#     # Tune Covariance Matrices (5x5)
#     # P: Initial Uncertainty
#     ekf.P = np.diag([5.0, 5.0, 0.5, 2.0, 0.1])

#     # Q: Process Noise (Trust in prediction)
#     ekf.Q += np.diag([
#         0.05,  # X
#         0.05,  # Y
#         0.01,  # Yaw
#         0.5,   # Velocity (High because simple friction model)
#         0.05   # Yaw Rate
#     ])
    
#     # Store history for plotting
#     x_est = np.zeros((t_steps, 5))
#     cov_trace = np.zeros(t_steps)

#     print(f"Simulating {t_steps} steps with new 5-state EKF...")
    
#     # ---------------------------------------------------------
#     # 5. Run Estimator Loop
#     # ---------------------------------------------------------
#     for k in range(t_steps):
        
#         # --- A. PREDICT STEP ---
#         # The new core uses Forward Accel + Yaw Rate for prediction
#         ekf.predict(
#             accel_fwd=meas_imu_accel[k], 
#             yaw_rate_meas=meas_imu_omega[k], 
#             dt=dt
#         )

#         # --- B. UPDATE: Wheel Velocity ---
#         # The new core doesn't have update_wheel, so we use the generic 'correct'
#         z_vel = np.zeros(5)
#         z_vel[3] = meas_wheel_v[k]  # Index 3 is Velocity
        
#         R_vel = np.zeros((5, 5))
#         R_vel[3, 3] = std_wheel_v**2
        
#         # Mask: Only update index 3 (Velocity)
#         update_mask = [False, False, False, True, False]
#         ekf.correct(z_vel, R_vel, update_mask)

#         # --- C. UPDATE: GPS Position (Low Rate) ---
#         if k % gps_step_interval == 0:
#             # We use the helper function provided in ekf_core
#             R_gps_2x2 = np.diag([std_gps_xy**2, std_gps_xy**2])
#             ekf.update_gps(meas_gps_xy[k, 0], meas_gps_xy[k, 1], R_gps_2x2)

#         # --- D. Store Data ---
#         x_est[k] = ekf.get_current_state()
#         cov_trace[k] = np.trace(ekf.P)

#     print("Simulation Complete. Plotting...")

#     # ---------------------------------------------------------
#     # 6. Plotting
#     # ---------------------------------------------------------
#     # Calculate errors (Compare index 0,1,2,3 with Truth)
#     pos_err = np.linalg.norm(x_est[:, 0:2] - x_true[:, 0:2], axis=1)
    
#     fig1 = plt.figure(figsize=(14, 8))

#     # Plot 1: Top-Down Trajectory
#     ax1 = fig1.add_subplot(2, 2, (1, 3)) 
#     ax1.plot(x_true[:, 0], x_true[:, 1], 'k-', linewidth=2, label='Ground Truth')
#     ax1.plot(x_est[:, 0], x_est[:, 1], 'b--', linewidth=2, label='EKF Estimate')
    
#     # Plot GPS dots
#     gps_idx = np.arange(0, t_steps, gps_step_interval)
#     ax1.scatter(meas_gps_xy[gps_idx, 0], meas_gps_xy[gps_idx, 1], 
#                 c='r', s=10, alpha=0.5, label='GPS Measurements')
    
#     ax1.set_title(f'Trajectory (2D Plane) - GPS Noise std: {std_gps_xy}m')
#     ax1.set_xlabel('X [m]')
#     ax1.set_ylabel('Y [m]')
#     ax1.axis('equal')
#     ax1.grid(True)
#     ax1.legend()

#     # Plot 2: Yaw Angle
#     ax2 = fig1.add_subplot(2, 2, 2)
#     ax2.plot(t, np.degrees(x_true[:, 2]), 'k-', label='True Yaw')
#     ax2.plot(t, np.degrees(x_est[:, 2]), 'b--', label='Est Yaw')
#     ax2.set_ylabel('Yaw [deg]')
#     ax2.set_title('Yaw Angle')
#     ax2.grid(True)
#     ax2.legend()

#     # Plot 3: Velocity
#     ax3 = fig1.add_subplot(2, 2, 4)
#     ax3.plot(t, x_true[:, 3], 'k-', label='True Speed')
#     ax3.plot(t, x_est[:, 3], 'b--', label='Est Speed')
#     ax3.set_ylabel('Speed [m/s]')
#     ax3.set_xlabel('Time [s]')
#     ax3.set_title('Velocity')
#     ax3.grid(True)
#     ax3.legend()

#     plt.tight_layout()
#     plt.savefig("ekf_trajectory.png")

#     # Plot 4: Error Analysis
#     fig2, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    
#     ax[0].plot(t, pos_err, 'r')
#     ax[0].set_ylabel('Pos Error [m]')
#     ax[0].set_title('Estimation Performance')
#     ax[0].grid(True)

#     ax[1].plot(t, cov_trace, 'g')
#     ax[1].set_ylabel('Trace(P)')
#     ax[1].set_xlabel('Time [s]')
#     ax[1].set_title('Covariance Trace (Uncertainty)')
#     ax[1].grid(True)
    
#     plt.tight_layout()
#     plt.savefig("ekf_simulation_results.png")
#     plt.show()

# if __name__ == "__main__":
#     simulate()







# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseWithCovarianceStamped
# import matplotlib.pyplot as plt
# import numpy as np
# import math
# import sys

# class DataLogger(Node):
#     def __init__(self):
#         super().__init__('data_logger')
        
#         # Buffers: [x, y, v, yaw, yaw_rate]
#         self.history_true = []
#         self.history_ekf = []
#         # Buffer: [x, y]
#         self.history_gps = []
        
#         self.counter = 0
#         self.limit = 5000  # Set duration of logging

#         # Subscribe to EKF, Ground Truth, and GPS
#         self.create_subscription(Odometry, '/ground_truth', self.cb_true, 10)
#         self.create_subscription(Odometry, '/odometry/ekf', self.cb_ekf, 10)
#         self.create_subscription(PoseWithCovarianceStamped, '/gps/enu_pose', self.cb_gps, 10)

#         self.get_logger().info("📊 COMPREHENSIVE LOGGER READY. Waiting for data...")

#     def get_yaw(self, q):
#         """Convert Quaternion to Yaw (Z-axis rotation)"""
#         # atan2(2(wz + xy), 1 - 2(ysq + zsq))
#         return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

#     def cb_true(self, msg):
#         yaw = self.get_yaw(msg.pose.pose.orientation)
#         self.history_true.append([
#             msg.pose.pose.position.x,      # 0: X
#             msg.pose.pose.position.y,      # 1: Y
#             msg.twist.twist.linear.x,      # 2: Velocity
#             yaw,                           # 3: Yaw
#             msg.twist.twist.angular.z      # 4: Yaw Rate
#         ])
        
#         # Check termination based on Truth messages
#         self.counter += 1
#         if self.counter % 500 == 0:
#             self.get_logger().info(f"Collected {self.counter}/{self.limit} steps...")
        
#         if self.counter >= self.limit:
#             self.plot_and_quit()

#     def cb_ekf(self, msg):
#         yaw = self.get_yaw(msg.pose.pose.orientation)
#         self.history_ekf.append([
#             msg.pose.pose.position.x,
#             msg.pose.pose.position.y,
#             msg.twist.twist.linear.x,
#             yaw,
#             msg.twist.twist.angular.z
#         ])

#     def cb_gps(self, msg):
#         self.history_gps.append([
#             msg.pose.pose.position.x,
#             msg.pose.pose.position.y
#         ])

#     def plot_and_quit(self):
#         self.get_logger().info("✅ DONE! Generating Comprehensive Plots...")
        
#         # Convert to numpy arrays
#         truth = np.array(self.history_true)
#         ekf = np.array(self.history_ekf)
#         gps = np.array(self.history_gps)

#         if len(ekf) == 0:
#             self.get_logger().error("NO EKF DATA RECEIVED! Check your nodes.")
#             sys.exit(1)

#         # Sync lengths (EKF might be slightly behind Truth)
#         min_len = min(len(truth), len(ekf))
#         truth = truth[:min_len]
#         ekf = ekf[:min_len]
#         steps = np.arange(min_len)

#         # --- CALCULATE ERRORS ---
#         # Position Error (Euclidean Distance)
#         pos_error = np.sqrt((truth[:,0] - ekf[:,0])**2 + (truth[:,1] - ekf[:,1])**2)
#         # Velocity Error
#         vel_error = truth[:,2] - ekf[:,2]
#         # Yaw Error (Wrapped)
#         yaw_error = np.arctan2(np.sin(truth[:,3] - ekf[:,3]), np.cos(truth[:,3] - ekf[:,3]))

#         # --- PLOTTING SETUP (4 Rows x 2 Columns) ---
#         fig, ax = plt.subplots(4, 2, figsize=(16, 20))
#         plt.subplots_adjust(hspace=0.4)

#         # 1. Trajectory (XY Plane)
#         ax[0, 0].set_title("1. 2D Trajectory (XY Plane)")
#         ax[0, 0].plot(truth[:,0], truth[:,1], 'k-', lw=2, label='Ground Truth')
#         ax[0, 0].plot(ekf[:,0], ekf[:,1], 'b--', lw=2, label='EKF Est')
#         if len(gps) > 0:
#             ax[0, 0].scatter(gps[:,0], gps[:,1], c='r', s=5, alpha=0.3, label='GPS Raw')
#         ax[0, 0].set_xlabel("X [m]")
#         ax[0, 0].set_ylabel("Y [m]")
#         ax[0, 0].axis('equal')
#         ax[0, 0].grid(True)
#         ax[0, 0].legend()

#         # 2. Position Error (Total)
#         ax[0, 1].set_title("2. Total Position Error (Euclidean)")
#         ax[0, 1].plot(steps, pos_error, 'r-', lw=1)
#         ax[0, 1].set_xlabel("Step")
#         ax[0, 1].set_ylabel("Error [m]")
#         ax[0, 1].grid(True)
#         ax[0, 1].fill_between(steps, pos_error, color='r', alpha=0.1)

#         # 3. X Position vs Time
#         ax[1, 0].set_title("3. X Position vs Time")
#         ax[1, 0].plot(steps, truth[:,0], 'k-', label='True')
#         ax[1, 0].plot(steps, ekf[:,0], 'b--', label='EKF')
#         ax[1, 0].set_ylabel("X [m]")
#         ax[1, 0].grid(True)
#         ax[1, 0].legend()

#         # 4. Y Position vs Time
#         ax[1, 1].set_title("4. Y Position vs Time")
#         ax[1, 1].plot(steps, truth[:,1], 'k-', label='True')
#         ax[1, 1].plot(steps, ekf[:,1], 'b--', label='EKF')
#         ax[1, 1].set_ylabel("Y [m]")
#         ax[1, 1].grid(True)
#         ax[1, 1].legend()

#         # 5. Velocity vs Time
#         ax[2, 0].set_title("5. Velocity Estimate")
#         ax[2, 0].plot(steps, truth[:,2], 'k-', label='True')
#         ax[2, 0].plot(steps, ekf[:,2], 'b--', label='EKF')
#         ax[2, 0].set_ylabel("Speed [m/s]")
#         ax[2, 0].grid(True)
#         ax[2, 0].legend()

#         # 6. Velocity Error
#         ax[2, 1].set_title("6. Velocity Error")
#         ax[2, 1].plot(steps, vel_error, 'g-', lw=1)
#         ax[2, 1].set_ylabel("Error [m/s]")
#         ax[2, 1].grid(True)
#         ax[2, 1].axhline(0, color='k', linestyle=':', alpha=0.5)

#         # 7. Yaw Angle (Heading)
#         ax[3, 0].set_title("7. Heading (Yaw) Angle")
#         ax[3, 0].plot(steps, np.degrees(truth[:,3]), 'k-', label='True')
#         ax[3, 0].plot(steps, np.degrees(ekf[:,3]), 'b--', label='EKF')
#         ax[3, 0].set_ylabel("Yaw [deg]")
#         ax[3, 0].set_xlabel("Step")
#         ax[3, 0].grid(True)
#         ax[3, 0].legend()

#         # 8. Yaw Rate (Angular Velocity)
#         ax[3, 1].set_title("8. Yaw Rate (Z-Gyro)")
#         ax[3, 1].plot(steps, truth[:,4], 'k-', label='True')
#         ax[3, 1].plot(steps, ekf[:,4], 'b--', label='EKF')
#         ax[3, 1].set_ylabel("Rate [rad/s]")
#         ax[3, 1].set_xlabel("Step")
#         ax[3, 1].grid(True)
#         ax[3, 1].legend()

#         filename = "ros_ekf_comprehensive.png"
#         plt.savefig(filename)
#         self.get_logger().info(f"💾 SAVED 8-PLOT GRID TO: {filename}")
        
#         plt.show()
#         sys.exit(0)

# def main(args=None):
#     rclpy.init(args=args)
#     node = DataLogger()
#     rclpy.spin(node)

# if __name__ == '__main__': main()





import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

import matplotlib.pyplot as plt
import numpy as np
import math
import sys

class RealDataLogger(Node):
    def __init__(self):
        super().__init__('real_data_logger')
        
        # --- QOS PROFILES ---
        ekf_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- BUFFERS ---
        self.data_ekf = []      # [t, x, y, v, yaw]
        self.data_gps_pos = []  # [t, x, y]
        self.data_gps_vel = []  # [t, speed_vector_magnitude]
        self.data_rmc_speed = [] # [t, speed_scalar_rmc]
        self.data_imu_yaw = []  # [t, yaw]
        
        self.start_t_epoch = None 
        self.last_valid_time = None 
        self.ekf_connected = False 

        # --- SUBSCRIPTIONS ---
        self.create_subscription(Odometry, '/odometry/ekf', self.cb_ekf, ekf_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/gps/enu_pose', self.cb_gps_pos, sensor_qos)
        self.create_subscription(TwistWithCovarianceStamped, '/gps/fix_velocity', self.cb_gps_vel, sensor_qos)
        self.create_subscription(Imu, '/imu/data_raw', self.cb_imu, sensor_qos)
        self.create_subscription(Float32, '/gps/speed_kmph', self.cb_rmc_speed, sensor_qos)

        self.get_logger().info("📊 LOGGER READY. Waiting for data... (Ctrl+C to stop)")

    def get_rel_time(self, stamp):
        t_sec = stamp.sec + stamp.nanosec * 1e-9
        if self.start_t_epoch is None:
            self.start_t_epoch = t_sec
        
        rel_time = t_sec - self.start_t_epoch
        self.last_valid_time = rel_time 
        return rel_time

    def get_yaw(self, q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def cb_ekf(self, msg):
        if not self.ekf_connected:
            self.get_logger().info("✅ Connected to EKF! Recording data...")
            self.ekf_connected = True

        t = self.get_rel_time(msg.header.stamp)
        yaw = self.get_yaw(msg.pose.pose.orientation)
        
        self.data_ekf.append([
            t,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.twist.twist.linear.x,
            yaw
        ])

    def cb_gps_pos(self, msg):
        t = self.get_rel_time(msg.header.stamp)
        self.data_gps_pos.append([t, msg.pose.pose.position.x, msg.pose.pose.position.y])

    def cb_gps_vel(self, msg):
        t = self.get_rel_time(msg.header.stamp)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx**2 + vy**2)
        self.data_gps_vel.append([t, speed])

    def cb_rmc_speed(self, msg):
        if self.last_valid_time is None: return 
        t = self.last_valid_time
        speed_mps = msg.data / 3.6 
        self.data_rmc_speed.append([t, speed_mps])

    def cb_imu(self, msg):
        t = self.get_rel_time(msg.header.stamp)
        yaw = self.get_yaw(msg.orientation)
        self.data_imu_yaw.append([t, yaw])

    def save_plots(self):
        self.get_logger().info("🛑 STOPPING... Generating 4 Analysis Files...")
        
        ekf = np.array(self.data_ekf)
        gps_pos = np.array(self.data_gps_pos)
        gps_vel = np.array(self.data_gps_vel)
        rmc_speed = np.array(self.data_rmc_speed)
        imu_yaw = np.array(self.data_imu_yaw)

        if len(ekf) == 0:
            self.get_logger().error("No EKF Data collected!")
            return

        # ==========================================
        # FILE 1: DASHBOARD OVERVIEW (2x2)
        # ==========================================
        fig1, ax1 = plt.subplots(2, 2, figsize=(16, 10))
        plt.suptitle("1. SYSTEM OVERVIEW", fontsize=16)
        
        # 1. Trajectory
        ax1[0,0].set_title("Trajectory Map")
        ax1[0,0].plot(ekf[:,1], ekf[:,2], 'b-', lw=2, label='EKF')
        if len(gps_pos) > 0: ax1[0,0].scatter(gps_pos[:,1], gps_pos[:,2], c='r', marker='x', s=20, label='GPS')
        ax1[0,0].axis('equal'); ax1[0,0].grid(True); ax1[0,0].legend()

        # 2. Speed
        ax1[0,1].set_title("Speed Profile")
        ax1[0,1].plot(ekf[:,0], ekf[:,3], 'b-', label='EKF Vx')
        if len(rmc_speed) > 0: ax1[0,1].plot(rmc_speed[:,0], rmc_speed[:,1], 'm--', alpha=0.7, label='GPS (RMC)')
        ax1[0,1].grid(True); ax1[0,1].legend()

        # 3. Yaw
        ax1[1,0].set_title("Heading (Yaw)")
        ax1[1,0].plot(ekf[:,0], np.degrees(ekf[:,4]), 'k-', label='EKF Yaw')
        if len(imu_yaw) > 0: ax1[1,0].scatter(imu_yaw[:,0], np.degrees(imu_yaw[:,1]), c='orange', s=5, alpha=0.3, label='IMU')
        ax1[1,0].grid(True); ax1[1,0].legend()

        # 4. X/Y Time Series
        ax1[1,1].set_title("Position Components")
        ax1[1,1].plot(ekf[:,0], ekf[:,1], 'b-', label='X')
        ax1[1,1].plot(ekf[:,0], ekf[:,2], 'r--', label='Y')
        ax1[1,1].grid(True); ax1[1,1].legend()
        
        plt.savefig("1_Overview.png")
        self.get_logger().info("💾 Saved 1_Overview.png")
        plt.close()

        # ==========================================
        # FILE 2: POSITION ANALYSIS (2x1)
        # ==========================================
        fig2, ax2 = plt.subplots(2, 1, figsize=(12, 10))
        plt.suptitle("2. POSITION DRIFT ANALYSIS", fontsize=16)

        # X Analysis
        ax2[0].set_title("X Position vs Time")
        ax2[0].plot(ekf[:,0], ekf[:,1], 'b-', lw=2, label='EKF X')
        if len(gps_pos) > 0: ax2[0].plot(gps_pos[:,0], gps_pos[:,1], 'r--', lw=1.5, label='GPS X')
        ax2[0].grid(True); ax2[0].legend()

        # Y Analysis
        ax2[1].set_title("Y Position vs Time")
        ax2[1].plot(ekf[:,0], ekf[:,2], 'b-', lw=2, label='EKF Y')
        if len(gps_pos) > 0: ax2[1].plot(gps_pos[:,0], gps_pos[:,2], 'r--', lw=1.5, label='GPS Y')
        ax2[1].grid(True); ax2[1].legend()

        plt.savefig("2_Position_Detail.png")
        self.get_logger().info("💾 Saved 2_Position_Detail.png")
        plt.close()

        # ==========================================
        # FILE 3: VELOCITY SOURCES (2x1)
        # ==========================================
        fig3, ax3 = plt.subplots(2, 1, figsize=(12, 10))
        plt.suptitle("3. VELOCITY SOURCE COMPARISON", fontsize=16)

        # Source Comparison
        ax3[0].set_title("Speed Source Overlay")
        ax3[0].plot(ekf[:,0], ekf[:,3], 'b-', lw=2, label='EKF Output')
        if len(rmc_speed) > 0: ax3[0].plot(rmc_speed[:,0], rmc_speed[:,1], 'm--', label='GPS RMC (Scalar)')
        if len(gps_vel) > 0: ax3[0].scatter(gps_vel[:,0], gps_vel[:,1], c='g', s=15, alpha=0.5, label='GPS VTG (Vector)')
        ax3[0].grid(True); ax3[0].legend()

        # Stability Check (Zoomed)
        ax3[1].set_title("Speed Stability (Zoomed)")
        ax3[1].plot(ekf[:,0], ekf[:,3], 'b.-', lw=1, label='EKF Output')
        ax3[1].set_ylim(bottom=-0.5, top=max(ekf[:,3]) + 1.0)
        ax3[1].grid(True)

        plt.savefig("3_Velocity_Analysis.png")
        self.get_logger().info("💾 Saved 3_Velocity_Analysis.png")
        plt.close()

        # ==========================================
        # FILE 4: HEADING & PATH (2x1)
        # ==========================================
        fig4, ax4 = plt.subplots(2, 1, figsize=(12, 10))
        plt.suptitle("4. HEADING & PATH ALIGNMENT", fontsize=16)

        # Yaw Detail
        ax4[0].set_title("Heading Alignment (EKF vs Raw IMU)")
        ax4[0].plot(ekf[:,0], np.degrees(ekf[:,4]), 'k-', lw=2, label='EKF (Corrected)')
        if len(imu_yaw) > 0: ax4[0].plot(imu_yaw[:,0], np.degrees(imu_yaw[:,1]), 'orange', alpha=0.6, label='Raw IMU')
        ax4[0].grid(True); ax4[0].legend()

        # Path Shape
        ax4[1].set_title("Resulting Path Shape")
        ax4[1].plot(ekf[:,1], ekf[:,2], 'b-', lw=2)
        ax4[1].axis('equal')
        ax4[1].grid(True)
        ax4[1].set_xlabel("X (m)")
        ax4[1].set_ylabel("Y (m)")

        plt.savefig("4_Heading_Analysis.png")
        self.get_logger().info("💾 Saved 4_Heading_Analysis.png")
        plt.close()

def main(args=None):
    rclpy.init(args=args)
    node = RealDataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_plots()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()