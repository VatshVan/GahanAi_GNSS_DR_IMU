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







#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import matplotlib.pyplot as plt
import numpy as np
import math
import sys

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        
        # Buffers: [x, y, v, yaw, yaw_rate]
        self.history_true = []
        self.history_ekf = []
        # Buffer: [x, y]
        self.history_gps = []
        
        self.counter = 0
        self.limit = 5000  # Set duration of logging

        # Subscribe to EKF, Ground Truth, and GPS
        self.create_subscription(Odometry, '/ground_truth', self.cb_true, 10)
        self.create_subscription(Odometry, '/odometry/ekf', self.cb_ekf, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/gps/enu_pose', self.cb_gps, 10)

        self.get_logger().info("📊 COMPREHENSIVE LOGGER READY. Waiting for data...")

    def get_yaw(self, q):
        """Convert Quaternion to Yaw (Z-axis rotation)"""
        # atan2(2(wz + xy), 1 - 2(ysq + zsq))
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def cb_true(self, msg):
        yaw = self.get_yaw(msg.pose.pose.orientation)
        self.history_true.append([
            msg.pose.pose.position.x,      # 0: X
            msg.pose.pose.position.y,      # 1: Y
            msg.twist.twist.linear.x,      # 2: Velocity
            yaw,                           # 3: Yaw
            msg.twist.twist.angular.z      # 4: Yaw Rate
        ])
        
        # Check termination based on Truth messages
        self.counter += 1
        if self.counter % 500 == 0:
            self.get_logger().info(f"Collected {self.counter}/{self.limit} steps...")
        
        if self.counter >= self.limit:
            self.plot_and_quit()

    def cb_ekf(self, msg):
        yaw = self.get_yaw(msg.pose.pose.orientation)
        self.history_ekf.append([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.twist.twist.linear.x,
            yaw,
            msg.twist.twist.angular.z
        ])

    def cb_gps(self, msg):
        self.history_gps.append([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

    def plot_and_quit(self):
        self.get_logger().info("✅ DONE! Generating Comprehensive Plots...")
        
        # Convert to numpy arrays
        truth = np.array(self.history_true)
        ekf = np.array(self.history_ekf)
        gps = np.array(self.history_gps)

        if len(ekf) == 0:
            self.get_logger().error("NO EKF DATA RECEIVED! Check your nodes.")
            sys.exit(1)

        # Sync lengths (EKF might be slightly behind Truth)
        min_len = min(len(truth), len(ekf))
        truth = truth[:min_len]
        ekf = ekf[:min_len]
        steps = np.arange(min_len)

        # --- CALCULATE ERRORS ---
        # Position Error (Euclidean Distance)
        pos_error = np.sqrt((truth[:,0] - ekf[:,0])**2 + (truth[:,1] - ekf[:,1])**2)
        # Velocity Error
        vel_error = truth[:,2] - ekf[:,2]
        # Yaw Error (Wrapped)
        yaw_error = np.arctan2(np.sin(truth[:,3] - ekf[:,3]), np.cos(truth[:,3] - ekf[:,3]))

        # --- PLOTTING SETUP (4 Rows x 2 Columns) ---
        fig, ax = plt.subplots(4, 2, figsize=(16, 20))
        plt.subplots_adjust(hspace=0.4)

        # 1. Trajectory (XY Plane)
        ax[0, 0].set_title("1. 2D Trajectory (XY Plane)")
        ax[0, 0].plot(truth[:,0], truth[:,1], 'k-', lw=2, label='Ground Truth')
        ax[0, 0].plot(ekf[:,0], ekf[:,1], 'b--', lw=2, label='EKF Est')
        if len(gps) > 0:
            ax[0, 0].scatter(gps[:,0], gps[:,1], c='r', s=5, alpha=0.3, label='GPS Raw')
        ax[0, 0].set_xlabel("X [m]")
        ax[0, 0].set_ylabel("Y [m]")
        ax[0, 0].axis('equal')
        ax[0, 0].grid(True)
        ax[0, 0].legend()

        # 2. Position Error (Total)
        ax[0, 1].set_title("2. Total Position Error (Euclidean)")
        ax[0, 1].plot(steps, pos_error, 'r-', lw=1)
        ax[0, 1].set_xlabel("Step")
        ax[0, 1].set_ylabel("Error [m]")
        ax[0, 1].grid(True)
        ax[0, 1].fill_between(steps, pos_error, color='r', alpha=0.1)

        # 3. X Position vs Time
        ax[1, 0].set_title("3. X Position vs Time")
        ax[1, 0].plot(steps, truth[:,0], 'k-', label='True')
        ax[1, 0].plot(steps, ekf[:,0], 'b--', label='EKF')
        ax[1, 0].set_ylabel("X [m]")
        ax[1, 0].grid(True)
        ax[1, 0].legend()

        # 4. Y Position vs Time
        ax[1, 1].set_title("4. Y Position vs Time")
        ax[1, 1].plot(steps, truth[:,1], 'k-', label='True')
        ax[1, 1].plot(steps, ekf[:,1], 'b--', label='EKF')
        ax[1, 1].set_ylabel("Y [m]")
        ax[1, 1].grid(True)
        ax[1, 1].legend()

        # 5. Velocity vs Time
        ax[2, 0].set_title("5. Velocity Estimate")
        ax[2, 0].plot(steps, truth[:,2], 'k-', label='True')
        ax[2, 0].plot(steps, ekf[:,2], 'b--', label='EKF')
        ax[2, 0].set_ylabel("Speed [m/s]")
        ax[2, 0].grid(True)
        ax[2, 0].legend()

        # 6. Velocity Error
        ax[2, 1].set_title("6. Velocity Error")
        ax[2, 1].plot(steps, vel_error, 'g-', lw=1)
        ax[2, 1].set_ylabel("Error [m/s]")
        ax[2, 1].grid(True)
        ax[2, 1].axhline(0, color='k', linestyle=':', alpha=0.5)

        # 7. Yaw Angle (Heading)
        ax[3, 0].set_title("7. Heading (Yaw) Angle")
        ax[3, 0].plot(steps, np.degrees(truth[:,3]), 'k-', label='True')
        ax[3, 0].plot(steps, np.degrees(ekf[:,3]), 'b--', label='EKF')
        ax[3, 0].set_ylabel("Yaw [deg]")
        ax[3, 0].set_xlabel("Step")
        ax[3, 0].grid(True)
        ax[3, 0].legend()

        # 8. Yaw Rate (Angular Velocity)
        ax[3, 1].set_title("8. Yaw Rate (Z-Gyro)")
        ax[3, 1].plot(steps, truth[:,4], 'k-', label='True')
        ax[3, 1].plot(steps, ekf[:,4], 'b--', label='EKF')
        ax[3, 1].set_ylabel("Rate [rad/s]")
        ax[3, 1].set_xlabel("Step")
        ax[3, 1].grid(True)
        ax[3, 1].legend()

        filename = "ros_ekf_comprehensive.png"
        plt.savefig(filename)
        self.get_logger().info(f"💾 SAVED 8-PLOT GRID TO: {filename}")
        
        plt.show()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    rclpy.spin(node)

if __name__ == '__main__': main()