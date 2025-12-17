"""
ekf_sim.py

Simulation and plotting for the EKF defined in ekf_core.py.
Simulates a ground vehicle following a curved trajectory and fuses:
 - IMU yaw-rate (Prediction step input)
 - Wheel speed (Update step)
 - GPS position (Update step)
"""

import numpy as np
import matplotlib.pyplot as plt
import sys

# Try importing the EKF class
try:
    from ekf_core import EKF
except ImportError:
    print("Error: Could not import 'EKF' from 'ekf_core.py'.")
    print("Make sure ekf_core.py is in the same directory.")
    sys.exit(1)

def wrap_angle(a):
    """Normalize angle to [-pi, pi]"""
    return (a + np.pi) % (2*np.pi) - np.pi

def angle_diff(a, b):
    """Calculate smallest difference between two angles"""
    d = a - b
    return wrap_angle(d)

def simulate():
    # 1. Setup Simulation Parameters
    np.random.seed(42) # Fixed seed for reproducibility
    sim_time = 80.0    # Total time [s]
    dt = 0.02          # Time step [s] (50 Hz)
    t_steps = int(sim_time / dt)
    t = np.arange(t_steps) * dt

    # Sensor Rates
    gps_dt = 0.5       # GPS updates every 0.5s (2 Hz)
    gps_step_interval = int(gps_dt / dt)

    # 2. Generate True Trajectory
    # We create a synthetic path using sinusoidal inputs
    true_omega = 0.08 * np.sin(0.1 * t) + 0.04 * np.sin(0.3 * t) # Varying yaw rate
    true_acc   = 0.05 * np.cos(0.05 * t)                         # Varying acceleration

    # State: [x, y, yaw, v]
    x_true = np.zeros((t_steps, 4))
    x_true[0] = [0.0, 0.0, 0.0, 2.0] # Initial state (start moving at 2 m/s)

    for k in range(1, t_steps):
        px, py, yaw, v = x_true[k-1]
        
        # Physics Integration (Euler)
        yaw_new = wrap_angle(yaw + true_omega[k-1] * dt)
        v_new   = max(0.0, v + true_acc[k-1] * dt) # Constraint: No reversing
        px_new  = px + v_new * np.cos(yaw_new) * dt
        py_new  = py + v_new * np.sin(yaw_new) * dt
        
        x_true[k] = [px_new, py_new, yaw_new, v_new]

    # 3. Generate Noisy Measurements
    # Noise Standard Deviations
    std_imu_omega = np.deg2rad(1.0) # 1.0 deg/s noise
    std_gps_xy    = 1.5             # 1.5 m GPS noise
    std_wheel_v   = 0.2             # 0.2 m/s Speedometer noise

    # Add Gaussian noise
    meas_imu_omega = true_omega + np.random.randn(t_steps) * std_imu_omega
    meas_gps_xy    = x_true[:, 0:2] + np.random.randn(t_steps, 2) * std_gps_xy
    meas_wheel_v   = x_true[:, 3] + np.random.randn(t_steps) * std_wheel_v

    # 4. Initialize EKF
    ekf = EKF()
    
    # Initial Guess (Intentionally slightly wrong to verify convergence)
    ekf.x = np.array([-2.0, 1.0, np.deg2rad(5.0), 1.0]) 
    
    # Initial Covariance (Uncertainty)
    ekf.P = np.diag([5.0, 5.0, np.deg2rad(30)**2, 2.0**2])

    # Tunable Noise Matrices
    # Q: Process Noise (How much we trust the prediction model)
    ekf.Q = np.diag([
        0.05**2,             # x variance
        0.05**2,             # y variance
        np.deg2rad(0.5)**2,  # yaw variance
        0.1**2               # velocity variance
    ])
    
    # R: Measurement Noise (How much we trust sensors)
    ekf.R_gps = np.diag([std_gps_xy**2, std_gps_xy**2]) # GPS covariance
    ekf.R_wheel = np.array([[std_wheel_v**2]])          # Wheel speed covariance

    # 5. Run Estimator Loop
    x_est = np.zeros((t_steps, 4))
    cov_trace = np.zeros(t_steps)

    print(f"Simulating {t_steps} steps...")
    
    for k in range(t_steps):
        # A. PREDICT STEP (High Rate)
        # Use IMU yaw rate as control input
        ekf.predict(omega=meas_imu_omega[k], dt=dt)

        # B. UPDATE STEPS
        # 1. Wheel Speed Update (High Rate)
        ekf.update_wheel(meas_wheel_v[k])

        # 2. GPS Update (Low Rate)
        if k % gps_step_interval == 0:
            ekf.update_gps(meas_gps_xy[k])

        # C. STORE RESULTS
        x_est[k] = ekf.x
        cov_trace[k] = np.trace(ekf.P) # Sum of diagonal variance (measure of total uncertainty)

    print("Simulation Complete. Plotting...")

    # 6. Plotting
    # Calculate errors
    pos_err = np.linalg.norm(x_est[:, 0:2] - x_true[:, 0:2], axis=1)
    yaw_err = np.abs(angle_diff(x_est[:, 2], x_true[:, 2]))
    
    fig1 = plt.figure(figsize=(14, 8))

    # Plot 1: Top-Down Trajectory
    ax1 = fig1.add_subplot(2, 2, (1, 3)) # Spans left column
    ax1.plot(x_true[:, 0], x_true[:, 1], 'k-', linewidth=2, label='Ground Truth')
    ax1.plot(x_est[:, 0], x_est[:, 1], 'b--', linewidth=2, label='EKF Estimate')
    
    # Plot GPS dots (subsampled)
    gps_idx = np.arange(0, t_steps, gps_step_interval)
    ax1.scatter(meas_gps_xy[gps_idx, 0], meas_gps_xy[gps_idx, 1], 
                c='r', s=10, alpha=0.5, label='GPS Measurements')
    
    ax1.set_title(f'Trajectory (2D Plane) - GPS Noise std: {std_gps_xy}m')
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.axis('equal')
    ax1.grid(True)
    ax1.legend()

    # Plot 2: States vs Time
    ax2 = fig1.add_subplot(2, 2, 2)
    ax2.plot(t, np.degrees(x_true[:, 2]), 'k-', label='True Yaw')
    ax2.plot(t, np.degrees(x_est[:, 2]), 'b--', label='Est Yaw')
    ax2.set_ylabel('Yaw [deg]')
    ax2.set_title('Yaw Angle')
    ax2.grid(True)
    ax2.legend()

    ax3 = fig1.add_subplot(2, 2, 4)
    ax3.plot(t, x_true[:, 3], 'k-', label='True Speed')
    ax3.plot(t, x_est[:, 3], 'b--', label='Est Speed')
    ax3.set_ylabel('Speed [m/s]')
    ax3.set_xlabel('Time [s]')
    ax3.set_title('Velocity')
    ax3.grid(True)
    ax3.legend()

    plt.tight_layout()

    # Plot 3: Error Analysis
    fig2, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    
    ax[0].plot(t, pos_err, 'r')
    ax[0].set_ylabel('Pos Error [m]')
    ax[0].set_title('Estimation Performance')
    ax[0].grid(True)

    ax[1].plot(t, cov_trace, 'g')
    ax[1].set_ylabel('Trace(P)')
    ax[1].set_xlabel('Time [s]')
    ax[1].set_title('Covariance Trace (Uncertainty)')
    ax[1].grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    simulate()