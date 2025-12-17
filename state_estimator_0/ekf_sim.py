"""
ekf_sim.py

Simulation and plotting for the EKF defined in ekf_core.py.

Simulates a ground vehicle following a curved trajectory. Generates:
 - IMU yaw-rate measurements (high-rate)
 - Wheel speed measurements (high-rate)
 - GPS position measurements (low-rate)

Runs the EKF and plots:
 - True vs estimated 2D trajectory
 - True vs estimated yaw and speed vs time
 - Position error over time

Run:
    python ekf_sim.py
"""

import numpy as np
import matplotlib.pyplot as plt
from ekf_core import EKF

def wrap_angle(a):
    return (a + np.pi) % (2*np.pi) - np.pi

def simulate():
    np.random.seed(1)

    # Simulation parameters
    sim_time = 80.0          # seconds
    dt = 0.02                # simulation step (50 Hz)
    t_steps = int(sim_time / dt)

    # Sensor rates
    imu_rate = 1.0 / dt      # IMU at every sim step (i.e., dt)
    wheel_rate = 1.0 / dt    # wheel speed same as dt
    gps_dt = 0.5             # GPS at 2 Hz
    gps_steps = int(gps_dt / dt)

    # True trajectory parameters (we will create a varying yaw-rate and acceleration)
    t = np.arange(t_steps) * dt

    # Create a smooth yaw-rate profile: small sinusoidal steering actions
    true_omega = 0.06 * np.sin(0.08 * t) + 0.03 * np.sin(0.4 * t)

    # True acceleration profile to vary speed moderately
    true_acc = 0.02 * np.sin(0.05 * t) + 0.01 * np.cos(0.17 * t)

    # Initialize true state
    x_true = np.zeros((t_steps, 4))  # px, py, yaw, v
    x_true[0, :] = np.array([0.0, 0.0, 0.0, 2.0])  # start at 2 m/s

    for k in range(1, t_steps):
        px, py, yaw, v = x_true[k-1]
        omega = true_omega[k-1]
        a = true_acc[k-1]

        # integrate true dynamics (simple bicycle-free point model)
        yaw_new = yaw + omega * dt
        v_new = max(0.0, v + a * dt)  # avoid negative speeds
        px_new = px + v_new * dt * np.cos(yaw_new)
        py_new = py + v_new * dt * np.sin(yaw_new)

        x_true[k] = np.array([px_new, py_new, wrap_angle(yaw_new), v_new])

    # Sensor noise parameters (std dev)
    imu_omega_std = np.deg2rad(0.5)  # rad/s
    wheel_v_std = 0.1                # m/s
    gps_pos_std = 1.2                # meters

    # Generate measurements
    imu_meas = true_omega + np.random.randn(t_steps) * imu_omega_std
    wheel_meas = x_true[:,3] + np.random.randn(t_steps) * wheel_v_std
    gps_meas = x_true[:,0:2] + np.random.randn(t_steps,2) * gps_pos_std

    # Initialize EKF
    ekf = EKF()
    # give EKF a poorer initial position guess
    ekf.x = np.array([ -5.0,  3.0, np.deg2rad(10.0), 1.5])
    ekf.P = np.diag([10.0, 10.0, (np.deg2rad(30.0))**2, 1.0])

    # Optionally tune Q and R for better results
    ekf.Q = np.diag([0.02, 0.02, (np.deg2rad(0.2))**2, 0.05**2])
    ekf.R_gps = np.diag([gps_pos_std**2, gps_pos_std**2])
    ekf.R_wheel = np.array([[wheel_v_std**2]])

    # Storage for estimated states
    x_est = np.zeros_like(x_true)
    P_trace = np.zeros(t_steps)
    pos_err = np.zeros(t_steps)

    for k in range(t_steps):
        omega_m = imu_meas[k]
        # Predict
        ekf.predict(omega=omega_m, dt=dt)

        # Wheel update every step
        ekf.update_wheel(wheel_meas[k])

        # GPS update at lower rate
        if (k % gps_steps) == 0:
            ekf.update_gps(gps_meas[k])

        x_est[k] = ekf.get_state()
        P_trace[k] = np.trace(ekf.get_cov())
        pos_err[k] = np.linalg.norm(x_est[k,0:2] - x_true[k,0:2])

    # Plotting
    fig = plt.figure(figsize=(12,10))

    # Trajectory plot
    ax1 = fig.add_subplot(2,2,1)
    ax1.plot(x_true[:,0], x_true[:,1], label='True', linewidth=2)
    ax1.plot(x_est[:,0], x_est[:,1], label='EKF estimate', linestyle='--')
    ax1.scatter(gps_meas[::gps_steps,0], gps_meas[::gps_steps,1],
                s=10, c='tab:orange', label='GPS measurements', alpha=0.6)
    ax1.set_title('XY Trajectory')
    ax1.axis('equal')
    ax1.legend()
    ax1.grid(True)

    # X/Y vs time
    ax2 = fig.add_subplot(2,2,2)
    ax2.plot(t, x_true[:,0], label='x true')
    ax2.plot(t, x_est[:,0], label='x est', linestyle='--')
    ax2.plot(t, x_true[:,1], label='y true')
    ax2.plot(t, x_est[:,1], label='y est', linestyle='--')
    ax2.set_title('Position components vs time')
    ax2.set_xlabel('time [s]')
    ax2.legend()
    ax2.grid(True)

    # Yaw and speed
    ax3 = fig.add_subplot(2,2,3)
    ax3.plot(t, x_true[:,2], label='yaw true')
    ax3.plot(t, x_est[:,2], label='yaw est', linestyle='--')
    ax3.set_title('Yaw (rad)')
    ax3.legend()
    ax3.grid(True)

    ax4 = fig.add_subplot(2,2,4)
    ax4.plot(t, x_true[:,3], label='v true')
    ax4.plot(t, x_est[:,3], label='v est', linestyle='--')
    ax4.set_title('Speed (m/s)')
    ax4.legend()
    ax4.grid(True)

    plt.tight_layout()
    plt.show()

    # Error plots
    fig2, ax = plt.subplots(2,1, figsize=(9,6), sharex=True)
    ax[0].plot(t, pos_err)
    ax[0].set_ylabel('Position error [m]')
    ax[0].grid(True)

    ax[1].plot(t, P_trace)
    ax[1].set_ylabel('Trace(P)')
    ax[1].set_xlabel('time [s]')
    ax[1].grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    simulate()
