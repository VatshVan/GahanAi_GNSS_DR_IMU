"""
ekf_core.py

Extended Kalman Filter for a ground vehicle with state:
    x = [px, py, yaw, v]^T

Core Logic:
    - PREDICT: Uses IMU (Yaw Rate + Forward Accel) to propagate state. 
               This handles "Dead Reckoning" when GPS is lost.
    - UPDATE:  Fuses GPS [x, y] to correct position drift.
"""
import numpy as np
from collections import deque
from math import sin, cos, atan2, sqrt, pi

# ------------------ Utilities ------------------

def normalize_angle(a):
    return (a + pi) % (2 * pi) - pi


# ------------------ EKF ------------------

class EKF:
    """
    State vector:
        x = [pos_x, pos_y, yaw, velocity, yaw_rate]
    Covariance matrix:
        P = 5x5 covariance matrix
    Process noise:
        Q = 5x5 process noise matrix
    Methods:
        - predict(accel_fwd, yaw_rate_meas, dt)
        - correct(z, R, update_vector)
        - check_alignment(x_gps, y_gps)
        - update_gps(x_gps, y_gps, R_gps_2x2)
        - get_current_state()
    """

    X = 0
    Y = 1
    YAW = 2
    V = 3
    YAW_RATE = 4

    def __init__(self):
        # -------- State --------
        self.state = np.zeros(5)

        # -------- Covariances --------
        self.P = np.diag([1.0, 1.0, 0.1, 1.0, 0.1])

        # -------- Process Noise (Q) - UPDATED FOR REAL LIFE --------
        # Low values = High Inertia (Smooth). High values = Twitchy.

        # -------- Process Noise (Q) - FINE TUNED --------
        self.Q = np.diag([
            0.001,  # X: Very low (Smooth path)
            0.001,  # Y: Very low
            0.001,  # Yaw: No instant snap-turns
            0.005,  # Velocity: VERY LOW (High Inertia/Mass) <--- CRITICAL CHANGE
            0.01    # Yaw Rate: Gyro bias drifts slowly
        ])

        # -------- Yaw bootstrap --------
        self.is_yaw_initialized = False
        self.start_gps_x = None
        self.start_gps_y = None
        self.min_align_distance = 4.0  # meters
        self.last_gps_x = None
        self.last_gps_y = None
        self.last_gps_time = None
        self.min_gps_heading_speed = 1.5  # m/s

    def update_gps_heading(self, yaw_gps, var_yaw):
        z = np.zeros(5)
        z[self.YAW] = yaw_gps

        R = np.zeros((5, 5))
        R[self.YAW, self.YAW] = var_yaw

        update_vector = [False, False, True, False, False]
        self.correct(z, R, update_vector)
    
    def update_wheel_speed(self, v_meas, var_v):
        """
        Wheel odometry update: measures forward velocity only
        """
        z = np.zeros(5)
        z[self.V] = v_meas

        R = np.zeros((5, 5))
        R[self.V, self.V] = var_v

        update_vector = [False, False, False, True, False]
        self.correct(z, R, update_vector)

    # -------------------------------------------------
    # Prediction
    # -------------------------------------------------
    def predict(self, accel_fwd, yaw_rate_meas, dt):
        x, y, yaw, v, yaw_rate = self.state

        # ----- State prediction f(x) -----
        yaw_new = normalize_angle(yaw + yaw_rate_meas * dt)

        v_new = v + accel_fwd * dt
        # v_new *= 0.99  # drag
        v_new = np.clip(v_new, -5.0, 5.0)

        x_new = x + v * cos(yaw) * dt
        y_new = y + v * sin(yaw) * dt

        self.state = np.array([
            x_new,
            y_new,
            yaw_new,
            v_new,
            yaw_rate_meas
        ])

        # ----- Jacobian F = ∂f/∂x -----
        F = np.eye(5)
        F[self.X, self.YAW] = -v * sin(yaw) * dt
        F[self.X, self.V]   =  cos(yaw) * dt
        F[self.Y, self.YAW] =  v * cos(yaw) * dt
        F[self.Y, self.V]   =  sin(yaw) * dt
        F[self.YAW, self.YAW_RATE] = dt

        # ----- Covariance prediction -----
        self.P = F @ self.P @ F.T + self.Q * dt

    # -------------------------------------------------
    # Generic EKF correction (robot_localization::correct)
    # -------------------------------------------------
    def correct(self, z, R, update_vector):
        """
        z : full measurement vector (size 5)
        R : full measurement covariance (5x5)
        update_vector : list[bool] of size 5
        """

        # ----- Select valid update indices -----
        update_indices = []
        for i, flag in enumerate(update_vector):
            if flag and np.isfinite(z[i]):
                update_indices.append(i)

        if not update_indices:
            return

        m = len(update_indices)

        # ----- Build sub-vectors -----
        x_sub = self.state[update_indices]
        z_sub = z[update_indices]

        R_sub = R[np.ix_(update_indices, update_indices)]

        # Handle bad covariances (exact ROS behavior)
        for i in range(m):
            if R_sub[i, i] < 0:
                R_sub[i, i] = abs(R_sub[i, i])
            if R_sub[i, i] < 1e-9:
                R_sub[i, i] = 1e-9

        # ----- Measurement matrix H -----
        H = np.zeros((m, 5))
        for row, idx in enumerate(update_indices):
            H[row, idx] = 1.0

        # ----- Innovation -----
        innovation = z_sub - x_sub

        # Angle wrapping on innovation
        for i, idx in enumerate(update_indices):
            if idx == self.YAW:
                innovation[i] = normalize_angle(innovation[i])

        # ----- Kalman gain -----
        S = H @ self.P @ H.T + R_sub
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        K = self.P @ H.T @ S_inv

        # ----- State update -----
        self.state += K @ innovation
        self.state[self.YAW] = normalize_angle(self.state[self.YAW])

        # ----- Joseph-form covariance update -----
        I = np.eye(5)
        KH = K @ H
        self.P = (I - KH) @ self.P @ (I - KH).T + K @ R_sub @ K.T

    # -------------------------------------------------
    # GPS yaw bootstrap (same logic you already used)
    # -------------------------------------------------
    # def check_alignment(self, x_gps, y_gps):
    #     if self.start_gps_x is None:
    #         self.start_gps_x = x_gps
    #         self.start_gps_y = y_gps
    #         return False

    #     dx = x_gps - self.start_gps_x
    #     dy = y_gps - self.start_gps_y
    #     dist = sqrt(dx * dx + dy * dy)

    #     if dist > self.min_align_distance:
    #         heading = atan2(dy, dx)

    #         self.state[self.YAW] = heading
    #         self.state[self.X] = x_gps
    #         self.state[self.Y] = y_gps

    #         self.is_yaw_initialized = True
    #         return True

    #     return False

    def check_alignment(self, x_gps, y_gps, current_time):
        if self.start_gps_x is None:
            self.start_gps_x = x_gps
            self.start_gps_y = y_gps
            self.start_gps_time = current_time
            return False

        dx = x_gps - self.start_gps_x
        dy = y_gps - self.start_gps_y
        dist = sqrt(dx * dx + dy * dy)

        # Wait for 4.0 meters to get a clean Heading
        if dist > self.min_align_distance:
            # 1. Heading (This is still useful)
            heading = atan2(dy, dx)

            # 2. Velocity - FORCE TO ZERO
            # The GPS noise makes calculation here too dangerous (resulted in 5.0 m/s).
            # We will let the update_wheel_speed() loop handle the speed up.
            velocity = 0.0 

            self.state[self.X] = x_gps
            self.state[self.Y] = y_gps
            self.state[self.YAW] = heading
            self.state[self.V] = velocity
            
            # Tighter covariance for yaw, loose for velocity (since we guessed 0)
            self.P = np.diag([0.5, 0.5, 0.1, 5.0, 0.1])

            self.is_yaw_initialized = True
            return True

        return False

    # -------------------------------------------------
    # Convenience GPS update
    # -------------------------------------------------
    def update_gps(self, x_gps, y_gps, R_gps_2x2):
        z = np.zeros(5)
        z[self.X] = x_gps
        z[self.Y] = y_gps

        R = np.zeros((5, 5))

        # This tells the EKF: "The IMU prediction is smoother than these GPS jumps."
        R_gps_2x2 *= 4.0 
        
        R[:2, :2] = R_gps_2x2

        update_vector = [True, True, False, False, False]
        self.correct(z, R, update_vector)

    # -------------------------------------------------
    # Getter
    # -------------------------------------------------
    def get_current_state(self):
        return self.state.copy()
