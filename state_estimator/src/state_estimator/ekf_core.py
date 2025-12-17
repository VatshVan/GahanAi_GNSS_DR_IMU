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
        self.Q = np.diag([0.05, 0.05, 0.01, 0.5, 0.05])

        # -------- Yaw bootstrap --------
        self.is_yaw_initialized = False
        self.start_gps_x = None
        self.start_gps_y = None
        self.min_align_distance = 1.5  # meters

    # -------------------------------------------------
    # Prediction (robot_localization-style EKF predict)
    # -------------------------------------------------
    def predict(self, accel_fwd, yaw_rate_meas, dt):
        x, y, yaw, v, yaw_rate = self.state

        # ----- State prediction f(x) -----
        yaw_new = normalize_angle(yaw + yaw_rate_meas * dt)

        v_new = v + accel_fwd * dt
        v_new *= 0.99  # drag
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
    def check_alignment(self, x_gps, y_gps):
        if self.start_gps_x is None:
            self.start_gps_x = x_gps
            self.start_gps_y = y_gps
            return False

        dx = x_gps - self.start_gps_x
        dy = y_gps - self.start_gps_y
        dist = sqrt(dx * dx + dy * dy)

        if dist > self.min_align_distance:
            heading = atan2(dy, dx)

            self.state[self.YAW] = heading
            self.state[self.X] = x_gps
            self.state[self.Y] = y_gps

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
        R[:2, :2] = R_gps_2x2

        update_vector = [True, True, False, False, False]
        self.correct(z, R, update_vector)

    # -------------------------------------------------
    # Getter
    # -------------------------------------------------
    def get_current_state(self):
        return self.state.copy()







# class EKF:
#     def __init__(self,
#                  x0=None,
#                  P0=None,
#                  Q=None,
#                  R_gps=None):
        
#         # --- 1. STATE INITIALIZATION ---
#         # State Vector: [pos_x, pos_y, yaw, velocity]
#         if x0 is None:
#             self.x = np.zeros(4, dtype=float)
#         else:
#             self.x = x0.astype(float)

#         # --- 2. COVARIANCE INITIALIZATION ---
#         # Initial uncertainty. We are unsure about position (1.0), but sure about velocity (1.0).
#         if P0 is None:
#             self.P = np.diag([1.0, 1.0, 0.1, 1.0])
#         else:
#             self.P = P0.astype(float)

#         self.accel_history = deque(maxlen=20) 
#         self.gyro_history = deque(maxlen=20)

#         # --- 3. PROCESS NOISE (Q) ---
#         # How much "randomness" enters the system every step?
#         # If the robot drifts a lot in dead reckoning, INCREASE these values.
#         if Q is None:
#             self.Q = np.diag([
#                 0.05,  # px variance (growth per step)
#                 0.05,  # py variance
#                 0.01,  # yaw variance (gyro noise integration)
#                 0.1    # vel variance (accel noise integration)
#             ])
#         else:
#             self.Q = Q.astype(float)

#         # --- 4. MEASUREMENT NOISE (R) ---
#         # How much do we trust the GPS?
#         # High value = Trust GPS less (Smooths jumpy data).
#         # Low value = Trust GPS more (Snaps to points).
#         if R_gps is None:
#             self.R_gps = np.diag([2.0, 2.0]) # 2.0 meters variance (~1.4m standard deviation)
#         else:
#             self.R_gps = R_gps.astype(float)

#     def predict(self, omega, accel, dt):
#         """
#         PREDICT STEP (Motion Model)
#         Handles the "Dead Reckoning" phase.
        
#         Inputs:
#             omega: Yaw Rate (rad/s) from Gyro
#             accel: Forward Acceleration (m/s^2) from IMU (Corrected for gravity/friction)
#             dt:    Time step (seconds)
#         """
#         px, py, yaw, v = self.x

#         # --- A. STATE PREDICTION ---
#         # Model:
#         # px' = px + v * cos(yaw) * dt
#         # py' = py + v * sin(yaw) * dt
#         # yaw' = yaw + omega * dt
#         # v' = v + accel * dt   <-- THIS IS KEY FOR IMU-ONLY DRIVING
        
#         new_px = px + v * dt * np.cos(yaw)
#         new_py = py + v * dt * np.sin(yaw)
#         new_yaw = yaw + omega * dt
#         new_v = v + accel * dt

#         # Normalize yaw to [-pi, pi]
#         new_yaw = (new_yaw + np.pi) % (2 * np.pi) - np.pi

#         self.x = np.array([new_px, new_py, new_yaw, new_v])

#         # --- B. COVARIANCE PREDICTION (Jacobian F) ---
#         # We calculate how uncertainty spreads.
#         F = np.eye(4)
        
#         # Derivatives for Position
#         F[0, 2] = -v * dt * np.sin(yaw) # d(px)/d(yaw)
#         F[0, 3] = dt * np.cos(yaw)      # d(px)/d(v)
#         F[1, 2] = v * dt * np.cos(yaw)  # d(py)/d(yaw)
#         F[1, 3] = dt * np.sin(yaw)      # d(py)/d(v)
        
#         # Derivatives for Velocity
#         # d(v)/d(v) = 1 (Already in Identity matrix)

#         # Predict P = F * P * F_transpose + Q
#         self.P = F @ self.P @ F.T + self.Q

#     def update_gps(self, z, R=None):
#         """
#         UPDATE STEP (Correction)
#         Call this ONLY when a valid GPS measurement is received.
        
#         Inputs:
#             z: [x_gps, y_gps] (measured position)
#             R: Custom measurement noise (optional)
#         """
#         if R is None:
#             R = self.R_gps

#         # --- A. MEASUREMENT MODEL (H) ---
#         # We observe px and py directly.
#         H = np.zeros((2, 4))
#         H[0, 0] = 1 # Measure px
#         H[1, 1] = 1 # Measure py

#         z = np.asarray(z).reshape(2)
        
#         # --- B. KALMAN FILTER MATH ---
        
#         # 1. Innovation (Error): z_meas - z_pred
#         y = z - H @ self.x 

#         # 2. Innovation Covariance (S)
#         S = H @ self.P @ H.T + R
        
#         # 3. Kalman Gain (K)
#         # K = P * H_T * inv(S)
#         K = self.P @ H.T @ np.linalg.inv(S)

#         # 4. Update State
#         self.x = self.x + K @ y
        
#         # Normalize yaw after update
#         self.x[2] = (self.x[2] + np.pi) % (2 * np.pi) - np.pi

#         # 5. Update Covariance
#         I = np.eye(4)
#         self.P = (I - K @ H) @ self.P

#     def get_state(self):
#         return self.x.copy()

#     def get_cov(self):
#         return self.P.copy()
    

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




# # --- MATH HELPER CLASS ---
# class EKF:
#     def __init__(self):
#         # State Vector: [pos_x, pos_y, yaw, velocity, yaw_rate]
#         self.state = np.zeros(5)
        
#         # COVARIANCE
#         self.P = np.diag([1.0, 1.0, 0.1, 1.0, 0.1])
#         self.Q = np.diag([0.05, 0.05, 0.01, 0.5, 0.05])

#         # --- INITIALIZATION STATE ---
#         self.is_yaw_initialized = False
#         self.start_gps_x = None
#         self.start_gps_y = None
#         self.min_align_distance = 1.5  # Meters to drive before aligning

#     def predict_state(self, accel_fwd, yaw_rate, dt):
#         x, y, yaw, v, omega = self.state
        
#         # Physics Prediction
#         new_yaw = yaw + yaw_rate * dt
#         new_v = v + accel_fwd * dt
#         new_v *= 0.99 # Drag
#         new_v = max(min(new_v, 5.0), -5.0)

#         new_x = x + new_v * np.cos(yaw) * dt
#         new_y = y + new_v * np.sin(yaw) * dt
        
#         self.state = np.array([new_x, new_y, new_yaw, new_v, yaw_rate])

#         # Covariance Prediction
#         F = np.eye(5)
#         F[0, 2] = -v * np.sin(yaw) * dt
#         F[0, 3] = np.cos(yaw) * dt
#         F[1, 2] = v * np.cos(yaw) * dt
#         F[1, 3] = np.sin(yaw) * dt
#         F[2, 4] = dt

#         self.P = F @ self.P @ F.T + (self.Q * dt)

#     def check_alignment(self, x_gps, y_gps):
#         """
#         Calculates initial heading based on the vector of motion.
#         Returns True if alignment was successful.
#         """
#         if self.start_gps_x is None:
#             self.start_gps_x = x_gps
#             self.start_gps_y = y_gps
#             return False

#         dx = x_gps - self.start_gps_x
#         dy = y_gps - self.start_gps_y
#         dist = sqrt(dx*dx + dy*dy)

#         # Wait until we have moved enough to get a clean vector
#         if dist > self.min_align_distance:
#             # Calculate the true heading from GPS motion
#             true_heading = atan2(dy, dx)
            
#             # Update the State's Yaw immediately
#             self.state[2] = true_heading
            
#             # Reset Position to match GPS exactly (snap to grid)
#             self.state[0] = x_gps
#             self.state[1] = y_gps
            
#             self.is_yaw_initialized = True
#             return True
        
#         return False

#     def update_gps(self, z, R):
#         # 1. Measurement Model (H)
#         H = np.array([
#             [1, 0, 0, 0, 0],
#             [0, 1, 0, 0, 0]
#         ])

#         # 2. Innovation
#         y = z - H @ self.state
#         S = H @ self.P @ H.T + R
        
#         try:
#             K = self.P @ H.T @ np.linalg.inv(S)
#         except np.linalg.LinAlgError:
#             return

#         # 3. Update
#         self.state = self.state + K @ y
#         self.P = (np.eye(5) - K @ H) @ self.P
    
#     def get_current_state(self): return self.state

