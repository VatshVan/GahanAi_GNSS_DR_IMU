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
    

# --- MATH CLASS ---
class EKF:
    def __init__(self):
        self.x = np.zeros(5)
        self.P = np.eye(5) * 0.1
        self.Q = np.diag([0.0, 0.0, 0.001, 0.01, 0.01])
        self.accel_history = deque(maxlen=20) 
        self.gyro_history = deque(maxlen=20)

    def predict(self, alpha, omega, dt):
        x, y, yaw, v, _ = self.x
        new_yaw = yaw + omega * dt
        new_x = x + v * np.cos(yaw) * dt
        new_y = y + v * np.sin(yaw) * dt
        self.x = np.array([new_x, new_y, new_yaw, v, omega])
        
        F = np.eye(5)
        F[0, 2] = -v * np.sin(yaw) * dt
        F[0, 3] = np.cos(yaw) * dt
        F[1, 2] = v * np.cos(yaw) * dt
        F[1, 3] = np.sin(yaw) * dt
        self.P = F @ self.P @ F.T + self.Q
    
    def get_state(self): return self.x

    def apply_velocity_constraint(self, sigma=0.2):
        # pseudo-measurement: v ≈ current v
        H = np.zeros((1, 5))
        H[0, 3] = 1.0

        R = np.array([[sigma**2]])

        z = np.array([[self.x[3]]])
        y = z - H @ self.x.reshape(-1,1)

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + (K @ y).flatten()
        self.P = (np.eye(5) - K @ H) @ self.P