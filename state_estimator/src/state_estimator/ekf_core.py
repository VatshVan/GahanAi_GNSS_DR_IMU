# """
# Extended Kalman Filter (EKF) Core
# Hybrid Version: Supports GNSS + IMU + Wheel + (Optional) Lidar
# """
# import numpy as np
# from math import sin, cos, atan2, sqrt, pi

# def normalize_angle(a):
#     return (a + pi) % (2 * pi) - pi

# class EKF:
#     def __init__(self):
#         # State: [x, y, yaw, v, yaw_rate]
#         self.state = np.zeros(5)
        
#         # Initial Uncertainty
#         self.P = np.diag([5.0, 5.0, 1.0, 5.0, 1.0])

#         # Process Noise (Tuning)
#         # We assume a balanced model that trusts sensors when available
#         self.Q = np.diag([
#             0.05,   # X
#             0.05,   # Y
#             0.001,  # Yaw (Trust BNO085)
#             0.1,    # Velocity (Flexible to allow Wheel or Lidar updates)
#             0.05    # Yaw Rate
#         ])

#         self.is_yaw_initialized = False
#         self.start_gps_x = None
#         self.start_gps_y = None
        
#     def predict(self, dt):
#         """ Prediction uses internal state only """
#         x, y, yaw, v, yaw_rate = self.state

#         yaw_new = normalize_angle(yaw + yaw_rate * dt)
#         x_new = x + v * cos(yaw) * dt
#         y_new = y + v * sin(yaw) * dt
        
#         # Jacobian F
#         F = np.eye(5)
#         F[0, 2] = -v * sin(yaw) * dt
#         F[0, 3] = cos(yaw) * dt
#         F[1, 2] = v * cos(yaw) * dt
#         F[1, 3] = sin(yaw) * dt
        
#         self.P = F @ self.P @ F.T + self.Q * dt
#         self.state = np.array([x_new, y_new, yaw_new, v, yaw_rate])

#     def correct(self, z, R, update_vector):
#         idxs = [i for i, val in enumerate(update_vector) if val]
#         if not idxs: return

#         z_sub = z[idxs]
#         x_sub = self.state[idxs]
#         H = np.zeros((len(idxs), 5))
#         for i, original_idx in enumerate(idxs):
#             H[i, original_idx] = 1.0
            
#         y = z_sub - x_sub
#         if 2 in idxs: y[idxs.index(2)] = normalize_angle(y[idxs.index(2)])

#         S = H @ self.P @ H.T + R[np.ix_(idxs, idxs)]
#         K = self.P @ H.T @ np.linalg.inv(S)

#         self.state += K @ y
#         self.state[2] = normalize_angle(self.state[2])
#         self.P = (np.eye(5) - K @ H) @ self.P

#     # --- SENSOR INTERFACES ---

#     def update_gps(self, x, y, R_mat):
#         z = np.zeros(5); z[0] = x; z[1] = y
#         self.correct(z, R_mat, [True, True, False, False, False])

#     def update_wheel_speed(self, speed, var):
#         # Moderate Variance (e.g. 0.1)
#         z = np.zeros(5); z[3] = speed
#         R = np.zeros((5,5)); R[3,3] = var
#         self.correct(z, R, [False, False, False, True, False])

#     def update_lidar_twist(self, v_fwd, yaw_rate, R_twist):
#         # Low Variance (e.g. 0.01) - High Trust!
#         z = np.zeros(5); z[3] = v_fwd; z[4] = yaw_rate
#         R = np.zeros((5,5))
#         R[3,3] = R_twist[0,0]; R[4,4] = R_twist[1,1]
#         self.correct(z, R, [False, False, False, True, True])

#     def update_imu_gyro(self, yaw_rate, var):
#         z = np.zeros(5); z[4] = yaw_rate
#         R = np.zeros((5,5)); R[4,4] = var
#         self.correct(z, R, [False, False, False, False, True])

#     # --- SAFETY GATES ---
#     def check_mahalanobis_gate(self, x_meas, y_meas, threshold=3.0):
#         """ Rejects GPS jumps > threshold meters """
#         dx = x_meas - self.state[0]
#         dy = y_meas - self.state[1]
#         dist = sqrt(dx*dx + dy*dy)
#         return dist < threshold

#     def get_current_state(self): return self.state



"""
Extended Kalman Filter (EKF) Core
---------------------------------
Professional 5-DOF State Estimator.
Math engine only. No ROS dependencies.
"""
import numpy as np
from math import sin, cos, sqrt, pi

def normalize_angle(a):
    return (a + pi) % (2 * pi) - pi

class EKF:
    def __init__(self):
        # State: [x, y, yaw, v, yaw_rate]
        self.state = np.zeros(5)
        
        # Matrices (Initialized to Identity, overridden by ROS Node)
        self.P = np.eye(5)
        self.Q = np.eye(5)

        self.is_yaw_initialized = False

    def setup_matrices(self, p_diag, q_diag):
        """Called by ROS Node to set tuning params from YAML."""
        self.P = np.diag(p_diag)
        self.Q = np.diag(q_diag)

    def predict(self, dt):
        """Prediction Step (Motion Model)."""
        x, y, yaw, v, yaw_rate = self.state

        # Physics Model
        yaw_new = normalize_angle(yaw + yaw_rate * dt)
        x_new = x + v * cos(yaw) * dt
        y_new = y + v * sin(yaw) * dt
        
        # Jacobian (F)
        F = np.eye(5)
        F[0, 2] = -v * sin(yaw) * dt
        F[0, 3] = cos(yaw) * dt
        F[1, 2] = v * cos(yaw) * dt
        F[1, 3] = sin(yaw) * dt
        
        # Covariance Update
        self.P = F @ self.P @ F.T + self.Q * dt
        self.state = np.array([x_new, y_new, yaw_new, v, yaw_rate])

    def correct(self, z, R, update_vector):
        """Generic Measurement Update."""
        # Find which indices we are updating (e.g., [4] for IMU)
        idxs = [i for i, val in enumerate(update_vector) if val]
        if not idxs: return

        z_sub = z[idxs]
        x_sub = self.state[idxs]
        
        # Create Measurement Matrix H (Maps full state to measurement)
        H = np.zeros((len(idxs), 5))
        for i, original_idx in enumerate(idxs):
            H[i, original_idx] = 1.0
            
        y = z_sub - x_sub
        if 2 in idxs: y[idxs.index(2)] = normalize_angle(y[idxs.index(2)])

        try:
            # FIX WAS HERE: R needs to be sliced using the same indices
            # So R passed in MUST be 5x5
            S = H @ self.P @ H.T + R[np.ix_(idxs, idxs)]
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return # Matrix singular, skip update

        self.state += K @ y
        self.state[2] = normalize_angle(self.state[2])
        I = np.eye(5)
        self.P = (I - K @ H) @ self.P

    # --- SENSOR INTERFACES (FIXED R MATRIX CREATION) ---

    def update_gps(self, x, y, noise_std):
        z = np.zeros(5); z[0] = x; z[1] = y
        # FIX: Create full 5x5 matrix
        R = np.zeros((5,5))
        R[0,0] = noise_std**2
        R[1,1] = noise_std**2
        self.correct(z, R, [True, True, False, False, False])

    def update_velocity(self, speed, noise_std, is_lidar=False):
        z = np.zeros(5); z[3] = speed
        # FIX: Create full 5x5 matrix
        R = np.zeros((5,5))
        R[3,3] = noise_std**2
        self.correct(z, R, [False, False, False, True, False])

    def update_imu(self, yaw_rate, noise_std):
        z = np.zeros(5); z[4] = yaw_rate
        # FIX: Create full 5x5 matrix so R[4,4] exists
        R = np.zeros((5,5))
        R[4,4] = noise_std**2
        self.correct(z, R, [False, False, False, False, True])

    def check_gate(self, x, y, threshold):
        dx = x - self.state[0]
        dy = y - self.state[1]
        return sqrt(dx*dx + dy*dy) < threshold