import numpy as np
from math import sin, cos, pi

def normalize_angle(a):
    return (a + pi) % (2 * pi) - pi

class EKF:
    def __init__(self):
        # State: [x, y, yaw, v, yaw_rate]
        self.state = np.zeros(5)
        # P: Error covariance
        self.P = np.diag([0.5, 0.5, 0.01, 0.1, 0.01])
        # Q: Process noise
        self.Q = np.diag([0.1, 0.1, 0.001, 0.2, 0.005]) 
        self.is_yaw_initialized = False

    def predict(self, dt, accel=0.0):
        """
        Uses constant acceleration model for state prediction.
        accel: SH2_LINEAR_ACCELERATION from BNO085.
        """
        x, y, yaw, v, yaw_rate = self.state
        
        # State transition
        # x_next = x + v*cos(yaw)*dt + 0.5*a*cos(yaw)*dt^2
        dist = v * dt + 0.5 * accel * dt**2
        self.state[0] = x + dist * cos(yaw)
        self.state[1] = y + dist * sin(yaw)
        self.state[2] = normalize_angle(yaw + yaw_rate * dt)
        self.state[3] = v + accel * dt
        # yaw_rate (index 4) assumed constant between updates

        # Jacobian of F with respect to state
        F = np.eye(5)
        F[0, 2] = -dist * sin(yaw)
        F[0, 3] = cos(yaw) * dt
        F[1, 2] = dist * cos(yaw)
        F[1, 3] = sin(yaw) * dt
        F[2, 4] = dt
        F[3, 3] = 1.0 

        self.P = F @ self.P @ F.T + self.Q * dt

    def correct(self, z, R, mask):
        """Universal correction step using a sensor mask."""
        idxs = [i for i, val in enumerate(mask) if val]
        if not idxs: return

        H = np.zeros((len(idxs), 5))
        for i, idx in enumerate(idxs):
            H[i, idx] = 1.0
        
        # Innovation
        z_obs = z[idxs]
        z_pred = self.state[idxs]
        y = z_obs - z_pred
        
        # Normalize yaw innovation if present
        if 2 in idxs:
            y_idx = idxs.index(2)
            y[y_idx] = normalize_angle(y[y_idx])

        S = H @ self.P @ H.T + R[np.ix_(idxs, idxs)]
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.state += K @ y
        self.state[2] = normalize_angle(self.state[2])
        self.P = (np.eye(5) - K @ H) @ self.P

    def update_velocity(self, v, noise):
        """Allows for future encoder or GPS SOG integration"""
        z, R = np.zeros(5), np.zeros((5,5))
        z[3], R[3,3] = v, noise**2
        self.correct(z, R, [False, False, False, True, False])

    def update_imu_absolute(self, yaw, noise):
        z, R = np.zeros(5), np.zeros((5,5))
        z[2], R[2,2] = yaw, noise**2
        self.correct(z, R, [False, False, True, False, False])

    def update_imu_velocity(self, yaw_rate, noise):
        z, R = np.zeros(5), np.zeros((5,5))
        z[4], R[4,4] = yaw_rate, noise**2
        self.correct(z, R, [False, False, False, False, True])

    def update_gps(self, x, y, noise):
        z, R = np.zeros(5), np.zeros((5,5))
        z[0], z[1], R[0,0], R[1,1] = x, y, noise**2, noise**2
        self.correct(z, R, [True, True, False, False, False])