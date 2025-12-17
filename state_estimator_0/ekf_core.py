"""
ekf_core.py

Extended Kalman Filter for a ground vehicle with state:
    x = [px, py, yaw, v]^T

Predict step uses IMU yaw-rate (omega) as control input (no control acceleration).
Measurements supported:
    - GPS: [px, py]
    - Wheel speed: v

Usage:
    from ekf_core import EKF
    ekf = EKF()
    ekf.predict(omega, dt)
    ekf.update_gps(np.array([x_meas, y_meas]), R=...)
    ekf.update_wheel(v_meas, R=...)
"""

import numpy as np

class EKF:
    def __init__(self,
                 x0=None,
                 P0=None,
                 Q=None,
                 R_gps=None,
                 R_wheel=None):
        # State: [px, py, yaw, v]
        if x0 is None:
            self.x = np.zeros(4, dtype=float)
        else:
            self.x = x0.astype(float)

        if P0 is None:
            self.P = np.diag([1.0, 1.0, 0.5**2, 1.0])  # initial covariance
        else:
            self.P = P0.astype(float)

        # Process noise covariance
        if Q is None:
            # small uncertainty in position process (due to modeling), yaw, and v
            self.Q = np.diag([0.01, 0.01, (np.deg2rad(1.0))**2, 0.1**2])
        else:
            self.Q = Q.astype(float)

        # Measurement covariances
        if R_gps is None:
            self.R_gps = np.diag([1.5**2, 1.5**2])  # meters^2
        else:
            self.R_gps = R_gps.astype(float)

        if R_wheel is None:
            self.R_wheel = np.array([[0.2**2]])  # (m/s)^2
        else:
            self.R_wheel = np.atleast_2d(R_wheel).astype(float)

    def predict(self, omega, dt):
        """
        EKF predict step using control input omega (yaw rate from IMU).
        Model:
            px' = px + v*dt*cos(yaw)
            py' = py + v*dt*sin(yaw)
            yaw' = yaw + omega*dt
            v' = v
        """
        px, py, yaw, v = self.x

        # Predict state
        px_pred = px + v * dt * np.cos(yaw)
        py_pred = py + v * dt * np.sin(yaw)
        yaw_pred = yaw + omega * dt
        v_pred = v

        # Normalize yaw_pred to [-pi, pi]
        yaw_pred = (yaw_pred + np.pi) % (2 * np.pi) - np.pi

        self.x = np.array([px_pred, py_pred, yaw_pred, v_pred])

        # Jacobian F = df/dx (4x4)
        F = np.eye(4)
        # ∂px/∂yaw = -v*dt*sin(yaw)
        F[0,2] = -v * dt * np.sin(yaw)
        # ∂px/∂v = dt*cos(yaw)
        F[0,3] = dt * np.cos(yaw)

        # ∂py/∂yaw = v*dt*cos(yaw)
        F[1,2] = v * dt * np.cos(yaw)
        # ∂py/∂v = dt*sin(yaw)
        F[1,3] = dt * np.sin(yaw)

        # ∂yaw/∂yaw = 1 (already)
        # ∂v/∂v = 1

        # Covariance predict
        self.P = F @ self.P @ F.T + self.Q

    def update_gps(self, z, R=None):
        """
        GPS update with measurement z = [px_meas, py_meas].
        Linear measurement: H = [[1,0,0,0],[0,1,0,0]]
        """
        if R is None:
            R = self.R_gps

        H = np.zeros((2,4))
        H[0,0] = 1.0
        H[1,1] = 1.0

        z = np.asarray(z).reshape(2)

        y = z - H @ self.x  # innovation

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        # Ensure yaw stays normalized
        self.x[2] = (self.x[2] + np.pi) % (2 * np.pi) - np.pi

        I = np.eye(4)
        self.P = (I - K @ H) @ self.P

    def update_wheel(self, v_meas, R=None):
        """
        Wheel speed update with scalar measurement v_meas.
        H = [0,0,0,1]
        """
        if R is None:
            R = self.R_wheel

        H = np.array([[0.0, 0.0, 0.0, 1.0]])
        z = np.atleast_1d(v_meas).astype(float)

        y = z - (H @ self.x).reshape(-1)

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + (K @ y).flatten()
        # normalize yaw
        self.x[2] = (self.x[2] + np.pi) % (2 * np.pi) - np.pi

        I = np.eye(4)
        self.P = (I - K @ H) @ self.P

    def get_state(self):
        return self.x.copy()

    def get_cov(self):
        return self.P.copy()
