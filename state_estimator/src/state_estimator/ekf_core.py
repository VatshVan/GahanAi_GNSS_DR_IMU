"""
Extended Kalman Filter (EKF) for 2D vehicle pose and velocity estimation.
Module summary
---------------
This module implements a compact, practical Extended Kalman Filter (EKF) tailored
for real-world 2D vehicle dead-reckoning using forward acceleration, yaw-rate
(gyro), wheel speed and GPS position/heading measurements. The filter maintains
a 5-element state vector:
    [x, y, yaw, v, yaw_rate]^T
Key design goals:
- Robust, minimal, and easily auditable for long-term maintenance (decades).
- Explicit and conservative numerical practices to avoid silent failures.
- Clear separation between prediction (motion model) and correction (measurement
  update) steps, enabling straightforward sensor fusion and unit testing.
- Practical defaults and configurable tuning matrices to adapt to different
  platforms and sensor suites.
State representation
--------------------
State vector (state, units):
- x:   X position in global coordinates (meters).
- y:   Y position in global coordinates (meters).
- yaw: Heading (radians), normalized to (-pi, pi].
- v:   Forward speed in vehicle-forward direction (meters/second).
- yaw_rate: Angular rate about the vertical axis (radians/second).
The filter stores a 5x5 covariance matrix P describing the uncertainty of the
state estimate and a process noise covariance Q describing expected process
disturbances. Measurement noise covariance matrices R are provided per-measurement
during correction calls.
Coordinate frames and units
---------------------------
- All translational quantities are in meters (m) and meters/second (m/s).
- Angular quantities are in radians (rad) and radians/second (rad/s).
- Time increments dt are in seconds (s).
- GPS positions are expected in the same global coordinate frame as the EKF
  internal state (e.g., projected map coordinates such as UTM or local ENU).
  Converting latitude/longitude to metric coordinates is the caller's
  responsibility before passing to update_gps(...) or check_alignment(...).
Numerical and algorithmic details
---------------------------------
Prediction (motion model)
- Continuous-time vehicle motion is discretized using a first-order Euler
  integration of velocity and yaw-rate inputs:
    x_{k+1} = x_k + v_k * cos(yaw_k) * dt
    y_{k+1} = y_k + v_k * sin(yaw_k) * dt
    yaw_{k+1} = wrap(yaw_k + yaw_rate_meas * dt)
    v_{k+1} = clip(v_k + accel_fwd * dt, v_min, v_max)
    yaw_rate_{k+1} = yaw_rate_meas
- The filter uses a state transition Jacobian F derived from the above
  equations for covariance propagation:
    P_{k+1} = F P_k F^T + Q * dt
  The Q matrix provided by the user (or defaults) should represent
  continuous-time spectral densities; multiplying by dt is used here
  to obtain an approximate discrete-time process covariance.
- Velocity is clamped within a conservative [-20, 20] m/s bound to prevent
  instability from unrealistically large predicted speeds.
Correction (measurement model)
- The module supports partial updates through a generic correct(z, R, update_vector)
  interface:
    - z: full-length (5) measurement vector where unused entries can be zero.
    - R: full 5x5 measurement covariance matrix where only the sub-block(s)
         corresponding to active measurements need valid variances.
    - update_vector: boolean-like iterable length 5 indicating which state
                     indices are being corrected.
- The measurement model is linear and assumed to be direct observation of a
  subset of the state variables (H matrix contains 1.0 at observed indices).
- Yaw (heading) measurements and updates are wrapped using a robust
  normalize_angle(...) helper to ensure angular residuals remain in (-pi, pi].
- The code computes the Kalman gain via:
    K = P H^T (H P H^T + R_sub)^-1
  and updates state and covariance using standard linear Kalman update
  formulas. If S is singular or inversion fails, the correction is safely
  skipped to avoid corrupting the filter state.
Sensor-specific helper methods
- check_alignment(x_gps, y_gps, current_time):
    - A simple initialization alignment routine used to set the filter state
      when the platform has moved at least `min_align_distance` meters from the
      first received GPS fix. This prevents spurious initial headings from
      single-point GPS measurements.
    - Once triggered, state[0:2] are set to the GPS position, state[2] (yaw) is
      set to the heading from the initial displacement vector, velocity is set
      to zero, and is_yaw_initialized is set True.
    - Note: This is a one-shot alignment helper and does not replace a proper
      observability-based initialization.
- update_gps(x, y, R_mat):
    - Convenience wrapper to correct the x and y states using the provided
      5x5 measurement covariance matrix R_mat (only indices 0 and 1 are used).
- update_gps_heading(yaw, var):
    - Convenience wrapper to correct the yaw (index 2) with variance `var`.
    - Properly wraps angular residuals before update.
- update_wheel_speed(speed, var):
    - Convenience wrapper to correct the v (index 3) state using wheel-speed
      measurement with scalar variance `var`.
normalize_angle(a)
- Utility to map any real-valued angle `a` to (-pi, pi]. The function is used
  wherever angular differences are computed to ensure consistency and avoid
  discontinuities.
Defaults and tuning guidance
----------------------------
- Default P and Q matrices are conservative starting points intended for small
  wheeled vehicles. They should be adapted to the platform and sensors:
    - Increase diagonal Q entries for state elements expected to vary more
      between update cycles (e.g., large unmodeled accelerations).
    - Increase P diagonals to reflect greater initial uncertainty.
    - R matrices passed to correction methods must reflect sensor noise
      characteristics (variance), including any correlations if present.
- dt should be kept reasonably small (e.g., <= 0.1s) to maintain numerical
  accuracy for the simple Euler integration used here. If using larger dt,
  consider switching to a higher-fidelity integration (e.g., Runge-Kutta)
  and applying discrete-time process noise correctly.
Robustness, safety, and edge-cases
---------------------------------
- The class is not designed to be thread-safe. External synchronization is
  required if multiple threads may call predict/correct concurrently.
- The correction step tolerantly handles singular innovation covariances by
  skipping updates if the matrix inversion fails. This prevents raising
  exceptions in production, but it is recommended to log or monitor such
  occurrences during development and field testing.
- GPS inputs must be provided in consistent coordinates. Feeding lat/lon
  directly without projection will produce meaningless results.
- When yaw is not initialized, using gyro-only predictions will still update
  states; however yaw-dependent position predictions may be poor until the
  yaw is aligned/initialized.
Versioning, compatibility and maintainability hints
--------------------------------------------------
- This code is intentionally minimal and well-commented: maintainers should
  keep the following constraints in mind when evolving the module:
    - Preserve the 5-state structure or provide a clear migration plan and
      deprecation pathway if adding or removing state elements.
    - Keep the interface for predict(...) and correct(...) stable or provide
      wrappers to avoid breaking callers (e.g., a new signature should retain
      backward-compatible defaults).
    - Any performance optimizations (e.g., in-place numpy operations) should be
      covered by unit tests that assert numerical equality to a specified
      tolerance across the supported platform set.
- Recommended unit and integration tests:
    - Numerical consistency test: confirm P remains symmetric and positive
      semi-definite (to numerical tolerance) after sequences of predict/correct.
    - Angle wrap correctness: simulate yaw near -pi/pi and verify residuals and
      state remain continuous.
    - Alignment test: feed two GPS points separated by >min_align_distance and
      verify yaw initialization logic and resulting state.
    - Sensor fusion scenarios: joint GPS + wheel speed + gyro sequences to
      validate convergence and expected steady-state covariance.
Best practices for long-term operation (decades)
-----------------------------------------------
- Store sensor and filter configuration (Q, initial P, min_align_distance)
  in version-controlled configuration files, not hard-coded constants, to
  facilitate tuning and reproducibility.
- Log and version telemetry used for tuning and debugging (raw sensor streams,
  timestamps, and filter states) so regressions can be diagnosed years later.
- Add schema/signature versioning to serialized state blobs (if saving P/state)
  to allow safe upgrades and backwards compatibility.
- Keep math derivations, reference publications, and tuning notes near the
  implementation (e.g., in this docstring or adjacent README) to preserve
  rationale across team turnover.
References and background reading
---------------------------------
- Maybeck, P. S., "Stochastic Models, Estimation, and Control," Vol. 1.
- Bar-Shalom, Y., Li, X. R., & Kirubarajan, T., "Estimation with Applications to
  Tracking and Navigation."
- Thrun, S., Burgard, W., & Fox, D., "Probabilistic Robotics" — for practical
  sensor fusion and probabilistic representations.
Example usage (conceptual)
--------------------------
1. Create filter: ekf = EKF()
2. Initialize alignment if starting from GPS: ekf.check_alignment(gps_x0, gps_y0, t0)
3. In your control loop:
    - ekf.predict(accel_fwd, gyro_yaw_rate, dt)
    - ekf.update_wheel_speed(wheel_speed, wheel_var)        # optional
    - ekf.update_gps(gps_x, gps_y, gps_R_matrix)            # when GPS available
    - ekf.update_gps_heading(gps_yaw, gps_yaw_var)          # if available
    - state = ekf.get_current_state()
API reference
-------------
- class EKF:
    - __init__(): Create instance and set conservative defaults for P, Q.
    - predict(accel_fwd: float, yaw_rate_meas: float, dt: float) -> None
        Propagate state and covariance forward using IMU-derived inputs.
    - correct(z: np.ndarray, R: np.ndarray, update_vector: Iterable[bool]) -> None
        Generic partial-state correction. z and R are full-size (5) objects.
    - check_alignment(x_gps: float, y_gps: float, current_time: float) -> bool
        Use displacement from initial GPS to initialize yaw and position. Returns
        True when alignment occurs.
    - update_gps(x: float, y: float, R_mat: np.ndarray) -> None
        Correct x and y using provided measurement covariance matrix.
    - update_gps_heading(yaw: float, var: float) -> None
        Correct yaw with scalar variance `var` (radians^2).
    - update_wheel_speed(speed: float, var: float) -> None
        Correct forward velocity (v) using wheel speed measurement (m/s).
    - get_current_state() -> np.ndarray
        Return the current 5-element state vector.
- function normalize_angle(a: float) -> float
    Normalize angle to the (-pi, pi] interval.
Limitations
-----------
- The process model ignores lateral slip and full vehicle dynamics; for aggressive
  maneuvers or high-slip conditions (e.g., off-road, low-friction surfaces),
  extend the state and process model to include lateral velocity, slip angle,
  or use a higher-fidelity vehicle model.
- The filter treats the yaw-rate measurement as the current yaw_rate state;
  if a separate internal bias model or gyro scale error modeling is required,
  add additional states (e.g., gyro bias) and extend the Jacobians accordingly.
Contact and maintainers
-----------------------
- Maintain a clear CODEOWNERS and contact information adjacent to this module
  in the repository for long-term stewardship and updates.

"""
import numpy as np
from math import sin, cos, atan2, sqrt, pi

def normalize_angle(a):
    return (a + pi) % (2 * pi) - pi

class EKF:
    X, Y, YAW, V, YAW_RATE = 0, 1, 2, 3, 4

    def __init__(self):
        self.state = np.zeros(5)
        
        # Initial Uncertainty
        self.P = np.diag([1.0, 1.0, 0.5, 1.0, 0.1])

        # --- TUNING FOR REAL LIFE ---
        self.Q = np.diag([
            0.05,   # X: Flexible
            0.05,   # Y: Flexible
            0.001,  # Yaw: Trust Gyro heavily (Stiff)
            0.1,    # Velocity: Allow changes from Wheel Odom
            0.02    # Yaw Rate: Slow gyro bias drift
        ])

        # Alignment
        self.is_yaw_initialized = False
        self.start_gps_x = None
        self.start_gps_y = None
        self.start_gps_time = None
        self.min_align_distance = 5.0
        
        self.last_gps_x = None
        self.last_gps_y = None
        self.last_gps_time = None

    def predict(self, accel_fwd, yaw_rate_meas, dt):
        x, y, yaw, v, _ = self.state

        yaw_new = normalize_angle(yaw + yaw_rate_meas * dt)
        v_new = v + accel_fwd * dt
        v_new = np.clip(v_new, -20.0, 20.0)

        x_new = x + v * cos(yaw) * dt
        y_new = y + v * sin(yaw) * dt

        self.state = np.array([x_new, y_new, yaw_new, v_new, yaw_rate_meas])

        F = np.eye(5)
        F[0, 2] = -v * sin(yaw) * dt
        F[0, 3] = cos(yaw) * dt
        F[1, 2] = v * cos(yaw) * dt
        F[1, 3] = sin(yaw) * dt
        F[4, 4] = 0 

        self.P = F @ self.P @ F.T + self.Q * dt

    def correct(self, z, R, update_vector):
        idxs = [i for i, val in enumerate(update_vector) if val]
        if not idxs: return

        z_sub = z[idxs]
        x_sub = self.state[idxs]
        
        H = np.zeros((len(idxs), 5))
        for i, original_idx in enumerate(idxs):
            H[i, original_idx] = 1.0
            
        y = z_sub - x_sub
        if 2 in idxs: # Yaw Wrap
            idx_in_y = idxs.index(2)
            y[idx_in_y] = normalize_angle(y[idx_in_y])

        S = H @ self.P @ H.T + R[np.ix_(idxs, idxs)]
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except: return

        self.state += K @ y
        self.state[2] = normalize_angle(self.state[2])
        self.P = (np.eye(5) - K @ H) @ self.P

    def check_alignment(self, x_gps, y_gps, current_time):
        if self.start_gps_x is None:
            self.start_gps_x = x_gps
            self.start_gps_y = y_gps
            self.start_gps_time = current_time
            return False

        dx = x_gps - self.start_gps_x
        dy = y_gps - self.start_gps_y
        dist = sqrt(dx*dx + dy*dy)

        if dist > self.min_align_distance:
            heading = atan2(dy, dx)
            # Init state
            self.state[0] = x_gps
            self.state[1] = y_gps
            self.state[2] = heading
            self.state[3] = 0.0 
            self.is_yaw_initialized = True
            return True
        return False

    def update_gps(self, x, y, R_mat):
        z = np.zeros(5); z[0] = x; z[1] = y
        self.correct(z, R_mat, [True, True, False, False, False])

    def update_gps_heading(self, yaw, var):
        z = np.zeros(5); z[2] = yaw
        R = np.zeros((5,5)); R[2,2] = var
        self.correct(z, R, [False, False, True, False, False])

    def update_wheel_speed(self, speed, var):
        z = np.zeros(5); z[3] = speed
        R = np.zeros((5,5)); R[3,3] = var
        # Update Velocity (Index 3)
        self.correct(z, R, [False, False, False, True, False])
        
    def get_current_state(self): return self.state




# """
# ekf_core.py

# Extended Kalman Filter for a ground vehicle with state:
#     x = [px, py, yaw, v]^T

# Core Logic:
#     - PREDICT: Uses IMU (Yaw Rate + Forward Accel) to propagate state. 
#                This handles "Dead Reckoning" when GPS is lost.
#     - UPDATE:  Fuses GPS [x, y] to correct position drift.
# """
# import numpy as np
# from collections import deque
# from math import sin, cos, atan2, sqrt, pi

# # ------------------ Utilities ------------------

# def normalize_angle(a):
#     return (a + pi) % (2 * pi) - pi


# # ------------------ EKF ------------------

# class EKF:
#     """
#     State vector:
#         x = [pos_x, pos_y, yaw, velocity, yaw_rate]
#     Covariance matrix:
#         P = 5x5 covariance matrix
#     Process noise:
#         Q = 5x5 process noise matrix
#     Methods:
#         - predict(accel_fwd, yaw_rate_meas, dt)
#         - correct(z, R, update_vector)
#         - check_alignment(x_gps, y_gps)
#         - update_gps(x_gps, y_gps, R_gps_2x2)
#         - get_current_state()
#     """

#     X = 0
#     Y = 1
#     YAW = 2
#     V = 3
#     YAW_RATE = 4

#     def __init__(self):
#         # -------- State --------
#         self.state = np.zeros(5)

#         # -------- Covariances --------
#         self.P = np.diag([1.0, 1.0, 0.1, 1.0, 0.1])

#         # -------- Process Noise (Q) - UPDATED FOR REAL LIFE --------
#         # Low values = High Inertia (Smooth). High values = Twitchy.

#         # -------- Process Noise (Q) - FINE TUNED --------
#         self.Q = np.diag([
#             0.001,  # X: Very low (Smooth path)
#             0.001,  # Y: Very low
#             0.001,  # Yaw: No instant snap-turns
#             0.005,  # Velocity: VERY LOW (High Inertia/Mass) <--- CRITICAL CHANGE
#             0.01    # Yaw Rate: Gyro bias drifts slowly
#         ])

#         # -------- Yaw bootstrap --------
#         self.is_yaw_initialized = False
#         self.start_gps_x = None
#         self.start_gps_y = None
#         self.min_align_distance = 4.0  # meters
#         self.last_gps_x = None
#         self.last_gps_y = None
#         self.last_gps_time = None
#         self.min_gps_heading_speed = 1.5  # m/s

#     def update_gps_heading(self, yaw_gps, var_yaw):
#         z = np.zeros(5)
#         z[self.YAW] = yaw_gps

#         R = np.zeros((5, 5))
#         R[self.YAW, self.YAW] = var_yaw

#         update_vector = [False, False, True, False, False]
#         self.correct(z, R, update_vector)
    
#     def update_wheel_speed(self, v_meas, var_v):
#         """
#         Wheel odometry update: measures forward velocity only
#         """
#         z = np.zeros(5)
#         z[self.V] = v_meas

#         R = np.zeros((5, 5))
#         R[self.V, self.V] = var_v

#         update_vector = [False, False, False, True, False]
#         self.correct(z, R, update_vector)

#     # -------------------------------------------------
#     # Prediction
#     # -------------------------------------------------
#     def predict(self, accel_fwd, yaw_rate_meas, dt):
#         x, y, yaw, v, yaw_rate = self.state

#         # ----- State prediction f(x) -----
#         yaw_new = normalize_angle(yaw + yaw_rate_meas * dt)

#         v_new = v + accel_fwd * dt
#         # v_new *= 0.99  # drag
#         v_new = np.clip(v_new, -5.0, 5.0)

#         x_new = x + v * cos(yaw) * dt
#         y_new = y + v * sin(yaw) * dt

#         self.state = np.array([
#             x_new,
#             y_new,
#             yaw_new,
#             v_new,
#             yaw_rate_meas
#         ])

#         # ----- Jacobian F = ∂f/∂x -----
#         F = np.eye(5)
#         F[self.X, self.YAW] = -v * sin(yaw) * dt
#         F[self.X, self.V]   =  cos(yaw) * dt
#         F[self.Y, self.YAW] =  v * cos(yaw) * dt
#         F[self.Y, self.V]   =  sin(yaw) * dt
#         F[self.YAW, self.YAW_RATE] = dt

#         # ----- Covariance prediction -----
#         self.P = F @ self.P @ F.T + self.Q * dt

#     # -------------------------------------------------
#     # Generic EKF correction (robot_localization::correct)
#     # -------------------------------------------------
#     def correct(self, z, R, update_vector):
#         """
#         z : full measurement vector (size 5)
#         R : full measurement covariance (5x5)
#         update_vector : list[bool] of size 5
#         """

#         # ----- Select valid update indices -----
#         update_indices = []
#         for i, flag in enumerate(update_vector):
#             if flag and np.isfinite(z[i]):
#                 update_indices.append(i)

#         if not update_indices:
#             return

#         m = len(update_indices)

#         # ----- Build sub-vectors -----
#         x_sub = self.state[update_indices]
#         z_sub = z[update_indices]

#         R_sub = R[np.ix_(update_indices, update_indices)]

#         # Handle bad covariances (exact ROS behavior)
#         for i in range(m):
#             if R_sub[i, i] < 0:
#                 R_sub[i, i] = abs(R_sub[i, i])
#             if R_sub[i, i] < 1e-9:
#                 R_sub[i, i] = 1e-9

#         # ----- Measurement matrix H -----
#         H = np.zeros((m, 5))
#         for row, idx in enumerate(update_indices):
#             H[row, idx] = 1.0

#         # ----- Innovation -----
#         innovation = z_sub - x_sub

#         # Angle wrapping on innovation
#         for i, idx in enumerate(update_indices):
#             if idx == self.YAW:
#                 innovation[i] = normalize_angle(innovation[i])

#         # ----- Kalman gain -----
#         S = H @ self.P @ H.T + R_sub
#         try:
#             S_inv = np.linalg.inv(S)
#         except np.linalg.LinAlgError:
#             return

#         K = self.P @ H.T @ S_inv

#         # ----- State update -----
#         self.state += K @ innovation
#         self.state[self.YAW] = normalize_angle(self.state[self.YAW])

#         # ----- Joseph-form covariance update -----
#         I = np.eye(5)
#         KH = K @ H
#         self.P = (I - KH) @ self.P @ (I - KH).T + K @ R_sub @ K.T

#     # -------------------------------------------------
#     # GPS yaw bootstrap (same logic you already used)
#     # -------------------------------------------------
#     # def check_alignment(self, x_gps, y_gps):
#     #     if self.start_gps_x is None:
#     #         self.start_gps_x = x_gps
#     #         self.start_gps_y = y_gps
#     #         return False

#     #     dx = x_gps - self.start_gps_x
#     #     dy = y_gps - self.start_gps_y
#     #     dist = sqrt(dx * dx + dy * dy)

#     #     if dist > self.min_align_distance:
#     #         heading = atan2(dy, dx)

#     #         self.state[self.YAW] = heading
#     #         self.state[self.X] = x_gps
#     #         self.state[self.Y] = y_gps

#     #         self.is_yaw_initialized = True
#     #         return True

#     #     return False

#     def check_alignment(self, x_gps, y_gps, current_time):
#         if self.start_gps_x is None:
#             self.start_gps_x = x_gps
#             self.start_gps_y = y_gps
#             self.start_gps_time = current_time
#             return False

#         dx = x_gps - self.start_gps_x
#         dy = y_gps - self.start_gps_y
#         dist = sqrt(dx * dx + dy * dy)

#         # Wait for 4.0 meters to get a clean Heading
#         if dist > self.min_align_distance:
#             # 1. Heading (This is still useful)
#             heading = atan2(dy, dx)

#             # 2. Velocity - FORCE TO ZERO
#             # The GPS noise makes calculation here too dangerous (resulted in 5.0 m/s).
#             # We will let the update_wheel_speed() loop handle the speed up.
#             velocity = 0.0 

#             self.state[self.X] = x_gps
#             self.state[self.Y] = y_gps
#             self.state[self.YAW] = heading
#             self.state[self.V] = velocity
            
#             # Tighter covariance for yaw, loose for velocity (since we guessed 0)
#             self.P = np.diag([0.5, 0.5, 0.1, 5.0, 0.1])

#             self.is_yaw_initialized = True
#             return True

#         return False

#     # -------------------------------------------------
#     # Convenience GPS update
#     # -------------------------------------------------
#     def update_gps(self, x_gps, y_gps, R_gps_2x2):
#         z = np.zeros(5)
#         z[self.X] = x_gps
#         z[self.Y] = y_gps

#         R = np.zeros((5, 5))

#         # This tells the EKF: "The IMU prediction is smoother than these GPS jumps."
#         R_gps_2x2 *= 4.0 
        
#         R[:2, :2] = R_gps_2x2

#         update_vector = [True, True, False, False, False]
#         self.correct(z, R, update_vector)

#     # -------------------------------------------------
#     # Getter
#     # -------------------------------------------------
#     def get_current_state(self):
#         return self.state.copy()








