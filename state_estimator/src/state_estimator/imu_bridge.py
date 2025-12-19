"""
DiffIMUBridge Module
====================
Module Purpose
---------------
This module implements a ROS 2 node that bridges a dual-IMU serial data stream into standard
ROS2 sensor messages. It reads a single CSV line from a serial device containing synchronized
measurements from two IMUs, performs calibration and bias removal, fuses the two IMU streams by
simple averaging, applies a complementary filter to estimate roll and pitch, and publishes:
- A sensor_msgs.msg.Imu message on /imu/data_raw (primary output for EKF or other consumers).
- Float32 messages on /imu/pitch and /imu/roll (filtered tilt angles for debugging/visualization).
The node is designed for robust realtime operation in robotic systems and contains production-oriented
features: configurable serial connection, QoS tuned for sensor data, numeric covariance defaults, and
an initialization calibration phase.
Intended Audience
-----------------
- ROS 2 integrators who need to expose a dual-IMU device over serial as a single ROS sensor.
- Systems engineers tuning inertial sensors for state estimation (EKF/UKF) consumption.
- Developers extending or integrating IMU data preprocessing (filtering, bias estimation).
Key Characteristics
-------------------
- Runtime: ROS 2 Python (rclpy).
- Dependencies: rclpy, numpy, pyserial (serial), sensor_msgs, std_msgs, geometry_msgs.
- Real-time loop driven by a ROS 2 timer (default 100 Hz scheduling).
- Sensor QoS configured to BEST_EFFORT, VOLATILE, KEEP_LAST(depth=10) to match typical onboard EKFs.
- Calibration collected automatically on startup by gathering a buffer of raw samples.
- Bias correction applied per-axis and per-sensor; Z-axis accelerometer bias is zeroed after calibration
    (gravity handling assumption).
- Gyroscope data expected in degrees-per-second (°/s); converted to radians-per-second (rad/s) before
    integration and publication.
- Accelerometer data expected in linear acceleration units (m/s^2). No unit conversion is performed.
- Complementary filter parameter (alpha=0.98) fuses integrated gyro rates and accelerometer tilt.
- Orientation output is computed from roll/pitch estimates (yaw is set to zero) and published as a
    geometry_msgs.msg.Quaternion inside the Imu message.
Serial Input Format (required)
------------------------------
A single ASCII line per sample, newline-terminated, with exactly 12 comma-separated numeric fields:
        accel_left_x, accel_left_y, accel_left_z,
        gyro_left_x,  gyro_left_y,  gyro_left_z,
        accel_right_x, accel_right_y, accel_right_z,
        gyro_right_x, gyro_right_y, gyro_right_z
- All fields must be convertible to float.
- Ordering is critical: left sensor data precedes left gyro, then right accel and right gyro.
- Example: "0.01, -9.78, 0.03, 0.12, 0.01, -0.02, 0.00, -9.80, 0.05, 0.11, 0.02, -0.01"
Node Behavior and Processing Pipeline
------------------------------------
1. Serial read:
     - Non-blocking readline with a short timeout; malformed or empty lines are ignored.
2. Parsing:
     - Splits by comma; lines with != 12 fields are discarded (resilient to noise).
3. Calibration (startup only):
     - Raw samples are accumulated in calibration_buffer.
     - Once buffer size exceeds 300 samples, per-axis mean biases are computed for each IMU:
             bias_accel_left  = mean(accel_left_samples)
             bias_gyro_left   = mean(gyro_left_samples)
             bias_accel_right = mean(accel_right_samples)
             bias_gyro_right  = mean(gyro_right_samples)
     - After computing accelerometer biases, the Z-axis bias is set to 0.0 to avoid cancelling gravity
         (assumes node is stationary and level during calibration).
     - The node then sets is_calibrated = True and begins normal operation.
4. Bias removal:
     - Each new sample has its per-sensor bias subtracted.
5. Sensor fusion (dual-IMU averaging):
     - Accelerometer and gyro vectors are averaged element-wise across left and right sensors.
     - This simple averaging reduces uncorrelated noise between sensors.
6. Complementary filter (tilt estimation):
     - Accelerometer-based tilt (pitch_accel, roll_accel) computed with atan2 to avoid singularities.
     - Gyro rates are converted from degrees/s to radians/s, integrated by dt and combined with alpha.
     - alpha = 0.98 by default (high gyro trust, low accelerometer trust).
7. Message publication:
     - sensor_msgs/Imu:
             header.frame_id = "imu_link"
             header.stamp = ROS2 time at publish
             linear_acceleration = averaged accelerometer (units preserved)
             angular_velocity = averaged gyro converted to rad/s (x,y,z)
             orientation = quaternion from (roll_filtered, pitch_filtered, yaw=0)
             covariance arrays set from sensible defaults in the node (tuneable constants)
     - /imu/pitch and /imu/roll: Float32 messages containing filtered angles (radians).
Configuration Parameters
------------------------
- Constructor arguments:
        port: str
                Serial device path (default '/dev/ttyUSB0').
        baud: int
                Serial baud rate (default 115200).
- Hard-coded (editable in-source) tuning constants:
        - Calibration buffer size threshold: 300 samples.
        - Timer period: 0.01 s (100 Hz).
        - Serial timeout: 0.1 s.
        - Complementary filter alpha: 0.98.
        - Covariances:
                GYRO_COV  = 0.0004
Published Topics and Message Details
------------------------------------
- /imu/data_raw (sensor_msgs.msg.Imu)
        - orientation: Quaternion computed from filtered roll and pitch (yaw = 0).
        - angular_velocity: radians/sec (converted from degrees/sec input).
        - linear_acceleration: raw averaged accelerometer values (units preserved).
        - covariance arrays are populated with the node's default constants; tune per sensor.
- /imu/pitch (std_msgs.msg.Float32)
        - data: filtered pitch angle in radians.
- /imu/roll (std_msgs.msg.Float32)
        - data: filtered roll angle in radians.
Coordinate Frame and Axis Conventions
------------------------------------
- The node publishes messages with header.frame_id = "imu_link".
- The mapping from incoming serial axes to ROS axes is assumed to match the rest of the stack.
- The following internal conventions are used for tilt computation:
        pitch_accel = atan2(-ax, sqrt(ay^2 + az^2))
        roll_accel  = atan2(ay, az)
    These equations assume standard right-handed sensor axes. Confirm and adjust if your hardware
    uses a different axis convention.
Operational Assumptions and Limitations
--------------------------------------
- The device must be stationary and level during the calibration phase (first ~300 samples).
- Gyroscope input is expected in degrees per second (°/s). If your IMU outputs rad/s, remove the
    math.radians conversion or adapt the source accordingly.
- Accelerometer input is expected in linear acceleration units (m/s^2); if using g, apply a scale
    factor before publishing.
- Yaw is not estimated — the node sets yaw = 0 in the quaternion. If yaw is required, integrate a
    magnetometer or other heading source and/or use an AHRS filter.
- Simple averaging assumes sensors have similar characteristics and are roughly aligned. For
    strongly misaligned or biased sensors, replace averaging with a more robust fusion scheme.
- The complementary filter's alpha is fixed in code; tune for your platform dynamics.
Failure Modes and Diagnostics
-----------------------------
- Serial Port Unavailable: The node logs a Serial Connection Error and will not publish until
    serial_port is re-established. Recommended: use systemd or ros2 launch to ensure proper device
    permissions and ordering.
- Malformed Lines/Noise: Lines that do not contain exactly 12 numeric CSV fields are silently
    ignored. This protects the node from noisy serial traffic but may hide upstream format issues.
- Calibration Not Completed: Node will not publish IMU data until calibration completes. The
    node logs "CALIBRATION COMPLETE." once ready.
- dt <= 0 or irregular timestamps: Samples where computed dt <= 0 are discarded to avoid integration
    artifacts (this also handles clock resets).
Extensibility and Maintenance Recommendations
---------------------------------------------
- Replace averaging with a configurable fusion backend if you plan to support more advanced sensors.
- Expose tuning parameters (serial port, baud, calibration sample count, alpha, covariances, frame_id)
    via ROS 2 parameters so they can be adjusted at runtime and per-deployment without code changes.
- Add a health/status diagnostic topic or lifecycle state if integrating into safety-critical systems.
- Provide unit tests for:
        - Parsing of valid and invalid lines.
        - Calibration bias computation on synthetic data.
        - Complementary filter behavior across known motion profiles.
        - Quaternion correctness for known roll/pitch inputs.
- Consider publishing raw per-IMU topics for offline analysis and advanced filtering.
- Add support for parameterized axis remapping and scale factors to support different IMU vendors.
Security and Safety Notes
-------------------------
- Serial input is untrusted: the code already performs defensive parsing, but do not run this node
    with higher OS privileges than required.
- Ensure time synchronization across your robot (chrony/ntp) if fusion with other sensors requires
    accurate timestamp alignment.
- Tune covariance values in the Imu message to reflect actual sensor noise characteristics to prevent
    EKF / estimator instability.
Example Usage (conceptual)
--------------------------
- Launch as a ROS 2 node in a launch file, ensuring the serial device is present:
        node = DiffIMUBridge(port='/dev/ttyUSB0', baud=115200)
- During system bring-up, keep the device still and level until the node logs:
        "CALIBRATION COMPLETE."
Versioning and Long-Term Maintenance
------------------------------------
- Keep an eye on rclpy and ROS 2 API changes across distros; the node uses rclpy timers and Node APIs
    that are stable across mainstream ROS2 distributions but may require minor updates.
- Maintain a test harness that emulates the device's serial output to validate future changes.
- Document any changes to message frame_id, covariances, or input units in release notes.
Authoring and Provenance
------------------------
- This docstring is intended to serve as the canonical operational and integration documentation for
    the DiffIMUBridge node. It describes assumptions, interfaces, and extension points needed for
    reliable production use for the foreseeable future. Update this docstring when changing behavior,
    interfaces, or runtime assumptions.

"""
#!/usr/bin/env python3
import sys
import serial
import math
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy # <--- REQUIRED
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion

class DiffIMUBridge(Node):
    def __init__(self, port='/dev/ttyUSB0', baud=115200):
        super().__init__('diff_imu_bridge')

        # --- QOS PROFILE (CRITICAL FIX) ---
        # Must match EKF's "Best Effort" expectation
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- PUBLISHERS ---
        # CHANGED: Topic name to /imu/data_raw to match EKF
        # CHANGED: Added qos_profile=sensor_qos
        self.pub_imu_diff = self.create_publisher(Imu, '/imu/data_raw', sensor_qos)
        self.pub_pitch    = self.create_publisher(Float32, '/imu/pitch', sensor_qos)
        self.pub_roll     = self.create_publisher(Float32, '/imu/roll', sensor_qos)

        # --- SERIAL CONNECTION ---
        self.serial_port = None
        try:
            self.serial_port = serial.Serial(port, baud, timeout=0.1)
        except Exception as e:
            self.get_logger().error(f"Serial Connection Error: {e}")
        
        # --- CALIBRATION STATE ---
        self.calibration_buffer = []
        self.is_calibrated = False
        
        # Bias Vectors
        self.bias_accel_left  = np.zeros(3)
        self.bias_gyro_left   = np.zeros(3)
        self.bias_accel_right = np.zeros(3)
        self.bias_gyro_right  = np.zeros(3)
        
        # --- FILTER STATE ---
        self.pitch_filtered = 0.0
        self.roll_filtered  = 0.0
        self.last_timestamp = self.get_clock().now()

        # --- COVARIANCE ---
        ACCEL_COV = 0.0025 
        GYRO_COV = 0.0004 
        ORIENT_COV = 0.001 

        self.linear_accel_cov = [ACCEL_COV, 0., 0., 0., ACCEL_COV, 0., 0., 0., ACCEL_COV]
        self.angular_vel_cov = [GYRO_COV, 0., 0., 0., GYRO_COV, 0., 0., 0., GYRO_COV]
        self.orientation_cov = [ORIENT_COV, 0., 0., 0., ORIENT_COV, 0., 0., 0., ORIENT_COV]

        self.create_timer(0.01, self.read_serial_data)
        self.get_logger().info("DUAL IMU BRIDGE STARTED. Keep still for Calibration...")

    def read_serial_data(self):
        if self.serial_port is None: return
        try:
            raw_line = self.serial_port.readline().decode('utf-8', errors='replace').strip()
        except: return
        if not raw_line: return
        
        data_parts = raw_line.split(',')
        if len(data_parts) != 12: return

        try:
            values = [float(x) for x in data_parts]
            accel_left  = np.array(values[0:3])
            gyro_left   = np.array(values[3:6])
            accel_right = np.array(values[6:9])
            gyro_right  = np.array(values[9:12])

            # --- 1. CALIBRATION ---
            if not self.is_calibrated:
                self.calibration_buffer.append(values)
                if len(self.calibration_buffer) > 300:
                    calib_matrix = np.array(self.calibration_buffer)
                    self.bias_accel_left  = np.mean(calib_matrix[:, 0:3], axis=0)
                    self.bias_gyro_left   = np.mean(calib_matrix[:, 3:6], axis=0) 
                    self.bias_accel_right = np.mean(calib_matrix[:, 6:9], axis=0)
                    self.bias_gyro_right  = np.mean(calib_matrix[:, 9:12], axis=0) 
                    
                    self.bias_accel_left[2]  = 0.0
                    self.bias_accel_right[2] = 0.0
                    
                    self.is_calibrated = True
                    self.last_timestamp = self.get_clock().now()
                    self.get_logger().info("CALIBRATION COMPLETE.")
                return

            # --- 2. BIAS CORRECTION ---
            accel_left  -= self.bias_accel_left
            gyro_left   -= self.bias_gyro_left 
            accel_right -= self.bias_accel_right
            gyro_right  -= self.bias_gyro_right 
            
            # AVERAGE THE TWO SENSORS (Noise Reduction)
            accel_avg = (accel_left + accel_right) / 2.0
            gyro_avg  = (gyro_left + gyro_right) / 2.0

            # --- 3. FILTERING ---
            current_ros_time = self.get_clock().now()
            dt = (current_ros_time - self.last_timestamp).nanoseconds / 1e9
            self.last_timestamp = current_ros_time
            if dt <= 0: return

            pitch_accel = math.atan2(-accel_avg[0], math.sqrt(accel_avg[1]**2 + accel_avg[2]**2))
            roll_accel  = math.atan2(accel_avg[1], accel_avg[2])
            
            pitch_rate = math.radians(gyro_avg[1])
            roll_rate  = math.radians(gyro_avg[0])

            alpha = 0.98
            self.pitch_filtered = alpha * (self.pitch_filtered + pitch_rate * dt) + (1.0 - alpha) * pitch_accel
            self.roll_filtered  = alpha * (self.roll_filtered  + roll_rate * dt)  + (1.0 - alpha) * roll_accel

            # --- 4. PUBLISH ---
            imu_msg = Imu()
            imu_msg.header.stamp = current_ros_time.to_msg()
            imu_msg.header.frame_id = "imu_link"
            
            # Data
            imu_msg.linear_acceleration.x = accel_avg[0]
            imu_msg.linear_acceleration.y = accel_avg[1]
            imu_msg.linear_acceleration.z = accel_avg[2]

            imu_msg.angular_velocity.x = math.radians(gyro_avg[0])
            imu_msg.angular_velocity.y = math.radians(gyro_avg[1])
            imu_msg.angular_velocity.z = math.radians(gyro_avg[2]) # <--- EKF USES THIS

            # Orientation
            q = self.euler_to_quaternion(self.roll_filtered, self.pitch_filtered, 0.0)
            imu_msg.orientation = q

            # Covariance
            imu_msg.linear_acceleration_covariance = self.linear_accel_cov
            imu_msg.angular_velocity_covariance    = self.angular_vel_cov
            imu_msg.orientation_covariance         = self.orientation_cov
            
            self.pub_imu_diff.publish(imu_msg)

            # Debug
            p = Float32(); p.data = self.pitch_filtered
            r = Float32(); r.data = self.roll_filtered
            self.pub_pitch.publish(p)
            self.pub_roll.publish(r)

        except ValueError: pass

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    rclpy.init()
    node = DiffIMUBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()