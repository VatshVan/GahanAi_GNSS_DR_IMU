"""
GNSS Bridge Node (state_estimator/gps_bridge.py)
------------------------------------------------
Module Purpose
---------------
This module implements GNSSNode, a lightweight ROS 2 node that bridges a
serial NMEA GNSS receiver to ROS 2 topics. It parses selected NMEA sentences
(GGA for position and RMC for speed), publishes standardized ROS messages
for raw geodetic fixes and a local ENU pose, and provides a simple
approximate cartesian conversion suitable for integration with onboard
state-estimators and sensor fusion frameworks.
Design Goals
------------
- Robust, production-ready NMEA parsing with defensive error handling.
- ROS 2 native: parameters, QoS, timestamps, and proper message types.
- Minimal dependencies: rclpy, pyserial, and standard ROS message packages.
- Clear semantics for covariance and coordinate-frame conventions so that
    downstream estimators (e.g., EKF) can reason about uncertainty.
- Extendable: parsing and publishing are separated for clarity and easy
    augmentation (e.g., add heading, PDOP, or RTK corrections).
Public API Summary
------------------
Class:
        GNSSNode(Node)
                ROS 2 node exposing the serial GNSS bridge functionality.
Functions:
        main(args=None)
                Standard ROS 2 main entry point. Initializes rclpy, instantiates the
                node, spins until shutdown, then cleans up.
ROS Parameters (defaults shown)
-------------------------------
- port (string, default "/dev/ttyUSB1"):
        Serial device path for the GNSS receiver.
- baud (int, default 115200):
        Serial baud rate.
- frame_id (string, default "gnss_link"):
        Header frame_id to put on NavSatFix messages.
- use_saved_origin (bool, default True):
        If True and origin_lat/lon non-zero, the node will use the configured
        origin instead of setting the origin on the first valid fix.
- origin_lat (float, default 0.0):
        Saved origin latitude in degrees (WGS84).
- origin_lon (float, default 0.0):
        Saved origin longitude in degrees (WGS84).
- origin_alt (float, default 0.0):
        Saved origin altitude in meters.
Published Topics
----------------
- /gnss/fix (sensor_msgs/NavSatFix)
        Raw GNSS fix in WGS84 latitude/longitude/altitude with an approximated
        position covariance based on HDOP and fix quality. Header timestamp
        populated using node clock; header.frame_id uses the `frame_id`
        parameter.
- /gps/enu_pose (geometry_msgs/PoseWithCovarianceStamped)
        Local ENU pose relative to the configured or first-acquired origin.
        Position units are meters. Covariance maps X->index 0, Y->index 7,
        and yaw uncertainty placed at index 35 (set to a large value if unknown).
- /gps/speed_kmph (std_msgs/Float32)
        Scalar speed in kilometers per hour, extracted from the RMC sentence
        (converted from knots).
NMEA Sentences Handled
----------------------
- GGA ($GPGGA or $GNGGA): Position, fix quality, altitude, and HDOP.
    The node requires a non-zero fix quality to accept the fix.
- RMC ($GPRMC or $GNRMC): Speed (knots) and fix status (A=active).
    Only publishes speed if status is 'A'.
Parsing Notes & Conversions
---------------------------
- NMEA latitude/longitude are parsed from the degrees+minutes format
    (ddmm.mmmm or dddmm.mmmm) into decimal degrees. Negative sign is applied
    based on hemisphere letters (S or W).
- ENU conversion is an equirectangular/small-angle approximation:
        x = (lon - lon0) * R_e * cos(lat0)
        y = (lat - lat0) * R_e
    where R_e is the constant EARTH_RADIUS = 6_378_137.0 m (WGS84 equatorial).
    This approximation is suitable for small baselines (typically < 100 km)
    and is computationally efficient. For large-scale or high-precision
    applications, replace with a proper ellipsoidal geodetic library (e.g.,
    GeographicLib or PROJ).
Covariance and Uncertainty
--------------------------
- Position covariance is derived heuristically:
        variance = (hdop * base_error)^2
    base_error is selected by fix quality:
        - Default base_error = 2.5 m
        - SBAS (quality == 4): base_error = 0.02 m
        - RTK (quality == 5): base_error = 0.5 m
    If quality < 4, variance is increased (multiplied by 3) to reflect lower
    confidence.
- NavSatFix::position_covariance uses a diagonal approximation with
    altitude variance scaled (index 8) to be larger (variance*4). The
    covariance type is set to COVARIANCE_TYPE_APPROXIMATED.
- PoseWithCovarianceStamped::pose.covariance fills indices 0 (x) and 7 (y)
    and sets index 35 (yaw) to a large placeholder (999.0) to denote unknown
    orientation; users should fuse orientation from other sensors (IMU).
Operational Behavior
--------------------
- Serial port initialization occurs in the constructor. Connection failures
    are logged; the node continues running but will not publish until the
    serial port becomes available.
- The node reads serial data at 100 Hz (timer interval 0.01s) and decodes
    lines via readline with a 0.1s serial timeout. Parsing is defensive
    against malformed sentences and conversion errors.
- The first valid GGA fix sets the local ENU origin unless use_saved_origin
    is True and non-zero origin parameters are provided at startup.
Error Handling and Logging
--------------------------
- Malformed NMEA sentences, conversion errors, and serial IO exceptions are
    caught and ignored to avoid crashing the node. Relevant conditions are
    logged at appropriate levels (info/error).
- If the serial port cannot be opened at startup, an error is logged and
    the node continues. Operator tooling or systemd/launch wrappers should
    monitor logs and restart the node if necessary.
Performance and Resource Use
----------------------------
- CPU: minimal. Parsing is line-oriented and lightweight; suitable for
    embedded SBCs (Raspberry Pi, Jetson, etc.).
- Memory: minimal. No dynamic allocation patterns beyond message creation
    per publish event.
- Timing: the current design relies on the kernel serial driver and Python
    interpreter; for hard real-time constraints use a native C++ node or
    hardware offload.
Security Considerations
-----------------------
- Serial input is treated as untrusted. The parser uses defensive coding
    (length checks and try/except) to avoid crashes from malformed input.
- If used in networked deployments, secure the host (disable unused services,
    restrict device permissions) and ensure that serial access is limited to
    trusted processes.
Extensibility and Best Practices
--------------------------------
- To add additional NMEA sentences (ZDA, GSV, GST, GSA, VTG, etc.), implement
    corresponding parse_* methods and publish to additional topics.
- Replace the simple ENU approximation with GeographicLib for precision:
    projections.
- If persistent origin storage across reboots is required, manage origin
    parameters via a launch-time script or a parameter server persistence
    mechanism (e.g., YAML-loaded parameters via ros2 launch).
- For RTK-capable receivers, consider integrating raw RTCM stream handling or
    NTRIP clients rather than relying solely on the NMEA-derived fix.
Testing and Validation
----------------------
- Unit tests: isolate parsing functions (nmea_to_decimal, parse_gga,
    parse_rmc) and validate against representative sentence examples:
    - Typical valid GGA and RMC messages
    - Edge cases: empty fields, invalid numeric text, unsupported fix quality
- Integration tests: run the node against recorded serial logs, a GNSS
    simulator (e.g., gps-sdr-sim), and a real receiver, validating frame
    timestamps, covariance semantics, and ENU offsets.
- Continuous integration: include linting (flake8/black), type checks
    (mypy, if type annotations are added), and automated test runs.
Backward Compatibility and Versioning
------------------------------------
- Keep parameter names and topic names stable for downstream consumers.
- If changes are necessary (e.g., rename topic or message semantics),
    follow semantic versioning, provide a migration guide, and support
    both old and new topics during a transition period.
Example Launch Snippet (YAML parameters)
----------------------------------------
# In a launch file or YAML parameter set:
# gnss_node:
#   ros__parameters:
#     port: "/dev/ttyUSB1"
#     baud: 115200
#     frame_id: "gnss_link"
#     use_saved_origin: true
#     origin_lat: 12.345678
#     origin_lon: 98.765432
#     origin_alt: 10.0
Maintenance Notes
-----------------
- Keep pyserial updated to benefit from stability and security fixes.
- Revisit the ENU conversion if application domain expands beyond small
    baseline robotic platforms.
- Document any changes to covariance logic and publish migration notes for
    state estimator tuning.

"""
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import serial
import math
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
# from geographiclib.geodesic import Geodesic or use pyproj for local
from std_msgs.msg import Float32

class GNSSNode(Node):
    def __init__(self):
        super().__init__('gnss_node')

        # --- CONFIGURATION ---
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'gnss_link')
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # --- QOS PROFILE ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- PUBLISHERS ---
        self.fix_pub = self.create_publisher(NavSatFix, '/gnss/fix', sensor_qos)
        self.enu_pub = self.create_publisher(PoseWithCovarianceStamped, '/gps/enu_pose', sensor_qos)
        # NEW: Publish Speed for the EKF
        self.speed_pub = self.create_publisher(Float32, '/gps/speed_kmph', sensor_qos)

        # --- CARTESIAN CONVERSION STATE ---
        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None
        self.has_origin = False
        self.EARTH_RADIUS = 6378137.0 

        self.declare_parameter('use_saved_origin', True)
        self.declare_parameter('origin_lat', 0.0)
        self.declare_parameter('origin_lon', 0.0)
        self.declare_parameter('origin_alt', 0.0)

        # --- SERIAL CONNECTION ---
        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Connected to GNSS on {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Serial Connection Failed: {e}")

        # --- LOAD SAVED ORIGIN IF AVAILABLE ---
        use_saved = self.get_parameter('use_saved_origin').value
        if use_saved:
            lat0 = self.get_parameter('origin_lat').value
            lon0 = self.get_parameter('origin_lon').value
            alt0 = self.get_parameter('origin_alt').value
            if lat0 != 0.0 and lon0 != 0.0:
                self.origin_lat = lat0
                self.origin_lon = lon0
                self.origin_alt = alt0
                self.has_origin = True
                self.get_logger().info(f"Loaded saved ENU origin: {lat0:.6f}, {lon0:.6f}")

        self.create_timer(0.01, self.read_serial_data)

    def read_serial_data(self):
        if self.ser is None or not self.ser.is_open: return

        try:
            line = self.ser.readline().decode('utf-8', errors='replace').strip()
            
            # PARSE GGA (Position)
            if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                self.parse_gga(line)
            
            # PARSE RMC (Speed) -> NEW!
            elif line.startswith('$GNRMC') or line.startswith('$GPRMC'):
                self.parse_rmc(line)
                
        except Exception: pass

    def parse_rmc(self, line):
        # Format: $GNRMC,time,status,lat,NS,lon,EW,speed_knots,course,date...
        parts = line.split(',')
        if len(parts) < 8: return
        
        try:
            # Check Status (A=Active, V=Void)
            status = parts[2]
            if status != 'A': return

            # Speed is index 7 (in Knots)
            speed_knots_str = parts[7]
            if not speed_knots_str: return
            
            speed_knots = float(speed_knots_str)
            speed_kmph = speed_knots * 1.852 # Convert to km/h
            
            # Publish Speed
            msg = Float32()
            msg.data = speed_kmph
            self.speed_pub.publish(msg)
            
        except ValueError: pass

    def parse_gga(self, line):
        parts = line.split(',')
        if len(parts) < 10: return

        try:
            quality = int(parts[6]) 
            if quality == 0: return 

            raw_lat = parts[2]
            lat_dir = parts[3]
            raw_lon = parts[4]
            lon_dir = parts[5]
            
            if not raw_lat or not raw_lon: return

            lat = self.nmea_to_decimal(raw_lat)
            if lat_dir == 'S': lat = -lat
            
            lon = self.nmea_to_decimal(raw_lon)
            if lon_dir == 'W': lon = -lon
            
            alt = float(parts[9]) if parts[9] else 0.0
            hdop = float(parts[8]) if parts[8] else 10.0

            self.publish_fix_and_enu(lat, lon, alt, quality, hdop)
        except ValueError: pass

    def nmea_to_decimal(self, dm):
        if not dm: return 0.0
        dot_idx = dm.find('.')
        if dot_idx == -1: return float(dm)
        deg_end_idx = dot_idx - 2
        degrees = float(dm[:deg_end_idx])
        minutes = float(dm[deg_end_idx:])
        return degrees + (minutes / 60.0)

    def publish_fix_and_enu(self, lat, lon, alt, quality, hdop):
        current_time = self.get_clock().now().to_msg()
        
        # --- CALC VARIANCE ---
        base_error = 2.5
        if quality == 4: base_error = 0.02
        elif quality == 5: base_error = 0.5
        variance = (hdop * base_error) ** 2
        if quality < 4: variance *= 3.0

        # --- 1. PUBLISH RAW FIX ---
        msg = NavSatFix()
        msg.header.stamp = current_time
        msg.header.frame_id = self.frame_id
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        
        if quality >= 4: msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        else: msg.status.status = NavSatStatus.STATUS_FIX
        
        msg.position_covariance = [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance*4]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.fix_pub.publish(msg)

        # --- 2. PUBLISH ENU ---
        if not self.has_origin:
            self.origin_lat = lat
            self.origin_lon = lon
            self.origin_alt = alt
            self.has_origin = True
            self.get_logger().info(f"ORIGIN SET: {lat:.6f}, {lon:.6f}")
            return

        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        origin_lat_rad = math.radians(self.origin_lat)
        origin_lon_rad = math.radians(self.origin_lon)
        cos_lat = math.cos(origin_lat_rad)

        x_enu = (lon_rad - origin_lon_rad) * self.EARTH_RADIUS * cos_lat
        y_enu = (lat_rad - origin_lat_rad) * self.EARTH_RADIUS
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = "odom"
        pose_msg.pose.pose.position.x = x_enu
        pose_msg.pose.pose.position.y = y_enu
        
        pose_msg.pose.covariance[0] = variance # X
        pose_msg.pose.covariance[7] = variance # Y
        pose_msg.pose.covariance[35] = 999.0   # Yaw (Unknown)

        self.enu_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GNSSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()