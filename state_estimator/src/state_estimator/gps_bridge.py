import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import serial
import math
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseWithCovarianceStamped

class GNSSNode(Node):
    def __init__(self):
        super().__init__('gnss_node')
        self.declare_parameters(namespace='', parameters=[
            ('port', '/dev/ttyUSB1'), ('baud', 115200),
            ('origin_lat', 0.0), ('origin_lon', 0.0)
        ])
        
        self.origin_lat = self.get_parameter('origin_lat').value
        self.origin_lon = self.get_parameter('origin_lon').value
        self.has_origin = (self.origin_lat != 0.0)
        self.EARTH_RADIUS = 6378137.0

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub_enu = self.create_publisher(PoseWithCovarianceStamped, '/gps/enu_pose', qos)
        
        try:
            self.ser = serial.Serial(self.get_parameter('port').value, self.get_parameter('baud').value, timeout=0.1)
        except Exception: self.ser = None

        self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        if not self.ser: return
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                parts = line.split(',')
                if parts[6] != '0': self.process_gga(parts)
        except Exception: pass

    def process_gga(self, parts):
        lat = self.nmea_to_deg(parts[2], parts[3])
        lon = self.nmea_to_deg(parts[4], parts[5])
        
        if not self.has_origin:
            self.origin_lat = lat; self.origin_lon = lon; self.has_origin = True
            self.get_logger().info(f"Set Origin: {lat}, {lon}")
            return

        x = (math.radians(lon) - math.radians(self.origin_lon)) * self.EARTH_RADIUS * math.cos(math.radians(self.origin_lat))
        y = (math.radians(lat) - math.radians(self.origin_lat)) * self.EARTH_RADIUS
        
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.pose.position.x = x; msg.pose.pose.position.y = y
        self.pub_enu.publish(msg)

    def nmea_to_deg(self, raw, direction):
        if not raw: return 0.0
        val = float(raw[:2]) + float(raw[2:])/60.0
        return -val if direction in ['S','W'] else val

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GNSSNode())
    rclpy.shutdown()