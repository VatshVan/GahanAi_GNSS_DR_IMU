"""
IMU Bridge (Multi-Driver)
-------------------------
Supports:
1. Generic Serial IMU (Default)
2. BNO085 (Adafruit) - Uncomment BNO085Driver to use.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
import serial
import numpy as np

# =============================================================================
# 1. GENERIC SERIAL DRIVER (Active)
# =============================================================================
class GenericIMUDriver(Node):
    def __init__(self):
        super().__init__('imu_bridge')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frequency', 100.0)
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(Imu, '/imu/data_raw', qos)
        
        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.get_logger().info(f"Generic IMU Connected on {port}")
        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")
            self.ser = None

        self.create_timer(1.0/self.get_parameter('frequency').value, self.read_loop)
        
        # Simple Bias Calibration
        self.bias = np.zeros(3)
        self.calib_count = 0
        self.is_calibrated = False

    def read_loop(self):
        if not self.ser: return
        try:
            line = self.ser.readline().decode().strip()
            parts = [float(x) for x in line.split(',')]
            if len(parts) != 12: return 
            
            # Average Dual IMU Gyros (Indices 3-5 and 9-11)
            gyro_deg = (np.array(parts[3:6]) + np.array(parts[9:12])) / 2.0
            
            if not self.is_calibrated:
                self.perform_calibration(gyro_deg)
                return

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu_link"
            
            # Deg/s -> Rad/s - Bias
            gyro_rad = np.radians(gyro_deg - self.bias)
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro_rad
            
            # Orientation Identity (EKF handles integration)
            msg.orientation.w = 1.0
            
            self.pub.publish(msg)
        except Exception: pass

    def perform_calibration(self, raw_gyro):
        if self.calib_count < 200:
            self.bias += raw_gyro
            self.calib_count += 1
        elif self.calib_count == 200:
            self.bias /= 200.0
            self.is_calibrated = True
            self.get_logger().info(f"IMU CALIBRATED. Bias: {self.bias}")


# =============================================================================
# 2. BNO085 DRIVER (Keep Commented until needed)
# =============================================================================
# To use: 
# 1. Install library: pip3 install adafruit-circuitpython-bno08x
# 2. In main(), comment out GenericIMUDriver and uncomment BNO085Driver
"""
import board
import busio
from adafruit_bno08x import (BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_ROTATION_VECTOR)
from adafruit_bno08x.i2c import BNO08X_I2C

class BNO085Driver(Node):
    def __init__(self):
        super().__init__('imu_bridge')
        self.declare_parameter('frequency', 100.0)
        
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(Imu, '/imu/data_raw', qos)
        
        try:
            # I2C Setup (SCL=Pin 5, SDA=Pin 3 on RPi)
            i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = BNO08X_I2C(i2c)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            self.get_logger().info("BNO085 Connected")
        except Exception as e:
            self.get_logger().error(f"BNO Error: {e}")
            self.bno = None

        self.create_timer(1.0/self.get_parameter('frequency').value, self.read_loop)

    def read_loop(self):
        if not self.bno: return
        try:
            # BNO085 gives Quaternions directly!
            quat = self.bno.quaternion
            gyro = self.bno.gyro
            
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu_link"
            
            msg.orientation.i = quat[0]
            msg.orientation.j = quat[1]
            msg.orientation.k = quat[2]
            msg.orientation.real = quat[3]
            
            msg.angular_velocity.x = gyro[0]
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = gyro[2]
            
            self.pub.publish(msg)
        except Exception: pass
"""

def main(args=None):
    rclpy.init(args=args)
    
    # --- SELECT DRIVER ---
    node = GenericIMUDriver()
    # node = BNO085Driver() 
    
    rclpy.spin(node)
    rclpy.shutdown()