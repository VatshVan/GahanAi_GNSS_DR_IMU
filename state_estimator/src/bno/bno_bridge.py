import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
import numpy as np

class BNO085Bridge(Node):
    def __init__(self):
        super().__init__('bno_bridge')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(Imu, '/imu/data_raw', qos)
        
        # Assuming serial port sends: quat_i, quat_j, quat_k, quat_real, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accuracy
        # Serial connection logic would go here if not handled by a separate generic bridge
        self.get_logger().info("BNO085 Bridge Node Initialized.")

    def process_data(self, data_list):
        """
        Data: [qi, qj, qk, qr, gx, gy, gz, ax, ay, accuracy]
        Note: ax, ay should be SH2_LINEAR_ACCELERATION (gravity-removed)
        """
        if len(data_list) < 10: return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # Absolute Orientation
        msg.orientation.x, msg.orientation.y = data_list[0], data_list[1]
        msg.orientation.z, msg.orientation.w = data_list[2], data_list[3]

        # Angular Velocity
        msg.angular_velocity.x, msg.angular_velocity.y = data_list[4], data_list[5]
        msg.angular_velocity.z = data_list[6]

        # Linear Acceleration (Essential for V estimation without encoders)
        msg.linear_acceleration.x = data_list[7]
        msg.linear_acceleration.y = data_list[8]
        msg.linear_acceleration.z = 0.0  # Assuming no z-axis acceleration data provided
        # Accuracy packing
        msg.orientation_covariance[0] = float(data_list[9])
        self.pub.publish(msg)