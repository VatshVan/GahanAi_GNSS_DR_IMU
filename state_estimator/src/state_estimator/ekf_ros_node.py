# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# from nav_msgs.msg import Odometry  
# from sensor_msgs.msg import Imu
# from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
# from tf2_ros import TransformBroadcaster
# from math import sin, cos, atan2, pi
# import numpy as np

# try:
#     from .ekf_core import EKF
# except ImportError:
#     from ekf_core import EKF

# class EKFNode(Node):
#     def __init__(self):
#         super().__init__('ekf_node')
#         self.ekf = EKF()
        
#         self.last_msg_time = None
#         self.gps_initialized = False
#         self.filter_ready = False
        
#         # Track GPS Jumps
#         self.gps_rejection_count = 0
        
#         # QOS: Best Effort (Crucial for live sensors)
#         sensor_qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.VOLATILE,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=10
#         )

#         # --- SUBSCRIBERS (Hybrid Setup) ---
#         # 1. Wheel (Always there, moderate trust)
#         self.create_subscription(Odometry, '/odom', self.callback_wheel_odom, sensor_qos)
        
#         # 2. Lidar (Optional, high trust)
#         self.create_subscription(Odometry, '/lidar_odom', self.callback_lidar_odom, sensor_qos)
        
#         # 3. IMU (Always there)
#         self.create_subscription(Imu, '/imu/data_raw', self.callback_imu, sensor_qos)
        
#         # 4. GPS (With Safety Gate)
#         self.create_subscription(PoseWithCovarianceStamped, '/gps/enu_pose', self.callback_gnss, sensor_qos)

#         self.pub_odom = self.create_publisher(Odometry, '/odometry/ekf', 10)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         self.get_logger().info("EKF STARTED. Waiting for ANY sensor data...")

#     # --- 1. WHEEL (Fallback Velocity) ---
#     def callback_wheel_odom(self, msg: Odometry):
#         if not self.filter_ready: return
#         speed = msg.twist.twist.linear.x
#         # Variance 0.1: We trust this, but we know it slips.
#         self.ekf.update_wheel_speed(speed, 0.1)

#     # --- 2. LIDAR (Precision Velocity - OPTIONAL) ---
#     def callback_lidar_odom(self, msg: Odometry):
#         if not self.filter_ready: return
#         # If this callback never triggers (no Lidar), the EKF just uses Wheel Odom.
#         # If it DOES trigger, we fuse it with High Confidence.
        
#         v_x = msg.twist.twist.linear.x
#         yaw_rate = msg.twist.twist.angular.z
        
#         # Variance 0.01: We trust this 10x more than wheels.
#         R_lidar = np.array([[0.01, 0.0], [0.0, 0.001]]) 
#         self.ekf.update_lidar_twist(v_x, yaw_rate, R_lidar)

#     # --- 3. GPS (With Outlier Rejection) ---
#     def callback_gnss(self, msg):
#         x = msg.pose.pose.position.x
#         y = msg.pose.pose.position.y
        
#         if not self.gps_initialized:
#             self.ekf.state[0] = x
#             self.ekf.state[1] = y
#             self.gps_initialized = True
#             self.filter_ready = True
#             self.get_logger().info("GPS INIT DONE.")
#             return

#         # Check for Teleportation Jumps (>3m)
#         is_valid = self.ekf.check_mahalanobis_gate(x, y, threshold=3.0)
        
#         if is_valid or self.gps_rejection_count > 10:
#             if not is_valid: self.gps_rejection_count = 0 # Forced Resync
            
#             R_gps = np.diag([1.0, 1.0]) 
#             self.ekf.update_gps(x, y, R_gps)
#         else:
#             self.gps_rejection_count += 1

#     # --- 4. IMU (Prediction + Gyro) ---
#     def callback_imu(self, msg: Imu):
#         # Init Yaw
#         if not self.ekf.is_yaw_initialized:
#             norm = msg.orientation.x**2 + msg.orientation.y**2 + msg.orientation.z**2 + msg.orientation.w**2
#             if norm > 0.001:
#                 siny = 2 * (msg.orientation.w * msg.orientation.z + msg.orientation.x * msg.orientation.y)
#                 cosy = 1 - 2 * (msg.orientation.y * msg.orientation.y + msg.orientation.z * msg.orientation.z)
#                 self.ekf.state[2] = atan2(siny, cosy)
#                 self.ekf.is_yaw_initialized = True
#                 self.get_logger().info("IMU YAW INIT DONE.")

#         if not self.filter_ready: return

#         # Time Delta
#         now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
#         if self.last_msg_time is None: self.last_msg_time = now; return
#         dt = now - self.last_msg_time
#         self.last_msg_time = now
#         if dt <= 0: return

#         # PREDICT
#         self.ekf.predict(dt)

#         # UPDATE GYRO
#         yaw_rate = msg.angular_velocity.z
#         self.ekf.update_imu_gyro(yaw_rate, 0.005) 
        
#         self.publish_odometry(msg.header.stamp)

#     def publish_odometry(self, stamp):
#         s = self.ekf.state
#         t = TransformStamped()
#         t.header.stamp = stamp
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_link'
#         t.transform.translation.x = float(s[0])
#         t.transform.translation.y = float(s[1])
#         t.transform.rotation.z = sin(s[2]/2.0)
#         t.transform.rotation.w = cos(s[2]/2.0)
#         self.tf_broadcaster.sendTransform(t)
        
#         o = Odometry()
#         o.header = t.header
#         o.child_frame_id = t.child_frame_id
#         o.pose.pose.position.x = float(s[0])
#         o.pose.pose.position.y = float(s[1])
#         o.twist.twist.linear.x = float(s[3])
#         o.twist.twist.angular.z = float(s[4])
#         self.pub_odom.publish(o)

# def main(args=None):
#     rclpy.init(args=args)
#     rclpy.spin(EKFNode())
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()









import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry  
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
from math import sin, cos, atan2, pi
import numpy as np

try:
    from .ekf_core import EKF
except ImportError:
    from ekf_core import EKF

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        
        self.declare_parameters(namespace='', parameters=[
            ('topic_imu', '/imu/data_raw'),
            ('topic_gps', '/gps/enu_pose'),
            ('topic_odom', '/odom'),
            ('topic_lidar', '/lidar_odom'),
            ('odom_frame', 'odom'),
            ('base_frame', 'base_link'),
            ('initial_covariance', [1.0]*5),
            ('process_noise', [0.1]*5),
            ('gps_pos_noise', 1.0),
            ('wheel_vel_noise', 0.1),
            ('imu_gyro_noise', 0.01),
            ('lidar_vel_noise', 0.01),
            # INCREASED GATE: Accommodate Side Slip (Physical Model Mismatch)
            ('gps_outlier_threshold', 10.0), 
            ('max_gps_rejections', 10),
        ])

        self.ekf = EKF()
        self.ekf.setup_matrices(
            self.get_parameter('initial_covariance').value,
            self.get_parameter('process_noise').value
        )
        
        self.R_gps = self.get_parameter('gps_pos_noise').value
        self.R_wheel = self.get_parameter('wheel_vel_noise').value
        self.R_imu = self.get_parameter('imu_gyro_noise').value
        self.R_lidar = self.get_parameter('lidar_vel_noise').value
        self.gps_thresh = self.get_parameter('gps_outlier_threshold').value
        self.max_rejects = self.get_parameter('max_gps_rejections').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.last_msg_time = None
        self.gps_reject_count = 0
        self.filter_ready = False

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Imu, self.get_parameter('topic_imu').value, self.cb_imu, qos)
        self.create_subscription(Odometry, self.get_parameter('topic_odom').value, self.cb_wheel, qos)
        self.create_subscription(Odometry, self.get_parameter('topic_lidar').value, self.cb_lidar, qos)
        self.create_subscription(PoseWithCovarianceStamped, self.get_parameter('topic_gps').value, self.cb_gps, qos)

        self.pub_odom = self.create_publisher(Odometry, '/odometry/ekf', 10)
        self.tf = TransformBroadcaster(self)
        self.get_logger().info("EKF STARTED: Fusing IMU (Gyro+Head) + GPS + Wheel.")

    def cb_wheel(self, msg):
        if not self.filter_ready: return
        self.ekf.update_velocity(msg.twist.twist.linear.x, self.R_wheel, is_lidar=False)

    def cb_lidar(self, msg):
        if not self.filter_ready: return
        self.ekf.update_velocity(msg.twist.twist.linear.x, self.R_lidar, is_lidar=True)

    def cb_gps(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        if not self.filter_ready:
            self.ekf.state[0] = x
            self.ekf.state[1] = y
            self.filter_ready = True
            self.get_logger().info(f"GPS INIT: {x:.2f}, {y:.2f}")
            return

        # Gate Logic
        if self.ekf.check_gate(x, y, self.gps_thresh):
            # Normal Update: Valid GPS
            self.ekf.update_gps(x, y, self.R_gps)
            self.gps_reject_count = 0
        else:
            # Outlier Logic
            self.gps_reject_count += 1
            if self.gps_reject_count > self.max_rejects:
                # Force Reset: We are lost.
                self.get_logger().warn("GPS RESET: Too many rejections. Forcing update.")
                self.gps_reject_count = 0
                self.ekf.reset_covariance() # CRITICAL FIX: Explode P so we accept the jump
                self.ekf.update_gps(x, y, self.R_gps)

    def cb_imu(self, msg):
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        current_yaw = atan2(siny, cosy)

        if not self.ekf.is_yaw_initialized:
            self.ekf.state[2] = current_yaw
            self.ekf.is_yaw_initialized = True
            self.get_logger().info("YAW INITIALIZED.")

        if not self.filter_ready: return

        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_msg_time is None: self.last_msg_time = now; return
        dt = now - self.last_msg_time
        self.last_msg_time = now
        if dt <= 0: return

        self.ekf.predict(dt)
        self.ekf.update_imu(msg.angular_velocity.z, self.R_imu)
        self.ekf.update_heading(current_yaw, 0.05) 
        self.publish_odometry(msg.header.stamp)

    def publish_odometry(self, stamp):
        s = self.ekf.state
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x, t.transform.translation.y = float(s[0]), float(s[1])
        t.transform.rotation.z, t.transform.rotation.w = sin(s[2]/2.0), cos(s[2]/2.0)
        self.tf.sendTransform(t)
        
        o = Odometry()
        o.header = t.header
        o.child_frame_id = t.child_frame_id
        o.pose.pose.position.x, o.pose.pose.position.y = float(s[0]), float(s[1])
        o.twist.twist.linear.x, o.twist.twist.angular.z = float(s[3]), float(s[4])
        self.pub_odom.publish(o)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(EKFNode())
    rclpy.shutdown()