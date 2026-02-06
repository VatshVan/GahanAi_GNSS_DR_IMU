import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry  
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from math import sin, cos, atan2
import numpy as np
from .ekf_core import EKF

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        self.ekf = EKF()
        
        self.last_imu_time = None
        self.filter_ready = False
        self.R_imu_base = 0.05
        self.R_gps = 0.5 # Trust GPS significantly for position

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.sub_imu = self.create_subscription(Imu, '/imu/data_raw', self.cb_imu, qos)
        self.sub_gps = self.create_subscription(PoseWithCovarianceStamped, '/gps/enu_pose', self.cb_gps, qos)
        
        self.pub_odom = self.create_publisher(Odometry, '/odometry/ekf', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def cb_gps(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if not self.filter_ready:
            self.ekf.state[0], self.ekf.state[1] = x, y
            self.filter_ready = True
            return
        self.ekf.update_gps(x, y, self.R_gps)

    def cb_imu(self, msg):
        # 1. Orientation handling
        q = msg.orientation
        current_yaw = atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        
        # 2. Timing
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_imu_time is None:
            self.last_imu_time = now
            return
        dt = now - self.last_imu_time
        self.last_imu_time = now

        # 3. Dynamic Scaling via BNO085 Accuracy (msg.orientation_covariance[0])
        accuracy = msg.orientation_covariance[0]
        r_scale = 1.0 if accuracy >= 2 else 50.0

        if not self.ekf.is_yaw_initialized:
            self.ekf.state[2] = current_yaw
            self.ekf.is_yaw_initialized = True

        if not self.filter_ready: return

        # 4. Predict using IMU Linear Acceleration (X-axis)
        # This replaces the need for wheel encoders for velocity propagation
        self.ekf.predict(dt, accel=msg.linear_acceleration.x)
        
        # 5. Correct with Absolute Yaw and Yaw Rate
        self.ekf.update_imu_absolute(current_yaw, self.R_imu_base * r_scale)
        self.ekf.update_imu_velocity(msg.angular_velocity.z, self.R_imu_base)
        
        self.publish_odometry(msg.header.stamp)

    def publish_odometry(self, stamp):
        s = self.ekf.state
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id, t.child_frame_id = 'odom', 'base_link'
        t.transform.translation.x, t.transform.translation.y = float(s[0]), float(s[1])
        t.transform.rotation.z, t.transform.rotation.w = sin(s[2]/2.0), cos(s[2]/2.0)
        self.tf_broadcaster.sendTransform(t)