#!/usr/bin/env python3
# ekf_ros_node.py
#
# ROS2 Python node that wraps ekf_core.EKF and subscribes to:
#   /imu/data_raw  (sensor_msgs/Imu)
#   /wheel_odom    (nav_msgs/Odometry)  -> twist.twist.linear.x used as wheel speed
#   /gps/fix       (sensor_msgs/NavSatFix)
#   /gps/speed_mps (std_msgs/Float32)    -> fallback wheel speed
#
# Publishes:
#   /odometry/ekf  (nav_msgs/Odometry)
#
# Place ekf_core.EKF (your provided EKF) next to this file or install it.

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import numpy as np
from math import cos, sin, radians

from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistWithCovarianceStamped

# import your EKF
from .ekf_core import EKF

def latlon_to_local_xy(lat, lon, lat0, lon0):
    """
    Simple equirectangular approximation to convert lat/lon to local meters
    relative to (lat0, lon0). Accurate for small areas (few km).
    """
    # Earth radius (m)
    R = 6378137.0
    dlat = np.deg2rad(lat - lat0)
    dlon = np.deg2rad(lon - lon0)
    mean_lat = np.deg2rad((lat + lat0) / 2.0)
    x = R * dlon * np.cos(mean_lat)
    y = R * dlat
    return float(x), float(y)

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')

        # Params (tune these)
        self.declare_parameter('odom_topic', 'wheel_odom')
        self.declare_parameter('imu_topic', 'imu/data_raw')
        self.declare_parameter('gps_fix_topic', 'gps/fix')
        self.declare_parameter('gps_speed_topic', 'gps/speed_mps')
        self.declare_parameter('ekf_odom_topic', 'odometry/ekf')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')

        odom_topic = self.get_parameter('odom_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        gps_fix_topic = self.get_parameter('gps_fix_topic').value
        gps_speed_topic = self.get_parameter('gps_speed_topic').value
        ekf_odom_topic = self.get_parameter('ekf_odom_topic').value

        # EKF instance
        self.ekf = EKF()
        # initialize to something reasonable; large covariance
        self.ekf.P = np.diag([10.0, 10.0, (np.deg2rad(30.0))**2, 1.0])

        # state: x = [px, py, yaw, v]

        # Subscriptions
        self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_cb, 50)
        self.wheel_sub = self.create_subscription(Odometry, odom_topic, self.wheel_cb, 20)
        self.gps_sub = self.create_subscription(NavSatFix, gps_fix_topic, self.gps_cb, 5)
        self.gps_speed_sub = self.create_subscription(Float32, gps_speed_topic, self.gps_speed_cb, 5)

        # Publisher: fused odom
        self.odom_pub = self.create_publisher(Odometry, ekf_odom_topic, 10)

        # storage
        self.last_imu_time = None
        self.last_wheel_speed = None
        self.last_gps_time = None

        # reference for lat/lon -> local xy conversion
        self.lat0 = None
        self.lon0 = None

        self.get_logger().info('EKF node started')

    def imu_cb(self, msg: Imu):
        # Use timestamp to compute dt
        t = msg.header.stamp
        if t.sec == 0 and t.nanosec == 0:
            now = self.get_clock().now()
            t = Time(seconds=now.seconds_nanoseconds()[0], nanoseconds=now.seconds_nanoseconds()[1])

        stamp = t.sec + t.nanosec * 1e-9
        if self.last_imu_time is None:
            self.last_imu_time = stamp
            return

        dt = stamp - self.last_imu_time
        # guard dt
        if dt <= 0 or dt > 1.0:
            # ignore bad dt
            dt = 0.02
        self.last_imu_time = stamp

        # yaw rate from IMU (z axis)
        omega = 0.0
        try:
            omega = msg.angular_velocity.z
        except Exception:
            omega = 0.0

        # Predict step
        self.ekf.predict(omega=omega, dt=dt)

        # Optionally update with wheel speed if available (high-rate)
        if self.last_wheel_speed is not None:
            # Use wheel measurement to update v
            self.ekf.update_wheel(self.last_wheel_speed)

        # Publish fused odometry
        self.publish_odom(msg.header.stamp)

    def wheel_cb(self, msg: Odometry):
        # wheel speed assumed in twist.twist.linear.x (m/s)
        try:
            v = float(msg.twist.twist.linear.x)
            self.last_wheel_speed = v
        except Exception:
            pass
        # Optionally update immediate
        # self.ekf.update_wheel(v)

    def gps_speed_cb(self, msg: Float32):
        # if wheel speed not present, use gps speed as fallback
        if self.last_wheel_speed is None:
            try:
                self.last_wheel_speed = float(msg.data)
            except Exception:
                pass

    def gps_cb(self, msg: NavSatFix):
        # On first GPS fix, set reference origin
        if msg.status.status < 0 or msg.latitude == 0.0 and msg.longitude == 0.0:
            return

        if self.lat0 is None:
            self.lat0 = msg.latitude
            self.lon0 = msg.longitude

        x, y = latlon_to_local_xy(msg.latitude, msg.longitude, self.lat0, self.lon0)
        # Do GPS update
        self.ekf.update_gps(np.array([x, y]))
        self.last_gps_time = msg.header.stamp

        # Publish fused odometry using GPS stamp
        self.publish_odom(msg.header.stamp)

    def publish_odom(self, stamp):
        state = self.ekf.get_state()
        cov = self.ekf.get_cov()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.get_parameter('odom_frame_id').value
        odom.child_frame_id = self.get_parameter('base_frame_id').value

        odom.pose.pose.position.x = float(state[0])
        odom.pose.pose.position.y = float(state[1])
        odom.pose.pose.position.z = 0.0

        yaw = float(state[2])
        # convert yaw to quaternion
        qz = np.sin(yaw / 2.0)
        qw = np.cos(yaw / 2.0)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = float(state[3])
        odom.twist.twist.angular.z = 0.0

        # Fill covariance (nav_msgs/Odometry has 6x6 pose covariance and twist covariance)
        # We'll insert position (px,py) and yaw and speed approximations into the 6x6 arrays.
        # pose covariance: [x y z rot_x rot_y rot_z] (row-major)
        pose_cov = [0.0] * 36
        # px variance
        pose_cov[0] = float(cov[0,0])
        pose_cov[1] = float(cov[0,1])
        pose_cov[6] = float(cov[1,0])
        pose_cov[7] = float(cov[1,1])
        # yaw variance -> rot_z at index 35-? mapping: element (5,5) = index 35
        pose_cov[35] = float(cov[2,2])

        odom.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0] = float(cov[3,3])  # var of linear.x
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
