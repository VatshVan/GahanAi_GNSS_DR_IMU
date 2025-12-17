# # #!/usr/bin/env python3
# # # ekf_ros_node.py
# # #
# # # ROS2 Python node that wraps ekf_core.EKF and subscribes to:
# # #   /imu/data_raw  (sensor_msgs/Imu)
# # #   /wheel_odom    (nav_msgs/Odometry)  -> twist.twist.linear.x used as wheel speed
# # #   /gps/fix       (sensor_msgs/NavSatFix)
# # #   /gps/speed_mps (std_msgs/Float32)    -> fallback wheel speed
# # #
# # # Publishes:
# # #   /odometry/ekf  (nav_msgs/Odometry)
# # #
# # # Place ekf_core.EKF (your provided EKF) next to this file or install it.

# # import rclpy
# # from rclpy.node import Node
# # from rclpy.time import Time

# # import numpy as np
# # from math import cos, sin, radians

# # from sensor_msgs.msg import Imu, NavSatFix
# # from nav_msgs.msg import Odometry
# # from std_msgs.msg import Float32
# # from geometry_msgs.msg import TwistWithCovarianceStamped

# # # import your EKF
# # from .ekf_core import EKF

# # def latlon_to_local_xy(lat, lon, lat0, lon0):
# #     """
# #     Simple equirectangular approximation to convert lat/lon to local meters
# #     relative to (lat0, lon0). Accurate for small areas (few km).
# #     """
# #     # Earth radius (m)
# #     R = 6378137.0
# #     dlat = np.deg2rad(lat - lat0)
# #     dlon = np.deg2rad(lon - lon0)
# #     mean_lat = np.deg2rad((lat + lat0) / 2.0)
# #     x = R * dlon * np.cos(mean_lat)
# #     y = R * dlat
# #     return float(x), float(y)

# # class EKFNode(Node):
# #     def __init__(self):
# #         super().__init__('ekf_node')

# #         # Params (tune these)
# #         self.declare_parameter('odom_topic', 'wheel_odom')
# #         self.declare_parameter('imu_topic', 'imu/data_raw')
# #         self.declare_parameter('gps_fix_topic', 'gps/fix')
# #         self.declare_parameter('gps_speed_topic', 'gps/speed_mps')
# #         self.declare_parameter('ekf_odom_topic', 'odometry/ekf')
# #         self.declare_parameter('odom_frame_id', 'odom')
# #         self.declare_parameter('base_frame_id', 'base_link')

# #         odom_topic = self.get_parameter('odom_topic').value
# #         imu_topic = self.get_parameter('imu_topic').value
# #         gps_fix_topic = self.get_parameter('gps_fix_topic').value
# #         gps_speed_topic = self.get_parameter('gps_speed_topic').value
# #         ekf_odom_topic = self.get_parameter('ekf_odom_topic').value

# #         # EKF instance
# #         self.ekf = EKF()
# #         # initialize to something reasonable; large covariance
# #         self.ekf.P = np.diag([10.0, 10.0, (np.deg2rad(30.0))**2, 1.0])

# #         # state: x = [px, py, yaw, v]

# #         # Subscriptions
# #         self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_cb, 50)
# #         self.wheel_sub = self.create_subscription(Odometry, odom_topic, self.wheel_cb, 20)
# #         self.gps_sub = self.create_subscription(NavSatFix, gps_fix_topic, self.gps_cb, 5)
# #         self.gps_speed_sub = self.create_subscription(Float32, gps_speed_topic, self.gps_speed_cb, 5)

# #         # Publisher: fused odom
# #         self.odom_pub = self.create_publisher(Odometry, ekf_odom_topic, 10)

# #         # storage
# #         self.last_imu_time = None
# #         self.last_wheel_speed = None
# #         self.last_gps_time = None

# #         # reference for lat/lon -> local xy conversion
# #         self.lat0 = None
# #         self.lon0 = None

# #         self.get_logger().info('EKF node started')

# #     def imu_cb(self, msg: Imu):
# #         # Use timestamp to compute dt
# #         t = msg.header.stamp
# #         if t.sec == 0 and t.nanosec == 0:
# #             now = self.get_clock().now()
# #             t = Time(seconds=now.seconds_nanoseconds()[0], nanoseconds=now.seconds_nanoseconds()[1])

# #         stamp = t.nanoseconds * 1e-9
# #         if self.last_imu_time is None:
# #             self.last_imu_time = stamp
# #             return

# #         dt = stamp - self.last_imu_time
# #         # guard dt
# #         if dt <= 0 or dt > 1.0:
# #             # ignore bad dt
# #             dt = 0.02
# #         self.last_imu_time = stamp

# #         # yaw rate from IMU (z axis)
# #         omega = 0.0
# #         try:
# #             omega = msg.angular_velocity.z
# #         except Exception:
# #             omega = 0.0

# #         # Predict step
# #         self.ekf.predict(omega=omega, dt=dt)

# #         # Optionally update with wheel speed if available (high-rate)
# #         if self.last_wheel_speed is not None:
# #             # Use wheel measurement to update v
# #             self.ekf.update_wheel(self.last_wheel_speed)

# #         # Publish fused odometry
# #         self.publish_odom(msg.header.stamp)

# #     def wheel_cb(self, msg: Odometry):
# #         # wheel speed assumed in twist.twist.linear.x (m/s)
# #         try:
# #             v = float(msg.twist.twist.linear.x)
# #             self.last_wheel_speed = v
# #         except Exception:
# #             pass
# #         # Optionally update immediate
# #         # self.ekf.update_wheel(v)

# #     def gps_speed_cb(self, msg: Float32):
# #         # if wheel speed not present, use gps speed as fallback
# #         if self.last_wheel_speed is None:
# #             try:
# #                 self.last_wheel_speed = float(msg.data)
# #             except Exception:
# #                 pass

# #     def gps_cb(self, msg: NavSatFix):
# #         # On first GPS fix, set reference origin
# #         if msg.status.status < 0 or msg.latitude == 0.0 and msg.longitude == 0.0:
# #             return

# #         if self.lat0 is None:
# #             self.lat0 = msg.latitude
# #             self.lon0 = msg.longitude

# #         x, y = latlon_to_local_xy(msg.latitude, msg.longitude, self.lat0, self.lon0)
# #         # Do GPS update
# #         self.ekf.update_gps(np.array([x, y]))
# #         self.last_gps_time = msg.header.stamp

# #         # Publish fused odometry using GPS stamp
# #         self.publish_odom(msg.header.stamp)

# #     def publish_odom(self, stamp):
# #         state = self.ekf.get_state()
# #         cov = self.ekf.get_cov()

# #         odom = Odometry()
# #         odom.header.stamp = stamp
# #         odom.header.frame_id = self.get_parameter('odom_frame_id').value
# #         odom.child_frame_id = self.get_parameter('base_frame_id').value

# #         odom.pose.pose.position.x = float(state[0])
# #         odom.pose.pose.position.y = float(state[1])
# #         odom.pose.pose.position.z = 0.0

# #         yaw = float(state[2])
# #         # convert yaw to quaternion
# #         qz = np.sin(yaw / 2.0)
# #         qw = np.cos(yaw / 2.0)
# #         odom.pose.pose.orientation.x = 0.0
# #         odom.pose.pose.orientation.y = 0.0
# #         odom.pose.pose.orientation.z = qz
# #         odom.pose.pose.orientation.w = qw

# #         odom.twist.twist.linear.x = float(state[3])
# #         odom.twist.twist.angular.z = 0.0

# #         # Fill covariance (nav_msgs/Odometry has 6x6 pose covariance and twist covariance)
# #         # We'll insert position (px,py) and yaw and speed approximations into the 6x6 arrays.
# #         # pose covariance: [x y z rot_x rot_y rot_z] (row-major)
# #         pose_cov = [0.0] * 36
# #         # px variance
# #         pose_cov[0] = float(cov[0,0])
# #         pose_cov[1] = float(cov[0,1])
# #         pose_cov[6] = float(cov[1,0])
# #         pose_cov[7] = float(cov[1,1])
# #         # yaw variance -> rot_z at index 35-? mapping: element (5,5) = index 35
# #         pose_cov[35] = float(cov[2,2])

# #         odom.pose.covariance = pose_cov

# #         twist_cov = [0.0] * 36
# #         twist_cov[0] = float(cov[3,3])  # var of linear.x
# #         odom.twist.covariance = twist_cov

# #         self.odom_pub.publish(odom)


# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = EKFNode()
# #     try:
# #         rclpy.spin(node)
# #     finally:
# #         node.destroy_node()
# #         rclpy.shutdown()


# # if __name__ == '__main__':
# #     main()


# #!/usr/bin/env python3
# # ekf_ros_node.py
# #
# # State: x = [px, py, yaw, v]
# #
# # Inputs:
# #   - /imu/diff        (sensor_msgs/Imu)        -> yaw rate (z)
# #   - /wheel_odom     (nav_msgs/Odometry)      -> linear.x velocity
# #   - /gps/fix        (sensor_msgs/NavSatFix)  -> position correction
# #   - /gps/speed_mps  (std_msgs/Float32)       -> velocity fallback
# #
# # Output:
# #   - /odometry/ekf   (nav_msgs/Odometry)

# # import rclpy
# # from rclpy.node import Node
# # from rclpy.time import Time
# # from geometry_msgs.msg import TransformStamped
# # from tf2_ros import TransformBroadcaster

# # import numpy as np
# # from math import sin, cos

# # from sensor_msgs.msg import Imu, NavSatFix
# # from nav_msgs.msg import Odometry
# # from std_msgs.msg import Float32

# # from .ekf_core import EKF   # your EKF implementation


# # # ------------------ UTIL ------------------

# # def latlon_to_local_xy(lat, lon, lat0, lon0):
# #     R = 6378137.0  # Earth radius (m)
# #     dlat = np.deg2rad(lat - lat0)
# #     dlon = np.deg2rad(lon - lon0)
# #     mean_lat = np.deg2rad((lat + lat0) / 2.0)

# #     x = R * dlon * np.cos(mean_lat)
# #     y = R * dlat
# #     return float(x), float(y)


# # # ------------------ EKF NODE ------------------

# # class EKFNode(Node):
# #     def __init__(self):
# #         super().__init__('ekf_node')

# #         # ---------------- PARAMETERS ----------------
# #         self.declare_parameter('imu_topic', '/imu/diff')
# #         self.declare_parameter('wheel_odom_topic', '/wheel_odom')
# #         self.declare_parameter('gps_fix_topic', '/gps/fix')
# #         self.declare_parameter('gps_speed_topic', '/gps/speed_mps')
# #         self.declare_parameter('ekf_odom_topic', '/odometry/ekf')

# #         self.declare_parameter('odom_frame', 'odom')
# #         self.declare_parameter('base_frame', 'base_link')

# #         # ---------------- EKF ----------------
# #         self.ekf = EKF()

# #         # Initial covariance (large uncertainty)
# #         self.ekf.P = np.diag([
# #             10.0,            # px
# #             10.0,            # py
# #             np.deg2rad(30)**2,  # yaw
# #             1.0              # v
# #         ])

# #         # ---------------- STATE ----------------
# #         self.last_imu_time = None
# #         self.last_wheel_speed = None

# #         self.lat0 = None
# #         self.lon0 = None

# #         # ---------------- SUBSCRIBERS ----------------
# #         self.create_subscription(
# #             Imu,
# #             self.get_parameter('imu_topic').value,
# #             self.imu_cb,
# #             100
# #         )

# #         self.create_subscription(
# #             Odometry,
# #             self.get_parameter('wheel_odom_topic').value,
# #             self.wheel_cb,
# #             20
# #         )

# #         self.create_subscription(
# #             Float32,
# #             self.get_parameter('gps_speed_topic').value,
# #             self.gps_speed_cb,
# #             10
# #         )

# #         self.create_subscription(
# #             NavSatFix,
# #             self.get_parameter('gps_fix_topic').value,
# #             self.gps_cb,
# #             5
# #         )

# #         # ---------------- PUBLISHER ----------------
# #         self.odom_pub = self.create_publisher(
# #             Odometry,
# #             self.get_parameter('ekf_odom_topic').value,
# #             10
# #         )

# #         self.get_logger().info("Publishing TF odom -> base_link")

# #         self.tf_broadcaster = TransformBroadcaster(self)

# #         self.get_logger().info('EKF node (Diff IMU) started')

# #     # ---------------- IMU CALLBACK ----------------
# #     def imu_cb(self, msg: Imu):
# #         t = msg.header.stamp
# #         if t.sec == 0 and t.nanosec == 0:
# #             t = self.get_clock().now()

# #         stamp = t.nanoseconds * 1e-9

# #         if self.last_imu_time is None:
# #             self.last_imu_time = stamp
# #             return

# #         dt = stamp - self.last_imu_time
# #         self.last_imu_time = stamp

# #         if dt <= 0.0 or dt > 0.5:
# #             return

# #         # Yaw rate from DIFFERENTIAL IMU
# #         omega = msg.angular_velocity.z

# #         # --------- PREDICT ---------
# #         self.ekf.predict(omega=omega, dt=dt)

# #         # --------- VELOCITY UPDATE ---------
# #         if self.last_wheel_speed is not None:
# #             self.ekf.update_wheel(self.last_wheel_speed)

# #         self.publish_odom(msg.header.stamp)

# #     # ---------------- WHEEL ODOM ----------------
# #     def wheel_cb(self, msg: Odometry):
# #         try:
# #             self.last_wheel_speed = float(msg.twist.twist.linear.x)
# #         except Exception:
# #             pass

# #     # ---------------- GPS SPEED ----------------
# #     def gps_speed_cb(self, msg: Float32):
# #         if self.last_wheel_speed is None:
# #             try:
# #                 self.last_wheel_speed = float(msg.data)
# #             except Exception:
# #                 pass

# #     # ---------------- GPS POSITION ----------------
# #     def gps_cb(self, msg: NavSatFix):
# #         if msg.status.status < 0:
# #             return

# #         if self.lat0 is None:
# #             self.lat0 = msg.latitude
# #             self.lon0 = msg.longitude
# #             return

# #         x, y = latlon_to_local_xy(
# #             msg.latitude, msg.longitude,
# #             self.lat0, self.lon0
# #         )

# #         self.ekf.update_gps(np.array([x, y]))
# #         self.publish_odom(msg.header.stamp)

# #     # ---------------- ODOM PUBLISH ----------------
# #     def publish_odom(self, stamp):
# #         state = self.ekf.get_state()
# #         cov = self.ekf.get_cov()

# #         odom = Odometry()
# #         odom.header.stamp = stamp
# #         odom.header.frame_id = self.get_parameter('odom_frame').value
# #         odom.child_frame_id = self.get_parameter('base_frame').value

# #         # Position
# #         odom.pose.pose.position.x = float(state[0])
# #         odom.pose.pose.position.y = float(state[1])
# #         odom.pose.pose.position.z = 0.0

# #         # Orientation (yaw → quaternion)
# #         yaw = float(state[2])
# #         qz = sin(yaw / 2.0)
# #         qw = cos(yaw / 2.0)

# #         odom.pose.pose.orientation.x = 0.0
# #         odom.pose.pose.orientation.y = 0.0
# #         odom.pose.pose.orientation.z = qz
# #         odom.pose.pose.orientation.w = qw


# #         # Velocity
# #         odom.twist.twist.linear.x = float(state[3])
# #         odom.twist.twist.angular.z = 0.0

# #         # Covariance
# #         pose_cov = [0.0] * 36
# #         pose_cov[0] = cov[0, 0]
# #         pose_cov[1] = cov[0, 1]
# #         pose_cov[6] = cov[1, 0]
# #         pose_cov[7] = cov[1, 1]
# #         pose_cov[35] = cov[2, 2]
# #         odom.pose.covariance = pose_cov

# #         twist_cov = [0.0] * 36
# #         twist_cov[0] = cov[3, 3]
# #         odom.twist.covariance = twist_cov

# #         self.odom_pub.publish(odom)

# #         # --- TF: odom -> base_link ---
# #         t = TransformStamped()
# #         t.header.stamp = stamp
# #         t.header.frame_id = self.get_parameter('odom_frame').value
# #         t.child_frame_id = self.get_parameter('base_frame').value

# #         t.transform.translation.x = float(state[0])
# #         t.transform.translation.y = float(state[1])
# #         t.transform.translation.z = 0.0

# #         # Convert yaw to quaternion
# #         yaw = float(state[2])
# #         qz = np.sin(yaw / 2.0)
# #         qw = np.cos(yaw / 2.0)

# #         t.transform.rotation.x = 0.0
# #         t.transform.rotation.y = 0.0
# #         t.transform.rotation.z = qz
# #         t.transform.rotation.w = qw

# #         self.tf_broadcaster.sendTransform(t)


# # # ---------------- MAIN ----------------

# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = EKFNode()
# #     try:
# #         rclpy.spin(node)
# #     finally:
# #         node.destroy_node()
# #         rclpy.shutdown()


# # if __name__ == '__main__':
# #     main()





























# # #!/usr/bin/env python3
# # import rclpy
# # from rclpy.node import Node
# # from rclpy.time import Time
# # from geometry_msgs.msg import TransformStamped
# # from tf2_ros import TransformBroadcaster

# # import numpy as np
# # from math import sin, cos

# # from sensor_msgs.msg import Imu, NavSatFix
# # from nav_msgs.msg import Odometry
# # from std_msgs.msg import Float32
# # from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# # from .ekf_core import EKF  # your EKF implementation


# # def latlon_to_local_xy(lat, lon, lat0, lon0):
# #     R = 6378137.0  # Earth radius (m)
# #     dlat = np.deg2rad(lat - lat0)
# #     dlon = np.deg2rad(lon - lon0)
# #     mean_lat = np.deg2rad((lat + lat0) / 2.0)
# #     x = R * dlon * np.cos(mean_lat)
# #     y = R * dlat
# #     return float(x), float(y)


# # class EKFNode(Node):
# #     def __init__(self):
# #         super().__init__('ekf_node')

# #         # ---------------- PARAMETERS ----------------
# #         self.declare_parameter('imu_topic', '/imu/diff')
# #         # self.declare_parameter('wheel_odom_topic', '/wheel_odom')
# #         # self.declare_parameter('gps_fix_topic', '/gps/fix')
# #         # self.declare_parameter('gps_speed_topic', '/gps/speed_mps')
# #         self.declare_parameter('ekf_odom_topic', '/odometry/ekf')

# #         self.declare_parameter('odom_frame', 'odom')
# #         self.declare_parameter('base_frame', 'base_link')

# #         # ---------------- EKF ----------------
# #         self.ekf = EKF()
# #         # Initial covariance
# #         self.ekf.P = np.diag([10.0, 10.0, np.deg2rad(30)**2, 1.0])

# #         # ---------------- STATE ----------------
# #         self.last_imu_time = None
# #         self.last_wheel_speed = None
# #         self.lat0 = None
# #         self.lon0 = None

# #         # qos = QoSProfile(
# #         #     reliability=ReliabilityPolicy.BEST_EFFORT, # Match the sensor
# #         #     history=HistoryPolicy.KEEP_LAST,
# #         #     depth=10
# #         # )

# #         # ---------------- SUBSCRIBERS ----------------
# #         self.create_subscription(
# #             Imu,
# #             self.get_parameter('imu_topic').value,
# #             self.imu_cb,
# #             # qos,
# #             10#0
# #         )
# #         # self.create_subscription(
# #         #     Odometry,
# #         #     self.get_parameter('wheel_odom_topic').value,
# #         #     self.wheel_cb,
# #         #     20
# #         # )
# #         # self.create_subscription(
# #         #     Float32,
# #         #     self.get_parameter('gps_speed_topic').value,
# #         #     self.gps_speed_cb,
# #         #     10
# #         # )
# #         # self.create_subscription(
# #         #     NavSatFix,
# #         #     self.get_parameter('gps_fix_topic').value,
# #         #     self.gps_cb,
# #         #     5
# #         # )

# #         # ---------------- PUBLISHER ----------------
# #         self.odom_pub = self.create_publisher(
# #             Odometry,
# #             self.get_parameter('ekf_odom_topic').value,
# #             100
# #         )
# #         # self.tf_broadcaster = TransformBroadcaster(self)

# #         self.get_logger().info('EKF node (Diff IMU) started')

# #     # ---------------- IMU CALLBACK ----------------
# #     def imu_cb(self, msg: Imu):
# #         # self.get_logger().info(f"IMU Recv! dt check...")
# #         t = msg.header.stamp
# #         if t.sec == 0 and t.nanosec == 0:
# #             t = self.get_clock().now()
# #         stamp = t.nanoseconds * 1e-9

# #         if self.last_imu_time is None:
# #             self.last_imu_time = stamp
# #             return

# #         dt = stamp - self.last_imu_time
# #         self.last_imu_time = stamp
# #         # Was 0.5, increase to 1.0 or higher to handle the lag
# #         if dt <= 0.0 or dt > 1.0: 
# #             self.get_logger().warn(f"IMU dt out of bounds: {dt:.3f}s")
# #             return

# #         # yaw rate from diff IMU
# #         omega = msg.angular_velocity.z

# #         # EKF predict
# #         self.ekf.predict(omega=omega, dt=dt)

# #         # EKF velocity update
# #         if self.last_wheel_speed is not None:
# #             self.ekf.update_wheel(self.last_wheel_speed)

# #         self.publish_odom(msg.header.stamp)

# #     # ---------------- WHEEL ODOM ----------------
# #     # def wheel_cb(self, msg: Odometry):
# #     #     try:
# #     #         self.last_wheel_speed = float(msg.twist.twist.linear.x)
# #     #     except Exception:
# #     #         pass

# #     # # ---------------- GPS SPEED ----------------
# #     # def gps_speed_cb(self, msg: Float32):
# #     #     if self.last_wheel_speed is None:
# #     #         try:
# #     #             self.last_wheel_speed = float(msg.data)
# #     #         except Exception:
# #     #             pass

# #     # # ---------------- GPS POSITION ----------------
# #     # def gps_cb(self, msg: NavSatFix):
# #     #     if msg.status.status < 0:
# #     #         return
# #     #     if self.lat0 is None:
# #     #         self.lat0 = msg.latitude
# #     #         self.lon0 = msg.longitude
# #     #         return
# #     #     x, y = latlon_to_local_xy(msg.latitude, msg.longitude, self.lat0, self.lon0)
# #     #     self.ekf.update_gps(np.array([x, y]))
# #     #     self.publish_odom(msg.header.stamp)

# #     # ---------------- ODOM & TF ----------------
# #     def publish_odom(self, stamp):
# #         state = self.ekf.get_state()
# #         cov = self.ekf.get_cov()

# #         # Odometry message
# #         odom = Odometry()
# #         odom.header.stamp = stamp
# #         odom.header.frame_id = self.get_parameter('odom_frame').value
# #         odom.child_frame_id = self.get_parameter('base_frame').value

# #         odom.pose.pose.position.x = float(state[0])
# #         odom.pose.pose.position.y = float(state[1])
# #         odom.pose.pose.position.z = 0.0

# #         # Correct quaternion conversion
# #         yaw = float(state[2])
# #         qz = sin(yaw / 2.0)
# #         qw = cos(yaw / 2.0)
# #         odom.pose.pose.orientation.x = 0.0
# #         odom.pose.pose.orientation.y = 0.0
# #         odom.pose.pose.orientation.z = qz
# #         odom.pose.pose.orientation.w = qw

# #         odom.twist.twist.linear.x = float(state[3])
# #         odom.twist.twist.angular.z = 0.0

# #         # Covariance
# #         pose_cov = [0.0] * 36
# #         pose_cov[0] = cov[0, 0]
# #         pose_cov[1] = cov[0, 1]
# #         pose_cov[6] = cov[1, 0]
# #         pose_cov[7] = cov[1, 1]
# #         pose_cov[35] = cov[2, 2]
# #         odom.pose.covariance = pose_cov

# #         twist_cov = [0.0] * 36
# #         twist_cov[0] = cov[3, 3]
# #         odom.twist.covariance = twist_cov

# #         self.odom_pub.publish(odom)

# #         # TF broadcaster
# #         # t = TransformStamped()
# #         # t.header.stamp = stamp
# #         # t.header.frame_id = self.get_parameter('odom_frame').value
# #         # t.child_frame_id = self.get_parameter('base_frame').value
# #         # t.transform.translation.x = float(state[0])
# #         # t.transform.translation.y = float(state[1])
# #         # t.transform.translation.z = 0.0
# #         # t.transform.rotation.x = 0.0
# #         # t.transform.rotation.y = 0.0
# #         # t.transform.rotation.z = qz
# #         # t.transform.rotation.w = qw
# #         # self.tf_broadcaster.sendTransform(t)


# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = EKFNode()
# #     try:
# #         rclpy.spin(node)
# #     finally:
# #         node.destroy_node()
# #         rclpy.shutdown()


# # if __name__ == '__main__':
# #     main()






























# # #!/usr/bin/env python3
# # import rclpy
# # from rclpy.node import Node
# # import numpy as np
# # from math import sin, cos
# # from sensor_msgs.msg import Imu
# # from nav_msgs.msg import Odometry
# # from .ekf_core import EKF

# # IMU_BASELINE = 0.30  # meters

# # class EKFNode(Node):
# #     def __init__(self):
# #         super().__init__('ekf_node_diff_imu')

# #         self.declare_parameter('imu1_topic', '/imu1/data_raw')
# #         self.declare_parameter('imu_diff_topic', '/imu/diff')
# #         self.declare_parameter('ekf_odom_topic', '/odometry/ekf')
# #         self.declare_parameter('odom_frame', 'odom')
# #         self.declare_parameter('base_frame', 'base_link')

# #         # EKF
# #         self.ekf = EKF()
# #         self.ekf.P = np.diag([10.0, 10.0, np.deg2rad(45)**2, 2.0, np.deg2rad(20)**2])
# #         self.last_imu_time = None

# #         # Subscribers
# #         self.create_subscription(Imu,
# #             self.get_parameter('imu1_topic').value,
# #             self.imu1_cb, 20)

# #         self.create_subscription(Imu,
# #             self.get_parameter('imu_diff_topic').value,
# #             self.diff_cb, 20)

# #         # Publisher
# #         self.odom_pub = self.create_publisher(Odometry,
# #             self.get_parameter('ekf_odom_topic').value, 50)

# #         self.imu1_omega_z = 0.0  # reference yaw rate from IMU1

# #         self.get_logger().info("EKF node started (IMU1 reference + diff IMU)")

# #     # ---------------- IMU1 CALLBACK (Reference) ----------------
# #     def imu1_cb(self, msg: Imu):
# #         self.imu1_omega_z = msg.angular_velocity.z

# #     # ---------------- DIFF IMU CALLBACK ----------------
# #     def diff_cb(self, msg: Imu):
# #         if msg.header.stamp.sec == 0:
# #             return

# #         t = msg.header.stamp.nanoseconds * 1e-9

# #         if self.last_imu_time is None:
# #             self.last_imu_time = t
# #             return

# #         dt = t - self.last_imu_time
# #         self.last_imu_time = t

# #         if dt <= 0.0 or dt > 1.0:
# #             return

# #         # ---------------- Differential IMU model ----------------
# #         delta_ax = msg.linear_acceleration.x  # Δa_x
# #         alpha_z = delta_ax / IMU_BASELINE      # yaw acceleration

# #         delta_gyro_z = msg.angular_velocity.z  # diff gyro weak constraint

# #         # ---------------- EKF Prediction ----------------
# #         # Combine IMU1 yaw rate (reference) with differential IMU correction
# #         omega = self.imu1_omega_z + 0.1 * delta_gyro_z

# #         # Predict state
# #         self.ekf.predict(alpha=alpha_z, omega=omega, dt=dt)

# #         self.publish_odom(msg.header.stamp)

# #     # ---------------- PUBLISH ODOM ----------------
# #     def publish_odom(self, stamp):
# #         state = self.ekf.get_state()
# #         cov = self.ekf.get_cov()

# #         x, y, yaw, v, omega = state

# #         odom = Odometry()
# #         odom.header.stamp = stamp
# #         odom.header.frame_id = self.get_parameter('odom_frame').value
# #         odom.child_frame_id = self.get_parameter('base_frame').value

# #         odom.pose.pose.position.x = float(x)
# #         odom.pose.pose.position.y = float(y)
# #         odom.pose.pose.orientation.z = sin(yaw/2.0)
# #         odom.pose.pose.orientation.w = cos(yaw/2.0)
# #         odom.twist.twist.linear.x = float(v)
# #         odom.twist.twist.angular.z = float(omega)

# #         self.odom_pub.publish(odom)


# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = EKFNode()
# #     try:
# #         rclpy.spin(node)
# #     finally:
# #         node.destroy_node()
# #         rclpy.shutdown()


# # if __name__ == '__main__':
# #     main()























# #!/usr/bin/env python3
# # ekf_ros_node.py
# #
# # ROS2 Python node that wraps ekf_core.EKF and subscribes to:
# #   /imu/data_raw  (sensor_msgs/Imu)
# #   /wheel_odom    (nav_msgs/Odometry)  -> twist.twist.linear.x used as wheel speed
# #   /gps/fix       (sensor_msgs/NavSatFix)
# #   /gps/speed_mps (std_msgs/Float32)    -> fallback wheel speed
# #
# # Publishes:
# #   /odometry/ekf  (nav_msgs/Odometry)
# #
# # Place ekf_core.EKF (your provided EKF) next to this file or install it.

# import rclpy
# from rclpy.node import Node
# from rclpy.time import Time

# import numpy as np
# from math import cos, sin, radians

# from sensor_msgs.msg import Imu, NavSatFix
# from nav_msgs.msg import Odometry
# from std_msgs.msg import Float32
# from geometry_msgs.msg import TwistWithCovarianceStamped

# # import your EKF
# from .ekf_core import EKF

# def latlon_to_local_xy(lat, lon, lat0, lon0):
#     """
#     Simple equirectangular approximation to convert lat/lon to local meters
#     relative to (lat0, lon0). Accurate for small areas (few km).
#     """
#     # Earth radius (m)
#     R = 6378137.0
#     dlat = np.deg2rad(lat - lat0)
#     dlon = np.deg2rad(lon - lon0)
#     mean_lat = np.deg2rad((lat + lat0) / 2.0)
#     x = R * dlon * np.cos(mean_lat)
#     y = R * dlat
#     return float(x), float(y)

# class EKFNode(Node):
#     def __init__(self):
#         super().__init__('ekf_node')

#         # Params (tune these)
#         self.declare_parameter('odom_topic', 'wheel_odom')
#         self.declare_parameter('imu_topic', 'imu/data_raw')
#         self.declare_parameter('gps_fix_topic', 'gps/fix')
#         self.declare_parameter('gps_speed_topic', 'gps/speed_mps')
#         self.declare_parameter('ekf_odom_topic', 'odometry/ekf')
#         self.declare_parameter('odom_frame_id', 'odom')
#         self.declare_parameter('base_frame_id', 'base_link')

#         odom_topic = self.get_parameter('odom_topic').value
#         imu_topic = self.get_parameter('imu_topic').value
#         gps_fix_topic = self.get_parameter('gps_fix_topic').value
#         gps_speed_topic = self.get_parameter('gps_speed_topic').value
#         ekf_odom_topic = self.get_parameter('ekf_odom_topic').value

#         # EKF instance
#         self.ekf = EKF()
#         # initialize to something reasonable; large covariance
#         self.ekf.P = np.diag([10.0, 10.0, (np.deg2rad(30.0))**2, 1.0])

#         # state: x = [px, py, yaw, v]

#         # Subscriptions
#         self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_cb, 50)
#         self.wheel_sub = self.create_subscription(Odometry, odom_topic, self.wheel_cb, 20)
#         self.gps_sub = self.create_subscription(NavSatFix, gps_fix_topic, self.gps_cb, 5)
#         self.gps_speed_sub = self.create_subscription(Float32, gps_speed_topic, self.gps_speed_cb, 5)

#         # Publisher: fused odom
#         self.odom_pub = self.create_publisher(Odometry, ekf_odom_topic, 10)

#         # storage
#         self.last_imu_time = None
#         self.last_wheel_speed = None
#         self.last_gps_time = None

#         # reference for lat/lon -> local xy conversion
#         self.lat0 = None
#         self.lon0 = None

#         self.get_logger().info('EKF node started')

#     def imu_cb(self, msg: Imu):
#         # 1. Convert the message timestamp to a Python Time object immediately
#         # This fixes the ".sec" vs ".nanoseconds" crash forever.
#         t = Time.from_msg(msg.header.stamp)

#         # 2. Check if the time is zero (invalid)
#         # We check nanoseconds directly on the Time object
#         if t.nanoseconds == 0:
#             t = self.get_clock().now()

#         # 3. Get the float timestamp (This will now ALWAYS work)
#         stamp = t.nanoseconds * 1e-9

#         if self.last_imu_time is None:
#             self.last_imu_time = stamp
#             return

#         dt = stamp - self.last_imu_time
#         # guard dt
#         if dt <= 0 or dt > 1.0:
#             # ignore bad dt
#             dt = 0.02
#         self.last_imu_time = stamp

#         # yaw rate from IMU (z axis)
#         omega = 0.0
#         try:
#             omega = msg.angular_velocity.z
#         except Exception:
#             omega = 0.0

#         # Predict step
#         self.ekf.predict(omega=omega, dt=dt)

#         # Optionally update with wheel speed if available (high-rate)
#         if self.last_wheel_speed is not None:
#             # Use wheel measurement to update v
#             self.ekf.update_wheel(self.last_wheel_speed)

#         # Publish fused odometry
#         self.publish_odom(msg.header.stamp)

#     def wheel_cb(self, msg: Odometry):
#         # wheel speed assumed in twist.twist.linear.x (m/s)
#         try:
#             v = float(msg.twist.twist.linear.x)
#             self.last_wheel_speed = v
#         except Exception:
#             pass
#         # Optionally update immediate
#         # self.ekf.update_wheel(v)

#     def gps_speed_cb(self, msg: Float32):
#         # if wheel speed not present, use gps speed as fallback
#         if self.last_wheel_speed is None:
#             try:
#                 self.last_wheel_speed = float(msg.data)
#             except Exception:
#                 pass

#     def gps_cb(self, msg: NavSatFix):
#         # On first GPS fix, set reference origin
#         if msg.status.status < 0 or msg.latitude == 0.0 and msg.longitude == 0.0:
#             return

#         if self.lat0 is None:
#             self.lat0 = msg.latitude
#             self.lon0 = msg.longitude

#         x, y = latlon_to_local_xy(msg.latitude, msg.longitude, self.lat0, self.lon0)
#         # Do GPS update
#         self.ekf.update_gps(np.array([x, y]))
#         self.last_gps_time = msg.header.stamp

#         # Publish fused odometry using GPS stamp
#         self.publish_odom(msg.header.stamp)

#     def publish_odom(self, stamp):
#         state = self.ekf.get_state()
#         cov = self.ekf.get_cov()

#         odom = Odometry()
#         odom.header.stamp = stamp
#         odom.header.frame_id = self.get_parameter('odom_frame_id').value
#         odom.child_frame_id = self.get_parameter('base_frame_id').value

#         odom.pose.pose.position.x = float(state[0])
#         odom.pose.pose.position.y = float(state[1])
#         odom.pose.pose.position.z = 0.0

#         yaw = float(state[2])
#         # convert yaw to quaternion
#         qz = np.sin(yaw / 2.0)
#         qw = np.cos(yaw / 2.0)
#         odom.pose.pose.orientation.x = 0.0
#         odom.pose.pose.orientation.y = 0.0
#         odom.pose.pose.orientation.z = qz
#         odom.pose.pose.orientation.w = qw

#         odom.twist.twist.linear.x = float(state[3])
#         odom.twist.twist.angular.z = 0.0

#         # Fill covariance (nav_msgs/Odometry has 6x6 pose covariance and twist covariance)
#         # We'll insert position (px,py) and yaw and speed approximations into the 6x6 arrays.
#         # pose covariance: [x y z rot_x rot_y rot_z] (row-major)
#         pose_cov = [0.0] * 36
#         # px variance
#         pose_cov[0] = float(cov[0,0])
#         pose_cov[1] = float(cov[0,1])
#         pose_cov[6] = float(cov[1,0])
#         pose_cov[7] = float(cov[1,1])
#         # yaw variance -> rot_z at index 35-? mapping: element (5,5) = index 35
#         pose_cov[35] = float(cov[2,2])

#         odom.pose.covariance = pose_cov

#         twist_cov = [0.0] * 36
#         twist_cov[0] = float(cov[3,3])  # var of linear.x
#         odom.twist.covariance = twist_cov

#         self.odom_pub.publish(odom)


# def main(args=None):
#     rclpy.init(args=args)
#     node = EKFNode()
#     try:
#         rclpy.spin(node)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()




















































































# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import numpy as np
# from math import sin, cos
# from sensor_msgs.msg import Imu
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster
# from collections import deque
# import math
# from .ekf_core import EKF

# class EKFNode(Node):
#     def __init__(self):
#         super().__init__('ekf_node_diff_imu')

#         self.accel_history = deque(maxlen=20) 
#         self.gyro_history = deque(maxlen=20)

#         # 1. HARDCODED TOPIC (No params, no YAML confusion)
#         target_topic = '/imu/diff'

#         self.ekf = EKF()
#         self.last_imu_time = None
#         self.IMU_BASELINE = 0.30 
#         self.msg_count = 0

#         # 2. SUBSCRIBER
#         self.create_subscription(Imu, target_topic, self.diff_cb, 10)

#         # 3. PUBLISHER
#         self.odom_pub = self.create_publisher(Odometry, '/odometry/ekf', 50)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # 4. HEARTBEAT TIMER (To prove the node is alive)
#         self.create_timer(1.0, self.check_status)

#         self.get_logger().info(f"HARDCODED MODE: Listening strictly to '{target_topic}'")

#     def check_status(self):
#         # This will print every second if the node is alive but getting no data
#         if self.msg_count == 0:
#             self.get_logger().warn("Waiting for IMU data... (Check Bridge!)")
#         else:
#             # If we are getting data, we don't need to spam, just show total count
#             pass

#     def diff_cb(self, msg: Imu):
#         # Time calc
#         current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
#         if self.last_imu_time is None:
#             self.last_imu_time = current_time
#             self.current_pitch = 0.0
#             return
#         dt = current_time - self.last_imu_time
#         self.last_imu_time = current_time
#         if dt <= 0: return

#         # Unpack Data
#         omega_z = msg.orientation.x          
#         omega_y = msg.orientation.y          
#         raw_accel_x = msg.orientation_covariance[0] 
#         raw_accel_z = msg.orientation_covariance[1] 
        
#         delta_ax = msg.linear_acceleration.x 
#         alpha_z = delta_ax / self.IMU_BASELINE

#         # --- 1. POPULATE HISTORY (For Jitter Check) ---
#         # We need this data to calculate variance later
#         self.accel_history.append(raw_accel_x)
#         self.gyro_history.append(omega_z)

#         # --- 2. COMPLEMENTARY FILTER (Pitch) ---
#         pitch_accel = np.arctan2(raw_accel_x, raw_accel_z)
        
#         # Adaptive Gain: If G-Force is weird (turning/accelerating), trust Gyro more.
#         # This prevents the "Slow Tilt" bug you mentioned earlier.
#         g_force = math.sqrt(raw_accel_x**2 + raw_accel_z**2)
#         alpha = 0.02
#         if abs(g_force - 9.81) > 0.5: alpha = 0.005 

#         if not hasattr(self, 'current_pitch'): self.current_pitch = 0.0
#         self.current_pitch = (1.0 - alpha) * (self.current_pitch + omega_y * dt) + \
#                              (alpha) * (pitch_accel)

#         # --- 3. GRAVITY COMPENSATION ---
#         gravity_effect = 9.81 * sin(self.current_pitch)
#         corrected_accel_x = raw_accel_x - gravity_effect 

#         # --- 4. STATIONARY CHECK (Protected) ---
#         is_stationary = False
#         current_vel = self.ekf.x[3]
        
#         # [CRITICAL GUARD]: Only check variance if we are barely moving (< 1.0 m/s)
#         # This protects against the "Highway Freeze" bug.
#         if abs(current_vel) < 1.0 and len(self.accel_history) == 20:
            
#             acc_var = np.var(self.accel_history)
#             gyro_var = np.var(self.gyro_history)
            
#             # If variance is low, we are physically stopped (engine idle/parked)
#             if acc_var < 0.005 and gyro_var < 0.0005:
#                 is_stationary = True

#         # --- 5. MOTION LOGIC ---
#         if is_stationary:
#             # We are definitely stopped. Lock everything to prevent drift.
#             self.ekf.x[3] = 0.0
#             omega_z = 0.0        # Lock Yaw
#             corrected_accel_x = 0.0
#         else:
#             # We are moving (Cruising OR Accelerating)
            
#             # Simple noise gate
#             if abs(corrected_accel_x) < 0.05:
#                 corrected_accel_x = 0.0
            
#             # Integrate
#             self.ekf.x[3] += corrected_accel_x * dt
            
#             # Very light air resistance (prevent infinite runaway)
#             self.ekf.x[3] *= 0.999 

#         # --- DIAGNOSIS PRINTS ---
#         if self.accel_history: var_debug = np.var(self.accel_history)
#         else: var_debug = 0.0

#         self.get_logger().info(
#             f"Var: {var_debug:.4f} | "
#             f"Pitch: {np.degrees(self.current_pitch):.1f}° | "
#             f"Net: {corrected_accel_x:.2f} | "
#             f"Vel: {self.ekf.x[3]:.2f}", 
#             throttle_duration_sec=0.2
#         )

#         self.ekf.predict(omega=omega_z, accel=corrected_accel_x, dt=dt) 
#         self.publish_odom(msg.header.stamp)

#     def publish_odom(self, stamp):
#         state = self.ekf.get_state()
#         x, y, yaw, v, omega = state

#         t = TransformStamped()
#         t.header.stamp = stamp
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_link'
#         t.transform.translation.x = float(x)
#         t.transform.translation.y = float(y)
#         t.transform.rotation.z = sin(yaw / 2.0)
#         t.transform.rotation.w = cos(yaw / 2.0)
#         self.tf_broadcaster.sendTransform(t)

#         odom = Odometry()
#         odom.header.stamp = stamp
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id = 'base_link'
#         odom.pose.pose.position.x = float(x)
#         odom.pose.pose.position.y = float(y)
#         odom.pose.pose.orientation = t.transform.rotation
#         odom.twist.twist.linear.x = float(v)
#         odom.twist.twist.angular.z = float(omega)
#         self.odom_pub.publish(odom)

# def main(args=None):
#     rclpy.init(args=args)
#     node = EKFNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import numpy as np
# from math import sin, cos
# from sensor_msgs.msg import Imu
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster
# from collections import deque
# import math

# # Import the NEW Hybrid EKF (File 1)
# from .ekf_core import EKF

# class EKFNode(Node):
#     def __init__(self):
#         super().__init__('ekf_node_diff_imu')

#         # --- STATIONARY DETECTION BUFFERS ---
#         self.accel_history = deque(maxlen=20) 
#         self.gyro_history = deque(maxlen=20)

#         # 1. HARDCODED TOPIC
#         target_topic = '/imu/diff'

#         self.ekf = EKF()
#         self.last_imu_time = None
#         self.IMU_BASELINE = 0.30 
#         self.current_pitch = 0.0 # Initialize pitch

#         # 2. SUBSCRIBER
#         # Use Sensor Data QoS (Best Effort) to ensure connection
#         from rclpy.qos import qos_profile_sensor_data
#         self.create_subscription(Imu, target_topic, self.diff_cb, qos_profile_sensor_data)

#         # 3. PUBLISHER
#         self.odom_pub = self.create_publisher(Odometry, '/odometry/ekf', 50)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         self.get_logger().info(f"✅ Hybrid EKF (IMU+GPS Ready) Listening on '{target_topic}'")

#     def diff_cb(self, msg: Imu):
#         # Time calc
#         current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
#         if self.last_imu_time is None:
#             self.last_imu_time = current_time
#             return
#         dt = current_time - self.last_imu_time
#         self.last_imu_time = current_time
#         if dt <= 0: return

#         # Unpack Data
#         omega_z = msg.orientation.x          
#         omega_y = msg.orientation.y          
#         raw_accel_x = msg.orientation_covariance[0] 
#         raw_accel_z = msg.orientation_covariance[1] 
        
#         # --- 1. POPULATE HISTORY (For Jitter Check) ---
#         self.accel_history.append(raw_accel_x)
#         self.gyro_history.append(omega_z)

#         # --- 2. COMPLEMENTARY FILTER (Pitch) ---
#         pitch_accel = np.arctan2(raw_accel_x, raw_accel_z)
        
#         # Adaptive Gain
#         g_force = math.sqrt(raw_accel_x**2 + raw_accel_z**2)
#         alpha = 0.02
#         if abs(g_force - 9.81) > 0.5: alpha = 0.005 

#         self.current_pitch = (1.0 - alpha) * (self.current_pitch + omega_y * dt) + \
#                              (alpha) * (pitch_accel)

#         # --- 3. GRAVITY COMPENSATION ---
#         gravity_effect = 9.81 * sin(self.current_pitch)
#         corrected_accel_x = raw_accel_x - gravity_effect 

#         # --- 4. STATIONARY CHECK ---
#         is_stationary = False
#         current_vel = self.ekf.x[3]
        
#         if abs(current_vel) < 1.0 and len(self.accel_history) == 20:
#             acc_var = np.var(self.accel_history)
#             gyro_var = np.var(self.gyro_history)
#             if acc_var < 0.005 and gyro_var < 0.0005:
#                 is_stationary = True

#         # --- 5. MOTION LOGIC ---
#         if is_stationary:
#             # Lock everything
#             self.ekf.x[3] = 0.0 # Force Velocity 0
#             omega_z = 0.0       # Force Yaw Rate 0 (Prevent spin)
#             corrected_accel_x = 0.0
#         else:
#             # Simple noise gate
#             if abs(corrected_accel_x) < 0.05:
#                 corrected_accel_x = 0.0
            
#             # Note: We NO LONGER manually integrate velocity here.
#             # The EKF.predict function handles it using 'corrected_accel_x'

#         # --- 6. PREDICT STEP ---
#         # FIX 1: Pass 'accel' and remove 'alpha'
#         self.ekf.predict(omega=omega_z, accel=corrected_accel_x, dt=dt)

#         # --- 7. PUBLISH ---
#         # FIX 2: Pass 'omega_z' explicitly because it's not in the state vector anymore
#         self.publish_odom(msg.header.stamp, omega_z)

#     def publish_odom(self, stamp, omega_z):
#         state = self.ekf.get_state()
        
#         # FIX 3: Unpack only 4 values (Hybrid EKF standard)
#         x, y, yaw, v = state 

#         t = TransformStamped()
#         t.header.stamp = stamp
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_link'
#         t.transform.translation.x = float(x)
#         t.transform.translation.y = float(y)
#         t.transform.rotation.z = sin(yaw / 2.0)
#         t.transform.rotation.w = cos(yaw / 2.0)
#         self.tf_broadcaster.sendTransform(t)

#         odom = Odometry()
#         odom.header.stamp = stamp
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id = 'base_link'
#         odom.pose.pose.position.x = float(x)
#         odom.pose.pose.position.y = float(y)
#         odom.pose.pose.orientation = t.transform.rotation
#         odom.twist.twist.linear.x = float(v)
        
#         # FIX 4: Use the omega_z passed from diff_cb (which might be 0.0 if stopped)
#         odom.twist.twist.angular.z = float(omega_z) 
        
#         self.odom_pub.publish(odom)

# def main(args=None):
#     rclpy.init(args=args)
#     node = EKFNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()