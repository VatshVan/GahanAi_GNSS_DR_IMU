#!/usr/bin/env python3
"""
ekf_stress_validator.py
======================

Advanced EKF validation & stress-test analysis tool.

Collects:
- Ground truth
- EKF odometry
- GPS
- LiDAR odom
- Wheel odom
- IMU yaw rate

Computes:
- RMSE / MAE / P95 / Max error
- Drift rate
- Velocity error
- Statistical tests
- Error distributions

Generates:
- CSV log
- Full plots
- Console report
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import stats

# =======================================================

class EKFStressValidator(Node):

    def __init__(self):
        super().__init__('ekf_stress_validator')

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.truth = []
        self.ekf = []
        self.gps = []
        self.lidar = []
        self.wheel = []
        self.imu = []

        self.create_subscription(Odometry, '/ground_truth', self.cb_truth, qos)
        self.create_subscription(Odometry, '/odometry/ekf', self.cb_ekf, qos)
        self.create_subscription(PoseWithCovarianceStamped, '/gps/enu_pose', self.cb_gps, qos)
        self.create_subscription(Odometry, '/lidar_odom', self.cb_lidar, qos)
        self.create_subscription(Odometry, '/wheel_odom', self.cb_wheel, qos)
        self.create_subscription(Imu, '/imu/data_raw', self.cb_imu, qos)

        self.get_logger().info("📡 EKF Stress Validator running – collecting data...")

    # -------------------------

    def stamp(self, msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def cb_truth(self, msg):
        self.truth.append(dict(t=self.stamp(msg),
                               x=msg.pose.pose.position.x,
                               y=msg.pose.pose.position.y,
                               v=msg.twist.twist.linear.x))

    def cb_ekf(self, msg):
        self.ekf.append(dict(t=self.stamp(msg),
                             x=msg.pose.pose.position.x,
                             y=msg.pose.pose.position.y,
                             v=msg.twist.twist.linear.x))

    def cb_gps(self, msg):
        self.gps.append(dict(t=self.stamp(msg),
                             x=msg.pose.pose.position.x,
                             y=msg.pose.pose.position.y))

    def cb_lidar(self, msg):
        self.lidar.append(dict(t=self.stamp(msg),
                               v=msg.twist.twist.linear.x,
                               w=msg.twist.twist.angular.z))

    def cb_wheel(self, msg):
        self.wheel.append(dict(t=self.stamp(msg),
                               v=msg.twist.twist.linear.x))

    def cb_imu(self, msg):
        self.imu.append(dict(t=self.stamp(msg),
                             wz=msg.angular_velocity.z))

# =======================================================

def analyze(node: EKFStressValidator):

    print("\n🧠 ANALYZING EKF PERFORMANCE ...\n")

    if len(node.truth) < 100 or len(node.ekf) < 100:
        print("❌ Not enough data collected.")
        return

    df_truth = pd.DataFrame(node.truth).sort_values("t")
    df_ekf = pd.DataFrame(node.ekf).sort_values("t")
    df_gps = pd.DataFrame(node.gps)
    df_lidar = pd.DataFrame(node.lidar)
    df_wheel = pd.DataFrame(node.wheel)
    df_imu = pd.DataFrame(node.imu)

    df = pd.merge_asof(df_ekf, df_truth, on='t', suffixes=('_ekf', '_true'), direction='nearest')

    df['ex'] = df.x_ekf - df.x_true
    df['ey'] = df.y_ekf - df.y_true
    df['pos_err'] = np.sqrt(df.ex**2 + df.ey**2)
    df['vel_err'] = df.v_ekf - df.v_true

    duration = df.t.iloc[-1] - df.t.iloc[0]

    rmse = np.sqrt(np.mean(df.pos_err**2))
    mae = np.mean(np.abs(df.pos_err))
    p95 = np.percentile(df.pos_err, 95)
    maxe = np.max(df.pos_err)

    drift_rate = (df.pos_err.iloc[-1] - df.pos_err.iloc[0]) / duration

    vel_rmse = np.sqrt(np.mean(df.vel_err**2))

    print("==========================================")
    print(" EKF STRESS TEST REPORT")
    print("==========================================")
    print(f"Duration           : {duration:.2f} s")
    print(f"Samples            : {len(df)}")
    print(f"RMSE (position)    : {rmse:.3f} m")
    print(f"MAE                : {mae:.3f} m")
    print(f"95% error          : {p95:.3f} m")
    print(f"Max error          : {maxe:.3f} m")
    print(f"Drift rate         : {drift_rate:.4f} m/s")
    print(f"Velocity RMSE      : {vel_rmse:.3f} m/s")

    # -------- statistical tests --------

    shapiro_p = stats.shapiro(df.pos_err.sample(min(5000, len(df))))[1]
    bias_x = np.mean(df.ex)
    bias_y = np.mean(df.ey)

    print("\n--- Statistical checks ---")
    print(f"Zero-mean bias X: {bias_x:.4f} m")
    print(f"Zero-mean bias Y: {bias_y:.4f} m")
    print(f"Normality p-val : {shapiro_p:.4f}")

    # -------- save csv --------

    df.to_csv("ekf_stress_log.csv", index=False)
    print("\n💾 Saved ekf_stress_log.csv")

    # -------- plots --------

    plt.figure(figsize=(14, 10))

    # trajectory
    plt.subplot(2,3,1)
    plt.plot(df_truth.x, df_truth.y, 'k', label='Truth')
    plt.plot(df_ekf.x, df_ekf.y, 'b--', label='EKF')
    if not df_gps.empty:
        plt.scatter(df_gps.x, df_gps.y, s=6, c='r', alpha=0.3, label='GPS')
    plt.title("Trajectory")
    plt.axis('equal')
    plt.legend()
    plt.grid()

    # error vs time
    plt.subplot(2,3,2)
    plt.plot(df.t, df.pos_err)
    plt.axhline(rmse, color='r', linestyle=':')
    plt.title("Position Error vs Time")
    plt.grid()

    # velocity
    plt.subplot(2,3,3)
    plt.plot(df.t, df.v_true, label="True")
    plt.plot(df.t, df.v_ekf, '--', label="EKF")
    plt.title("Velocity Tracking")
    plt.legend()
    plt.grid()

    # histogram
    plt.subplot(2,3,4)
    plt.hist(df.pos_err, bins=50, density=True)
    plt.title("Error Distribution")

    # CDF
    plt.subplot(2,3,5)
    sorted_err = np.sort(df.pos_err)
    plt.plot(sorted_err, np.linspace(0,1,len(sorted_err)))
    plt.title("Error CDF")
    plt.grid()

    # axis errors
    plt.subplot(2,3,6)
    plt.plot(df.t, df.ex, label="X")
    plt.plot(df.t, df.ey, label="Y")
    plt.title("Axis Errors")
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.savefig("ekf_stress_report.png", dpi=200)
    plt.show()

    print("📊 Saved ekf_stress_report.png")

# =======================================================

def main(args=None):
    rclpy.init(args=args)
    node = EKFStressValidator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    analyze(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
