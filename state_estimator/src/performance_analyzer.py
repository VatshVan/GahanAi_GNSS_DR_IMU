import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np

class EKFAnalyzer(Node):
    def __init__(self):
        super().__init__('performance_analyzer')
        self.sub_ekf = self.create_subscription(Odometry, '/odometry/ekf', self.cb_ekf, 10)
        self.sub_true = self.create_subscription(PoseStamped, '/ground_truth', self.cb_true, 10)
        
        self.ekf_data = {'x': [], 'y': [], 'v': [], 't': []}
        self.true_data = {'x': [], 'y': [], 't': []}
        self.errors = []

        self.get_logger().info("Analyzer Started. Close plot window to stop.")

    def cb_ekf(self, msg):
        self.ekf_data['x'].append(msg.pose.pose.position.x)
        self.ekf_data['y'].append(msg.pose.pose.position.y)
        self.ekf_data['v'].append(msg.twist.twist.linear.x) # If your node publishes v
        self.ekf_data['t'].append(msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9)

    def cb_true(self, msg):
        self.true_data['x'].append(msg.pose.position.x)
        self.true_data['y'].append(msg.pose.position.y)
        self.true_data['t'].append(msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9)
        
        # Calculate Euclidean Error if EKF data exists
        if self.ekf_data['x']:
            err = np.sqrt((msg.pose.position.x - self.ekf_data['x'][-1])**2 + 
                          (msg.pose.position.y - self.ekf_data['y'][-1])**2)
            self.errors.append(err)

    def plot_results(self):
        plt.figure(figsize=(12, 5))
        
        # Trajectory Plot
        plt.subplot(1, 2, 1)
        plt.plot(self.true_data['x'], self.true_data['y'], 'g-', label='Ground Truth')
        plt.plot(self.ekf_data['x'], self.ekf_data['y'], 'r--', label='EKF Estimate')
        plt.title('Trajectory Comparison')
        plt.legend()

        # Error Plot
        plt.subplot(1, 2, 2)
        plt.plot(self.errors, 'r')
        plt.title('Euclidean Position Error (m)')
        plt.ylabel('Meters')
        
        plt.tight_layout()
        plt.show()

def main():
    rclpy.init()
    node = EKFAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.plot_results()
    finally:
        rclpy.shutdown()