#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading
import numpy as np

class VehicleVisualizer(Node):
    def __init__(self):
        super().__init__('vehicle_visualizer')
        
        # --- CONFIGURATION ---
        self.HISTORY_LEN = 1000  # Keep last 1000 points (prevent lag)
        
        # --- DATA STORAGE ---
        self.x_data = deque(maxlen=self.HISTORY_LEN)
        self.y_data = deque(maxlen=self.HISTORY_LEN)
        self.v_data = deque(maxlen=self.HISTORY_LEN)
        self.t_data = deque(maxlen=self.HISTORY_LEN)
        
        self.start_time = None

        # --- SUBSCRIBER ---
        # We listen to the EKF output
        self.create_subscription(Odometry, '/odometry/ekf', self.odom_callback, 10)
        
        self.get_logger().info("Visualizer Started. Waiting for EKF data...")

    def odom_callback(self, msg):
        # 1. Get Time
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = now
        
        elapsed = now - self.start_time
        
        # 2. Get Position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # 3. Get Velocity (Linear X is forward speed)
        v = msg.twist.twist.linear.x * 3.6 # Convert m/s to km/h

        # 4. Store
        self.t_data.append(elapsed)
        self.x_data.append(x)
        self.y_data.append(y)
        self.v_data.append(v)

# --- PLOTTING FUNCTION ---
def animate(i, node, line_path, line_speed, ax_map, ax_speed):
    if len(node.x_data) == 0:
        return line_path, line_speed

    # 1. Update Path (Map)
    line_path.set_data(node.x_data, node.y_data)
    
    # Adjust Map Limits dynamically
    ax_map.set_xlim(min(node.x_data)-5, max(node.x_data)+5)
    ax_map.set_ylim(min(node.y_data)-5, max(node.y_data)+5)

    # 2. Update Speed Graph
    line_speed.set_data(node.t_data, node.v_data)
    
    # Adjust Speed Limits
    if len(node.t_data) > 0:
        ax_speed.set_xlim(min(node.t_data), max(node.t_data) + 1)
        ax_speed.set_ylim(0, max(max(node.v_data), 10) + 2) # Auto-scale Y, min 10 km/h

    return line_path, line_speed

def main():
    # 1. Start ROS in a separate thread (so it doesn't block the GUI)
    rclpy.init()
    node = VehicleVisualizer()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    # 2. Setup Matplotlib
    fig, (ax_map, ax_speed) = plt.subplots(2, 1, figsize=(8, 10))
    fig.suptitle("Vehicle Live Dashboard")

    # --- TOP PLOT: TRAJECTORY ---
    ax_map.set_title("Live Position (GPS/EKF Path)")
    ax_map.set_xlabel("East (m)")
    ax_map.set_ylabel("North (m)")
    ax_map.grid(True)
    ax_map.axis('equal') # Keep aspect ratio square so map looks real
    line_path, = ax_map.plot([], [], 'b-', lw=2, label='Trajectory')
    ax_map.legend()

    # --- BOTTOM PLOT: SPEED ---
    ax_speed.set_title("Speed Profile")
    ax_speed.set_xlabel("Time (s)")
    ax_speed.set_ylabel("Speed (km/h)")
    ax_speed.grid(True)
    line_speed, = ax_speed.plot([], [], 'r-', lw=2, label='Speed')

    # 3. Start Animation Loop (Updates every 100ms)
    ani = animation.FuncAnimation(
        fig, animate, fargs=(node, line_path, line_speed, ax_map, ax_speed),
        interval=100, blit=False
    )

    # 4. Show Window
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()