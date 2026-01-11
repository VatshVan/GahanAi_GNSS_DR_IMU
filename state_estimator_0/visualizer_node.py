#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from collections import deque
import threading
import numpy as np

class VehicleVisualizer3D(Node):
    def __init__(self):
        super().__init__('vehicle_visualizer_3d')
        
        # --- CONFIGURATION ---
        self.HISTORY_LEN = 1000  # Keep last 1000 points
        
        # --- DATA STORAGE ---
        self.x_data = deque(maxlen=self.HISTORY_LEN)
        self.y_data = deque(maxlen=self.HISTORY_LEN)
        self.z_data = deque(maxlen=self.HISTORY_LEN)
        
        # State for Arrow/Marker
        self.cur_x = 0.0; self.cur_y = 0.0; self.cur_z = 0.0
        self.cur_u = 1.0; self.cur_v = 0.0; self.cur_w = 0.0
        
        # --- SUBSCRIBER ---
        self.create_subscription(Odometry, '/odometry/ekf', self.odom_callback, 10)
        self.get_logger().info("3D Visualizer Started. Waiting for Data...")

    def odom_callback(self, msg):
        # 1. Position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # 2. Orientation (Quaternion -> Direction Vector)
        q = msg.pose.pose.orientation
        # Rotate [1,0,0] by quaternion to get forward vector
        ux = 1 - 2 * (q.y**2 + q.z**2)
        uy = 2 * (q.x*q.y + q.w*q.z)
        uz = 2 * (q.x*q.z - q.w*q.y)

        # 3. Store Data
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)
        
        # 4. Update Current State
        self.cur_x, self.cur_y, self.cur_z = x, y, z
        self.cur_u, self.cur_v, self.cur_w = ux, uy, uz

# --- ANIMATION FUNCTION ---
def animate(i, node, ln_path, quiver, ax_3d):
    if len(node.x_data) == 0: return ln_path

    # --- UPDATE 3D TRAJECTORY ---
    ln_path.set_data(node.x_data, node.y_data)
    ln_path.set_3d_properties(node.z_data)
    
    # --- UPDATE ARROW ---
    global global_quiver
    if global_quiver: global_quiver.remove()
    
    global_quiver = ax_3d.quiver(
        node.cur_x, node.cur_y, node.cur_z,
        node.cur_u, node.cur_v, node.cur_w,
        length=2.0, color='red', linewidth=2
    )

    # --- CENTER CAMERA ---
    w = 75 # View window size (User Preference)
    ax_3d.set_xlim(node.cur_x - w, node.cur_x + w)
    ax_3d.set_ylim(node.cur_y - w, node.cur_y + w)
    ax_3d.set_zlim(node.cur_z - 2, node.cur_z + 8)

    return ln_path

global_quiver = None

def main():
    rclpy.init()
    node = VehicleVisualizer3D()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    # Create Figure (Single 3D Plot)
    fig = plt.figure(figsize=(10, 8))
    
    ax_3d = fig.add_subplot(1, 1, 1, projection='3d')
    ax_3d.set_title("3D Vehicle Trajectory")
    ax_3d.set_xlabel("X (East)")
    ax_3d.set_ylabel("Y (North)")
    ax_3d.set_zlabel("Z (Up)")
    
    # Initialize Line
    ln_path, = ax_3d.plot([], [], [], 'b-', lw=1, alpha=0.6)
    
    # Dummy arrow for initialization
    global global_quiver
    global_quiver = ax_3d.quiver(0,0,0, 1,0,0, length=1)

    # Start Animation
    ani = animation.FuncAnimation(
        fig, animate, fargs=(node, ln_path, global_quiver, ax_3d),
        interval=100, blit=False
    )

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()