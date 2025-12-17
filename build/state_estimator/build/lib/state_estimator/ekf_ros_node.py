import rclpy
from rclpy.node import Node
import numpy as np
from math import sin, cos, degrees
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32  
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# --- MATH HELPER CLASS ---
class EKF:
    def __init__(self):
        # State Vector: [pos_x, pos_y, yaw_angle, velocity, yaw_rate]
        self.state = np.zeros(5) 

    def predict_state(self, accel_fwd, yaw_rate, dt):
        pos_x, pos_y, yaw, velocity, _ = self.state
        
        # 1. Integrate Yaw
        new_yaw = yaw + yaw_rate * dt
        
        # 2. Integrate Velocity (with Drag Factor)
        new_velocity = velocity + accel_fwd * dt
        new_velocity *= 0.99 # Simple Air Drag
        
        # 3. Safety Clamping (Max Speed 5 m/s)
        if new_velocity > 5.0: new_velocity = 5.0
        if new_velocity < -5.0: new_velocity = -5.0

        # 4. Integrate Position (Dead Reckoning)
        new_pos_x = pos_x + new_velocity * np.cos(new_yaw) * dt
        new_pos_y = pos_y + new_velocity * np.sin(new_yaw) * dt
        
        self.state = np.array([new_pos_x, new_pos_y, new_yaw, new_velocity, yaw_rate])
    
    def get_current_state(self): return self.state

# --- ROS NODE ---
class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node_diff_imu')
        
        # --- CONFIGURATION ---
        self.ROTATION_RADIUS = 0.15  # Meters (Distance from Pivot to Sensor)
        
        # --- STATE ---
        self.ekf = EKF()
        self.last_msg_time = None
        self.current_pitch = 0.0

        # --- SUBSCRIBERS ---
        self.sub_imu_diff = self.create_subscription(Imu, '/imu/diff', self.callback_imu_diff, 10)
        # Note: We still subscribe to Pitch separately because the standard IMU msg 
        # uses Quaternions, and decoding them back to Euler here is extra work.
        # Since the Bridge publishes both, this is fine for now.
        self.sub_pitch    = self.create_subscription(Float32, '/imu/pitch', self.callback_pitch, 10)
        
        # --- PUBLISHERS ---
        self.pub_odom = self.create_publisher(Odometry, '/odometry/ekf', 50)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("--- EKF NODE STARTED [Standardized Inputs] ---")

    def callback_pitch(self, msg): 
        self.current_pitch = msg.data

    def callback_imu_diff(self, msg: Imu):
        # 1. Time Delta Calculation
        current_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_msg_time is None:
            self.last_msg_time = current_time_sec; return
        dt = current_time_sec - self.last_msg_time
        self.last_msg_time = current_time_sec
        if dt <= 0: return

        # 2. Unpack Data (UPDATED TO STANDARD ROS FIELDS)
        # linear_acceleration.x -> Forward Accel (Average)
        # angular_velocity.z    -> Yaw Rate
        
        accel_fwd_raw = msg.linear_acceleration.x  # <--- CHANGED FROM covariance[0]
        yaw_rate      = msg.angular_velocity.z     # <--- CHANGED FROM orientation.x
        
        # 3. PHYSICS CORRECTION: Gravity Compensation
        # Bridge uses atan2(-Ax), effectively flipping sign. We ADD gravity to compensate.
        gravity_vector_x = 9.81 * sin(self.current_pitch) 
        accel_gravity_corrected = accel_fwd_raw + gravity_vector_x 

        # 4. PHYSICS CORRECTION: Centrifugal Force
        # Subtracting fake forward force caused by rotation: F = w^2 * r
        accel_centrifugal = (yaw_rate ** 2) * self.ROTATION_RADIUS
        accel_linear_net = accel_gravity_corrected - accel_centrifugal

        # 5. MOTION LOGIC (Zero Velocity Update & Friction)
        accel_final = accel_linear_net
        
        # Noise Gate: Ignore tiny accelerations
        if abs(accel_final) < 0.15: 
            accel_final = 0.0
            
            # Apply Friction to decay velocity when coasting
            current_vel = self.ekf.state[3]
            self.ekf.state[3] = current_vel * 0.8 

            if abs(self.ekf.state[3]) < 0.05:
                self.ekf.state[3] = 0.0

        # 6. UPDATE EKF STATE
        self.ekf.predict_state(accel_final, yaw_rate, dt)
        self.publish_odometry(msg.header.stamp)

        # --- DIAGNOSTIC LOGGING ---
        self.get_logger().info(
            f"PITCH: {degrees(self.current_pitch):.1f} deg | "
            f"RAW_ACC: {accel_fwd_raw:.2f} | "
            f"GRAV_COMP: {gravity_vector_x:.2f} | "
            f"NET_ACC: {accel_final:.2f}",
            throttle_duration_sec=0.2
        )

    def publish_odometry(self, stamp):
        state = self.ekf.get_current_state()
        pos_x, pos_y, yaw, vel, omega = state
        
        # 1. Broadcast Transform
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(pos_x)
        t.transform.translation.y = float(pos_y)
        t.transform.rotation.z = sin(yaw / 2.0)
        t.transform.rotation.w = cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)
        
        # 2. Publish Odometry Message
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(pos_x)
        odom.pose.pose.position.y = float(pos_y)
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = float(vel)
        odom.twist.twist.angular.z = float(omega)
        self.pub_odom.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()