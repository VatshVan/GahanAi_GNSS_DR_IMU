import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'state_estimator'

    return LaunchDescription([
        
        # 1. IMU BRIDGE
        Node(
            package=pkg_name,
            executable='imu_bridge',
            name='imu_bridge_node',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB1', 'baud': 115200}]
        ),

        # 2. GNSS BRIDGE
        Node(
            package=pkg_name,
            executable='gnss_node',
            name='gnss_node',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB0', 'baud': 115200}]
        ),

        # 3. EKF NODE
        Node(
            package=pkg_name,
            executable='ekf_node',
            name='ekf_node',
            output='screen'
        ),

        # 4. TF TRANSFORMS
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'imu_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.1', '0', '0.3', '0', '0', '0', 'base_link', 'gnss_link']
        )
    ])