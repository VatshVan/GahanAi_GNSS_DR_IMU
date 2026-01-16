import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'state_estimator'
    config = os.path.join(get_package_share_directory(pkg_name), 'config', 'ekf_params.yaml')

    return LaunchDescription([
        Node(package=pkg_name, executable='imu_bridge', parameters=[config]),
        Node(package=pkg_name, executable='gps_bridge', parameters=[config]),
        Node(package=pkg_name, executable='ekf_node', parameters=[config]),
        
        # Static TF
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gnss_link']),
    ])