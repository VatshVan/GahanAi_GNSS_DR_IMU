import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('state_estimator')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf_params.yaml')

    return LaunchDescription([
        Node(
            package='vehicle_interface',
            executable='gps_ros_node',
            name='gps_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',   # adjust device if needed
                'baud': 115200,
            }],
        ),

        Node(
            package='vehicle_interface',
            executable='imu_ros_node',
            name='imu_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM0',   # adjust device if needed
                'baud': 115200,
            }],
        ),

        Node(
            package='state_estimator',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[ekf_config],
        ),
    ])
