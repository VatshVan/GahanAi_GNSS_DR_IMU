# import os
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     pkg_share = get_package_share_directory('state_estimator')
#     ekf_config = os.path.join(pkg_share, 'config', 'ekf_params.yaml')

#     return LaunchDescription([
#         Node(
#             package='vehicle_interface',
#             executable='gps_ros_node',
#             name='gps_node',
#             output='screen',
#             parameters=[{
#                 'port': '/dev/ttyUSB0',   # adjust device if needed
#                 'baud': 115200,
#             }],
#         ),

#         Node(
#             package='vehicle_interface',
#             executable='imu_ros_node',
#             name='imu_node',
#             output='screen',
#             parameters=[{
#                 'port': '/dev/ttyACM0',   # adjust device if needed
#                 'baud': 115200,
#             }],
#         ),

#         Node(
#             package='state_estimator',
#             executable='ekf_node',
#             name='ekf_node',
#             output='screen',
#             parameters=[ekf_config],
#         ),
#     ])


# import os
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     pkg_share = get_package_share_directory('state_estimator')
#     ekf_config = os.path.join(pkg_share, 'config', 'ekf_params.yaml')

#     return LaunchDescription([
#         # --- BLOCKS REMOVED: We are running sensors manually for now ---
#         # Node(package='vehicle_interface', executable='gps_ros_node', ...),
#         # Node(package='vehicle_interface', executable='imu_ros_node', ...),
        
#         # --- KEEPING ONLY THE BRAIN (EKF) ---
#         Node(
#             package='state_estimator',
#             executable='ekf_node',  # Ensure this matches entry_points in setup.py
#             name='ekf_node',
#             output='screen',
#             parameters=[ekf_config],
#         ),
#     ])

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('state_estimator')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf_params.yaml')

    return LaunchDescription([
        # 1. IMU MOUNTING (e.g., Center of car)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='imu_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
            output='screen'
        ),

        # 2. GPS ANTENNA MOUNTING (e.g., On the roof, 20cm behind center)
        # Change the first number (-0.2) to match your physical setup!
        # Node(
        #     package='tf2_ros', executable='static_transform_publisher',
        #     name='gps_tf',
        #     arguments=['-0.2', '0', '0.5', '0', '0', '0', 'base_link', 'gps_link'],
        #     output='screen'
        # ),

        # 3. THE BRAIN (EKF)
        Node(
            package='state_estimator',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[ekf_config],
        ),
    ])