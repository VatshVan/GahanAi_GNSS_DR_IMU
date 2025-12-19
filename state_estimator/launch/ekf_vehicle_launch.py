"""
ekf_vehicle_launch.py
High-level launch description for vehicle state estimation stack.
Overview
--------
This module defines a reproducible, production-grade ROS 2 launch description that
brings up a minimal set of nodes required for GNSS/IMU-aided vehicle state estimation
using an Extended Kalman Filter (EKF). The launch file is intentionally small and
opinionated to maximize reliability and long-term maintainability.
Provided runtime composition
- imu_bridge   : Hardware bridge for inertial measurement unit (IMU) serial device.
- gnss_node    : Hardware bridge for GNSS/RTK receiver serial device.
- ekf_node     : Extended Kalman Filter node consuming IMU/GNSS inputs and producing
                 fused vehicle pose/velocity/attitude estimates.
- static transforms (tf2_ros/static_transform_publisher):
                 Two static transforms that define sensor frames with respect to
                 the vehicle base frame:
                 * base_link -> imu_link  (default: x=0, y=0, z=0.2, rpy=0,0,0)
                 * base_link -> gnss_link (default: x=0.1, y=0, z=0.3, rpy=0,0,0)
Goals and design principles
---------------------------
- Clarity: explicit nodes and parameters make behavior easy to reason about.
- Reproducibility: default serial ports and baud rates are provided; intended to
  be overridden via launch arguments or ROS parameter overrides in deployment.
- Extensibility: small, well-documented entrypoint so operators can add logging,
  remappings, lifecycle management, or replacement nodes without ambiguity.
- Longevity: documentation and naming emphasize stable concepts (IMU, GNSS, EKF,
  base_link) rather than ephemeral implementation details.
Configuration and Parameters
----------------------------
This launch description currently exposes the following configurable values (set
directly in the Node.parameters arrays or intended to be replaced by launch
arguments in downstream deployments):
- IMU bridge parameters:
    - port (string) : serial device path (default: '/dev/ttyUSB1')
    - baud (int)    : serial baud rate (default: 115200)
- GNSS bridge parameters:
    - port (string) : serial device path (default: '/dev/ttyUSB0')
    - baud (int)    : serial baud rate (default: 115200)
- TF static transforms:
    - base_link -> imu_link:
        translation = [0.0, 0.0, 0.2]
        rotation (rpy) = [0.0, 0.0, 0.0]
    - base_link -> gnss_link:
        translation = [0.1, 0.0, 0.3]
        rotation (rpy) = [0.0, 0.0, 0.0]
Recommended launch-time overrides
--------------------------------
In production deployments you should:
- Replace literal parameter dictionaries with LaunchArguments so ports and baud
  rates can be overridden at runtime.
- Use namespacing or remapping to avoid conflicts when running multiple vehicles
  or simulation/test instances on the same ROS graph.
- Consider using lifecycle-managed nodes (if nodes support it) to ensure orderly
  bring-up and bring-down semantics.
Expected message contracts (conventions)
---------------------------------------
The code assumes conventional ROS message types and semantics for sensor/estimator
interfaces. For robust integration, ensure your IMU and GNSS bridge nodes adhere
to the following or provide appropriate remapping/adapters:
- IMU bridge should publish:
    - sensor_msgs/Imu on a well-known topic (e.g., /imu or under the node's namespace).
    - Properly filled header.stamp and header.frame_id matching imu_link.
- GNSS bridge should publish one or more of:
    - sensor_msgs/NavSatFix for raw GNSS fixes; and/or
    - nav_msgs/Odometry or a custom fused fix for position in a local frame.
    - When using raw GNSS, the EKF or an intermediary node must transform lat/lon
      into the estimator's map/local frame or provide RTK-corrected local ENU.
- EKF node should:
    - Subscribe to IMU and GNSS topics and publish fused pose/velocity (e.g.,
      sensor_msgs/Imu, nav_msgs/Odometry, geometry_msgs/PoseStamped, or TF).
    - Respect coordinate frame conventions: base_link is vehicle body frame,
      imu_link and gnss_link are sensor frames as published in the static transforms.
Best practices for production use
---------------------------------
- Parameterization: move hard-coded values into parameter files or a centralized
  config management system (YAML) and refer to them from the launch file.
- Health monitoring: add watchdogs or diagnostics nodes to monitor serial health,
  sensor update rates, and EKF convergence. Emit diagnostic_msgs/DiagnosticArray .
- Logging: route important node output to files and/or an aggregator (e.g., rosbag,
  logrotate) and include timestamps and node names.
- Determinism: ensure nodes publish signed timestamps (e.g., device timestamps) or
  synchronize using PTP/chrony in distributed systems.
- Security: protect serial devices and prohibit unexpected access. Use ROS 2 DDS
  security features (SROS2) when operating over untrusted networks.
Extensibility notes
-------------------
- Replacing static transforms: for setups with moving sensors or calibration
  rigs, swap static_transform_publisher with a transform_broadcaster node that
  reads calibrations from parameters or a central TF tree manager.
- Multiple sensors: to add a second IMU or GNSS receiver, instantiate additional
  bridge nodes with unique names and remap their output topics and frames.
- Simulation: to run in simulation, replace the bridge nodes with simulator bridges
  (e.g., ros_gz_bridge or Gazebo plugins) and ensure topics/message types match.
Compatibility
-------------
- This launch file uses the ROS 2 launch system (launch and launch_ros APIs) and
  the tf2_ros static_transform_publisher executable. It is compatible with ROS 2
  distributions that provide these packages. Test and pin to specific
  distributions in CI for long-term stability.
Operational considerations
--------------------------
- Device permissions: ensure the user running the launch has read/write access
  to the configured serial device files (e.g., /dev/ttyUSB0, /dev/ttyUSB1).
- Resource limits: serial devices and EKF computations must meet real-time or
  soft real-time constraints; monitor CPU and IO utilization.
- Graceful shutdown: wrap this launch in higher-level tooling that handles
  orchestration, health checks, and graceful node shutdown to avoid device lockups.
Maintenance and testing
-----------------------
- Unit tests: add tests that verify the launch composition using the ros2launch
  testing utilities to detect regressions when changing node names, parameters,
  or TF arguments.
- Integration tests: include hardware-in-the-loop (HIL) or hardware-available CI
  jobs that validate serial port handling, message rates, and EKF output fidelity.
- Documentation: keep README and changelog entries that document any changes to
  default ports, frame offsets, and message contracts.
Metadata
--------
- Stability        : Production-ready template (requires environment-specific
                     configuration for deployment)
Version history
---------------
- 1.0.0            : Initial production-grade launch template providing IMU,
                     GNSS, EKF and static TFs with clear extension points.
Usage examples
--------------
- Quick start (local override via environment or command line):
    ros2 launch state_estimator ekf_vehicle_launch.py
  (Recommend creating LaunchArguments or YAML parameter files to override
   default serial ports/baud rates in production.)
Notes
-----
Replace the hard-coded parameter dictionaries with LaunchArguments or external
YAML parameter files for any long-lived deployment. The purpose of this module
is to serve as a clear, auditable, and easily-extendable composition of the
vehicle's core sensing and estimation components.

"""
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