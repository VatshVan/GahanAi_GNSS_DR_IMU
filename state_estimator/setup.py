from setuptools import setup
import os
from glob import glob

package_name = 'state_estimator'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vatsh Van',
    maintainer_email='vatshvan.iitb@gmail.com',
    description='Professional EKF Pipeline',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ekf_node = state_estimator.ekf_ros_node:main',
            'gps_bridge = state_estimator.gps_bridge:main',
            'imu_bridge = state_estimator.imu_bridge:main',
        ],
    },
)