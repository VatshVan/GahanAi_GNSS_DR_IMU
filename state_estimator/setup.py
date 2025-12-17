from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'state_estimator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sedrica',
    maintainer_email='sedrica@todo.todo',
    description='state estimator package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # executable_name = package_name.script_name:main
            'ekf_node = state_estimator.ekf_ros_node:main',
            'gnss_node = state_estimator.gps_bridge:main',
            'imu_bridge = state_estimator.imu_bridge:main',
        ],
    },
)
