from setuptools import find_packages, setup

package_name = 'state_estimator'

setup(
    name=package_name,
    version='0.0.0',
    # 1. TELL PYTHON TO LOOK INSIDE 'src'
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ... your launch files ...
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vatshvan',
    maintainer_email='vatshvan.iitb@gmail.com',
    description='EKF State Estimator',
    license='Proprietary - All Rights Reserved',
    
    # 2. ENSURE ENTRY POINTS ARE CORRECT
    entry_points={
        'console_scripts': [
            'ekf_node = state_estimator.ekf_ros_node:main',
            'gnss_node = state_estimator.gps_bridge:main',
            'imu_bridge = state_estimator.imu_bridge:main',
        ],
    },
)