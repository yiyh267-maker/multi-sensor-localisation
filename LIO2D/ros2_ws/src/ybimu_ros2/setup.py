from setuptools import find_packages, setup

package_name = 'ybimu_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nalyence',
    maintainer_email='nalyence@example.com',
    description='YB IMU ROS2 publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_publisher = ybimu_ros2.imu_publisher:main',
        ],
    },
)
