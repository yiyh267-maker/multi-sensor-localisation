from setuptools import setup
import os
from glob import glob

package_name = 'lio2d_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nalyence',
    maintainer_email='nalyence@example.com',
    description='2D LiDAR + IMU Cartographer bringup',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
)
