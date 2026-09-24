from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('lio2d_bringup')
    config_dir = os.path.join(pkg_dir, 'config')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'rplidar_imu_2d.lua'
        ],
        remappings=[
            ('scan', '/scan'),
            ('imu', '/imu/data_raw'),
        ]
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node
    ])
