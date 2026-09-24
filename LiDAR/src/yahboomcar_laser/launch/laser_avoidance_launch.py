from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('rplidar_ros'), 'launch'),
         '/lidar.launch.py'])
      )

    laser_avoidance_node = Node(
        package='yahboomcar_laser',
        executable='laser_avoidance',
        output = 'screen'
    )

    bringup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('yahboomcar_bringup'), 'launch'),
        '/yahboomcar_bringup_X3_launch.py'])
    )

    launch_description = LaunchDescription([
        lidar_node,
        laser_avoidance_node,
        bringup_node
        ]) 
    return launch_description

