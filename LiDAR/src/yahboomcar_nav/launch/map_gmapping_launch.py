from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource,
from ament_index_python.packages import get_package_share_directory
from launch.conditions import LaunchConfigurationEquals,LaunchConfigurationNotEquals
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    RPLIDAR_TYPE = os.getenv('LIDAR_TYPE')
    rplidar_type_arg = DeclareLaunchArgument(name='rplidar_type', default_value=RPLIDAR_TYPE, 
                                              description='The type of robot')


    gmapping_s2_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch'),
        '/map_gmapping_4ros_s2_launch.py']),
        condition=LaunchConfigurationEquals('rplidar_type', 's2')
    )
    gmapping_a1_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch'),
        '/map_gmapping_a1_launch.py']),
        condition=LaunchConfigurationNotEquals('rplidar_type', 's2')
    )

    return LaunchDescription([
        rplidar_type_arg,
        gmapping_s2_launch, 
        gmapping_a1_launch
    ])
