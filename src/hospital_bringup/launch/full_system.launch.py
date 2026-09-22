from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')

    bringup_dir = get_package_share_directory('hospital_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser('~/hospital_robot_ws/maps/hospital_map.yaml')
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'navigation.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_file
            }.items()
        ),

        Node(
            package='hospital_delivery',
            executable='delivery_node',
            name='delivery_node',
            output='screen'
        ),

        Node(
            package='hospital_delivery',
            executable='voice_delivery_node',
            name='voice_delivery_node',
            output='screen'
        )
    ])