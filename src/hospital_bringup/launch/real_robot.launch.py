"""Real robot hardware: robot model, base controller (Mega), lidar.

static_map_odom:=true (drawn maps): map->odom is fixed at the robot's start
pose (map_odom_x/y = the map's "# home:"); the robot must start there, facing
+x. false: AMCL (scanned maps) or slam_toolbox (mapping) publishes map->odom.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_port = LaunchConfiguration('lidar_port')
    start_lidar = LaunchConfiguration('start_lidar')
    start_base = LaunchConfiguration('start_base')
    arduino_port = LaunchConfiguration('arduino_port')
    static_map_odom = LaunchConfiguration('static_map_odom')
    map_odom_x = LaunchConfiguration('map_odom_x')
    map_odom_y = LaunchConfiguration('map_odom_y')

    desc_pkg = get_package_share_directory('hospital_description')
    rplidar_pkg = get_package_share_directory('rplidar_ros')
    bringup_dir = get_package_share_directory('hospital_bringup')
    urdf_path = os.path.join(desc_pkg, 'urdf', 'robot.urdf')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('arduino_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('static_map_odom', default_value='true'),
        DeclareLaunchArgument('map_odom_x', default_value='0.0'),
        DeclareLaunchArgument('map_odom_y', default_value='0.0'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(
                    Command(['cat ', urdf_path]), value_type=str),
            }],
        ),
        Node(
            condition=IfCondition(static_map_odom),
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['--x', map_odom_x, '--y', map_odom_y,
                       '--frame-id', 'map', '--child-frame-id', 'odom'],
        ),
        Node(
            condition=IfCondition(start_base),
            package='wheel_odometry',
            executable='base_controller',
            name='base_controller',
            output='screen',
            # odometry.yaml: metres per wheel pulse (scripts/floor_calibrate.py)
            parameters=[os.path.join(bringup_dir, 'config', 'odometry.yaml'), {
                'use_sim_time': use_sim_time,
                'arduino_port': arduino_port,
            }],
        ),
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(rplidar_pkg, 'launch', 'rplidar_a2m8_launch.py'),
                    ),
                    condition=IfCondition(start_lidar),
                    launch_arguments={
                        'serial_port': lidar_port,
                        'frame_id': 'laser',
                    }.items(),
                ),
            ],
        ),
    ])
