from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import TimerAction
from launch.conditions import IfCondition

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_port = LaunchConfiguration('lidar_port')
    arduino_port = LaunchConfiguration('arduino_port')
    odom_source = LaunchConfiguration('odom_source')

    description_pkg = get_package_share_directory('hospital_description')
    rplidar_pkg = get_package_share_directory('rplidar_ros')

    robot_urdf = os.path.join(description_pkg, 'urdf', 'robot.urdf')

    return LaunchDescription(
        [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # Keep the Arduino and LiDAR on separate defaults. Override these
        # arguments when udev assigns different device names.
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('arduino_port', default_value='/dev/ttyUSB0'),
        # 'encoder' or 'command' (dead reckoning from sent wheel speeds).
        DeclareLaunchArgument('odom_source', default_value='encoder'),
        # Seconds without motion before the Arduino port is closed; 0 = never.
        DeclareLaunchArgument('idle_close_s', default_value='0.0'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command(['cat ', robot_urdf])
            }]
        ),

        # Disable when AMCL publishes map->odom.
        DeclareLaunchArgument('static_map_odom', default_value='true'),

        Node(
            condition=IfCondition(LaunchConfiguration('static_map_odom')),
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['2.2', '1.6', '0.0',
                       '0.0', '0.0', '0.0',
                       'map', 'odom'],
            output='screen'
        ),

        Node(
            package='wheel_odometry',
            executable='base_controller',
            name='base_controller',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'arduino_port': arduino_port,
                'odom_source': odom_source,
                'idle_close_s': LaunchConfiguration('idle_close_s'),
            }]
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(rplidar_pkg, 'launch', 'rplidar_a2m8_launch.py')
        #     ),
        #     launch_arguments={
        #         'serial_port': lidar_port,
        #         'frame_id': 'laser'
        #     }.items()
        # ),

        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(rplidar_pkg, 'launch', 'rplidar_a2m8_launch.py')
                    ),
                    launch_arguments={
                        'serial_port': lidar_port,
                        'frame_id': 'laser'
                    }.items()
                )
            ]
        )
    ]
)