from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution([
        FindPackageShare('hospital_world'),
        'worlds',
        'hospital.world',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'gz_args',
            default_value=['-r -s ', world],
            description='Arguments passed to Gazebo Sim',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('ros_gz_sim'),
                '/launch/gz_sim.launch.py',
            ]),
            launch_arguments={
                'gz_args': LaunchConfiguration('gz_args'),
            }.items(),
        ),
    ])
