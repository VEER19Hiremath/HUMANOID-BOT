from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():

    world = os.path.expanduser(
        '~/hospital_robot_ws/src/hospital_world/worlds/hospital.world'
    )

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot
    ])