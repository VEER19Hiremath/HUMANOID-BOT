from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='hospital_delivery',
            executable='voice_delivery_node',
            output='screen'
        )
    ])