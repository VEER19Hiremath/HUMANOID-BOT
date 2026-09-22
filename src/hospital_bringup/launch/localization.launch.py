from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    map_file = LaunchConfiguration("map")

    nav2_params = os.path.join(
        get_package_share_directory("hospital_bringup"),
        "config",
        "nav2_params.yaml"
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(
                os.path.expanduser("~/Desktop/veeresh/hospital_robot_ws"),
                "maps",
                "hospital_map.yaml"
            ),
            description="Path to map yaml"
        ),

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                {"yaml_filename": map_file},
                {"use_sim_time": False}
            ]
        ),

        # Node(
        #     package="nav2_amcl",
        #     executable="amcl",
        #     name="amcl",
        #     output="screen",
        #     parameters=[
        #         nav2_params,
        #         {"use_sim_time": False}
        #     ]
        # ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                {
                    "use_sim_time": False,
                    "autostart": True,
                    "node_names": [
                        "map_server"
                        # "amcl"
                    ]
                }
            ]
        )
    ])