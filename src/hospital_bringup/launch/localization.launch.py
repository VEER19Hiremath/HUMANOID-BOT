from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    map_file = LaunchConfiguration("map")
    # false for a drawn map (no real walls to match): real_robot's static
    # map->odom places the robot at its start point instead.
    use_amcl = LaunchConfiguration("use_amcl")

    nav2_params = os.path.join(
        get_package_share_directory("hospital_bringup"),
        "config",
        "nav2_params.yaml"
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(
                os.path.expanduser("~/Desktop/veeresh"),
                "maps",
                "hospital_map.yaml"
            ),
            description="Path to map yaml"
        ),
        DeclareLaunchArgument("use_amcl", default_value="true"),

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

        # Lidar localization: corrects the drifting wheel/command odometry so
        # the pose stays on the map. Starts at home (initial_pose in
        # nav2_params.yaml); start.sh runs real_robot with static_map_odom:=false.
        Node(
            condition=IfCondition(use_amcl),
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[
                nav2_params,
                {"use_sim_time": False}
            ]
        ),

        Node(
            condition=IfCondition(use_amcl),
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                {
                    "use_sim_time": False,
                    "autostart": True,
                    "bond_timeout": 45.0,
                    "node_names": [
                        "map_server",
                        "amcl"
                    ]
                }
            ]
        ),
        Node(
            condition=UnlessCondition(use_amcl),
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                {
                    "use_sim_time": False,
                    "autostart": True,
                    "bond_timeout": 45.0,
                    "node_names": [
                        "map_server"
                    ]
                }
            ]
        )
    ])