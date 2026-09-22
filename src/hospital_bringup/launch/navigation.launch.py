from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")

    params_file = os.path.join(
        get_package_share_directory("hospital_bringup"),
        "config",
        "nav2_params.yaml"
    )

    nav2_launch = os.path.join(
        get_package_share_directory("nav2_bringup"),
        "launch",
        "navigation_launch.py"
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": params_file
            }.items()
        )
    ])