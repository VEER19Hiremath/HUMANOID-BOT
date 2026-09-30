import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml

# (package, executable, component plugin, node name, extra remappings)
# Order: drive path first (controller → smoother), then planners/BT.
# route_server stays out: it / a late collision monitor often left
# velocity_smoother inactive, so cmd_vel_nav had no subscribers.
SERVERS = [
    ('nav2_controller', 'controller_server', 'nav2_controller::ControllerServer',
     'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
    ('nav2_smoother', 'smoother_server', 'nav2_smoother::SmootherServer',
     'smoother_server', []),
    ('nav2_planner', 'planner_server', 'nav2_planner::PlannerServer',
     'planner_server', []),
    ('nav2_behaviors', 'behavior_server', 'behavior_server::BehaviorServer',
     'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
    ('nav2_bt_navigator', 'bt_navigator', 'nav2_bt_navigator::BtNavigator',
     'bt_navigator', []),
    ('nav2_waypoint_follower', 'waypoint_follower',
     'nav2_waypoint_follower::WaypointFollower', 'waypoint_follower', []),
]
LIFECYCLE_ORDER = ['controller_server', 'velocity_smoother', 'planner_server',
                   'behavior_server', 'bt_navigator', 'waypoint_follower',
                   'smoother_server']


def launch_setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    collision = LaunchConfiguration('enable_collision_monitor').perform(context).lower() == 'true'
    composed = LaunchConfiguration('use_composition').perform(context).lower() == 'true'

    params = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration('params_file'),
            root_key=LaunchConfiguration('namespace'),
            param_rewrites={'autostart': autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )
    remap_tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    servers = list(SERVERS)
    if collision:
        # velocity_smoother → cmd_vel_smoothed → collision_monitor → cmd_vel
        servers.append(('nav2_velocity_smoother', 'velocity_smoother',
                        'nav2_velocity_smoother::VelocitySmoother',
                        'velocity_smoother', [('cmd_vel', 'cmd_vel_nav')]))
        servers.append(('nav2_collision_monitor', 'collision_monitor',
                        'nav2_collision_monitor::CollisionMonitor',
                        'collision_monitor', []))
    else:
        servers.append(('nav2_velocity_smoother', 'velocity_smoother',
                        'nav2_velocity_smoother::VelocitySmoother', 'velocity_smoother',
                        [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]))
    managed = LIFECYCLE_ORDER + (['collision_monitor'] if collision else [])
    manager_params = {'use_sim_time': use_sim_time, 'autostart': autostart,
                      'bond_timeout': 45.0, 'node_names': managed}

    if composed:
        # One process, one DDS participant: separate processes joining DDS at
        # the same moment left a random server hung at startup (bench
        # 2026-09-30: controller_server, then behavior_server), so Nav2 never
        # activated and nothing drove.
        nodes = [
            ComposableNode(package=pkg, plugin=plugin, name=name,
                           parameters=[params, {'use_sim_time': use_sim_time}],
                           remappings=remap_tf + extra)
            for pkg, _, plugin, name, extra in servers
        ]
        nodes.append(ComposableNode(
            package='nav2_lifecycle_manager',
            plugin='nav2_lifecycle_manager::LifecycleManager',
            name='lifecycle_manager_navigation',
            parameters=[manager_params]))
        return [ComposableNodeContainer(
            name='nav2_container', namespace='',
            package='rclcpp_components', executable='component_container_isolated',
            composable_node_descriptions=nodes,
            # The costmaps are child nodes of planner/controller and only see
            # the file through the container: without this they ran on Nav2
            # defaults (radius 0.1 m, no footprint) and plans clipped walls.
            parameters=[params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            output='screen')]

    nodes = [
        Node(package=pkg, executable=exe, name=name, output='screen',
             parameters=[params, {'use_sim_time': use_sim_time}],
             arguments=['--ros-args', '--log-level', log_level],
             remappings=remap_tf + extra)
        for pkg, exe, _, name, extra in servers
    ]
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[manager_params]))
    return nodes


def generate_launch_description():
    bringup_dir = get_package_share_directory('hospital_bringup')
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
        ),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'enable_collision_monitor',
            default_value='false',
            description='true wires VS→CM→cmd_vel; false (default) VS→cmd_vel directly',
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='true',
            description='true: all Nav2 servers in one container process',
        ),
        OpaqueFunction(function=launch_setup),
    ])
