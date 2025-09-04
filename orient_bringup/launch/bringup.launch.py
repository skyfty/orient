
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from orient_common.launch import ReplacePath
from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ParameterFile
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml, ReplaceString
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.actions import ExecuteProcess
from launch.conditions import UnlessCondition

from launch.event_handlers import OnProcessStart
from launch.events import Shutdown
from launch.actions import ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler, EmitEvent

def generate_launch_description():
    #=============================1.定位到包的地址=============================================================
    bringup_dir = get_package_share_directory('orient_bringup')
    description_dir = get_package_share_directory('orient_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    log_level = LaunchConfiguration('log_level')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)

    description_name = LaunchConfiguration('description', default=os.getenv('ORIENT_DESCRIPTION', 'kf2404'))
    orient_clientid = LaunchConfiguration('orient_clientid', default=os.getenv('ORIENT_CLIENTID', description_name))
    nav_graph_filename = LaunchConfiguration('nav_graph_file', default=os.path.join(bringup_dir, 'nav_graphs','0.yaml'))
    map_filename = LaunchConfiguration('map', default=os.path.join(bringup_dir, 'maps','reflector.yaml'))
 
    behaviors_dir = ReplacePath(name=description_name,path=description_dir,source_file='behaviors')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'orient_clientid': orient_clientid
    }
    
    param_path = ReplacePath(
        name=description_name,
        path=description_dir,
        source_file=os.path.join('params','params.yaml'))
    
    default_nav_bt_xml = ReplacePath(
        name=description_name,
        path=description_dir,
        source_file=os.path.join('behaviors','default.xml'))
    
    params_file = ReplaceString(
        source_file=param_path,
        replacements={'<robot_namespace>': ('/', namespace)},
        condition=IfCondition(use_namespace))

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    ld = LaunchDescription()

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    ld.add_action(stdout_linebuf_envvar)

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
    ld.add_action(declare_use_namespace_cmd)

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    ld.add_action(declare_use_respawn_cmd)

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    ld.add_action(declare_namespace_cmd)


    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')
    ld.add_action(declare_use_composition_cmd)

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    ld.add_action(declare_params_file_cmd)

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='orient_container',
        description='the name of conatiner that nodes will load in if use composition')
    ld.add_action(declare_container_name_cmd)

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')
    ld.add_action(declare_autostart_cmd)


    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    ld.add_action(declare_log_level_cmd)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(DeclareLaunchArgument(
        'description', default_value=description_name,
        description='description name'))
    
    ld.add_action(DeclareLaunchArgument(
        'map', default_value=map_filename,
        description='Full path to amcl map yaml file'))
    
    ld.add_action(DeclareLaunchArgument(
        'orient_clientid', default_value=orient_clientid,
        description='Orient client id'))

    declare_nav_graph_cmd = DeclareLaunchArgument(
        'nav_graph_file', default_value=nav_graph_filename,
        description='Full path to nav_graph yaml file')
    ld.add_action(declare_nav_graph_cmd)

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_dir,'/launch','/robot.launch.py']),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'description': description_name,
        }.items(),
    )
    ld.add_action(robot)

    agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_dir,'/launch','/agent.launch.py']),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'log_level': log_level,
            'description': description_name,
        }.items(),
    )
    ld.add_action(agent)


    orient_monitor = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
        Node(
            package='orient_monitor',
            executable='collision_monitor',
            name='orient_monitor_server',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                configured_params,
                {'use_sim_time': use_sim_time},
                {'yaml_filename':map_filename}
            ],
            remappings=remappings
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_monitor',
            parameters=[{
                'bond_timeout': 120000.0,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [
                    'orient_monitor_server',
                ]
            }]
        ) 
    ])
    ld.add_action(orient_monitor)

    aide_node = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
        Node(
            package='orient_aide',
            executable='musicale',
            name='orient_musicale_node',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                configured_params,
                {'use_sim_time': use_sim_time},
            ],
            remappings=remappings
        ),
        Node(
            package='orient_aide',
            executable='dac63004',
            name='orient_dac63004_node',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                configured_params,
                {'use_sim_time': use_sim_time},
            ],
            remappings=remappings
        ),
    ])
    ld.add_action(aide_node)


    robot_localization_node = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
        Node(
            package='orient_laser_odometry',
            executable='orient_laser_odometry',
            name='orient_laser_odometry',
            output='screen',
            parameters=[
                configured_params,
                {'use_sim_time': use_sim_time},
            ],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('scan', 'scan/filtered')],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='robot_localization_node',
            output='screen',
            parameters=[
                configured_params,
                {'use_sim_time': use_sim_time},
            ],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings 
        )
    ])
    ld.add_action(robot_localization_node)


    component_container = Node(
            name='orient_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen')
    ld.add_action(component_container)

    orient_amcl_group = GroupAction([
        LoadComposableNodes(
            condition=IfCondition(use_composition),
            target_container=container_name_full,
            composable_node_descriptions=[
                ComposableNode(
                    package='orient_amcl',
                    plugin='orient_amcl::AmclNode',
                    name='amcl',
                    parameters=[
                        configured_params,
                        {'use_sim_time': use_sim_time},
                    ],
                    remappings=remappings + [('scan', 'scan/filtered')]
                ),
                ComposableNode(
                    package='nav2_lifecycle_manager',
                    plugin='nav2_lifecycle_manager::LifecycleManager',
                    name='lifecycle_manager_navigation_amcl',
                    parameters=[{
                        'bond_timeout': 120000.0,
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'node_names': [
                            'amcl',
                        ]
                    }]
                ),
            ],
        )
    ])
    # ld.add_action(orient_amcl_group)


    reflector_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
        Node(
            package="orient_laser_filters",
            executable="scan_to_scan_filter_chain",
            name='orient_laser_reflector_recognize_filters_server',
            remappings= remappings + [('scan', 'scan/filtered')],
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                {'use_sim_time': use_sim_time},
                configured_params
            ]
        ),
        LoadComposableNodes(
            condition=IfCondition(use_composition),
            target_container=container_name_full,
            composable_node_descriptions=[
                ComposableNode(
                    package="orient_reflector",
                    plugin='orient_reflector::Localization',
                    name='reflector_localization_server',
                    remappings=remappings + [('scan', 'scan/filtered/reflector')],
                    parameters=[
                        configured_params,
                        {'use_sim_time': use_sim_time},
                        {'reflector_filename': map_filename},
                        {'marking': False},
                        {'publish_map': False},
                ]),
                ComposableNode(
                    package='nav2_lifecycle_manager',
                    plugin='nav2_lifecycle_manager::LifecycleManager',
                    name='lifecycle_manager_navigation_amcl',
                    parameters=[{
                        'bond_timeout': 120000.0,
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'node_names': [
                            'reflector_localization_server'
                        ]
                    }]
                ),
            ],
        )
    ])
    ld.add_action(reflector_group)


    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
        LoadComposableNodes(
            condition=IfCondition(use_composition),
            target_container=container_name_full,
            composable_node_descriptions=[
                ComposableNode(
                    package='nav2_map_server',
                    plugin='nav2_map_server::MapServer',
                    name='map_server',
                    parameters=[
                        configured_params,
                        {'use_sim_time': use_sim_time},
                        {'yaml_filename':map_filename}
                    ],
                    remappings=remappings
                ),
               ComposableNode(
                    package="orient_checkpoint",
                    plugin='orient_checkpoint::CheckpointServer',
                    name='checkpoint_server',
                    parameters=[
                        configured_params,
                        {'use_sim_time': use_sim_time},
                        {'orient_clientid': orient_clientid},
                        {'map':map_filename},
                        {'nav_graph_file':nav_graph_filename}
                    ],
                ),
                ComposableNode(
                    package="orient_govern",
                    plugin='orient_govern::Govern',
                    remappings=remappings,
                    name='govern_server',
                    parameters=[
                        configured_params,
                        {'use_sim_time': use_sim_time},
                        {'orient_clientid': orient_clientid},
                        {'behaviors': behaviors_dir},
                        {'default_nav_through_poses_bt_xml': default_nav_bt_xml},
                    ],
                ),
                ComposableNode(
                    package='orient_controller',
                    plugin='orient_controller::ControllerServer',
                    name='controller_server',
                    parameters=[
                        configured_params,
                        {'use_sim_time': use_sim_time},
                    ],
                    remappings=remappings + [('cmd_vel', 'cmd_vel_nav'),('scan', 'scan/filtered')]
                ),
                ComposableNode(
                    package='nav2_behaviors',
                    plugin='behavior_server::BehaviorServer',
                    name='behavior_server',
                    parameters=[
                        configured_params,
                        {'use_sim_time': use_sim_time},
                    ],
                    remappings=remappings + [('cmd_vel', 'cmd_vel_smoothed')]
                ),
                ComposableNode(
                    package='orient_velocity_smoother',
                    plugin='orient_velocity_smoother::VelocitySmoother',
                    name='velocity_smoother',
                    parameters=[
                        configured_params,
                        {'use_sim_time': use_sim_time},
                    ],
                    remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
                ComposableNode(
                    package='nav2_lifecycle_manager',
                    plugin='nav2_lifecycle_manager::LifecycleManager',
                    name='lifecycle_manager_navigation_agv',
                    parameters=[{
                        'bond_timeout': 120000.0,
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'node_names': [
                            'map_server',
                            'govern_server',
                            'checkpoint_server',
                            'controller_server',
                            'behavior_server',
                            'velocity_smoother',
                        ]
                    }]
                ),
            ],
        )
    ])
    ld.add_action(bringup_cmd_group)


    return ld
