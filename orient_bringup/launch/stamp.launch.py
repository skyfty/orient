# 导入库
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
import os
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from orient_common.launch import ReplacePath
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import UnlessCondition
from nav2_common.launch import RewrittenYaml, ReplaceString
from launch.conditions import IfCondition
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.actions import PushRosNamespace

# 定义函数名称为：generate_launch_description
def generate_launch_description():
    bringup_dir = get_package_share_directory('orient_bringup')
    description_dir = get_package_share_directory('orient_description')

    description_name = LaunchConfiguration('description', default=os.getenv('ORIENT_DESCRIPTION', 'fishbot'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    autostart = LaunchConfiguration('autostart', default='True')
    namespace = LaunchConfiguration('namespace', default='')
    use_namespace = LaunchConfiguration('use_namespace')
    log_level = LaunchConfiguration('log_level', default="info")
    map_filename = LaunchConfiguration('map', default=os.path.join(bringup_dir, 'maps','reflector.yaml'))
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    hostip = LaunchConfiguration('hostip', default=os.getenv('ORIENT_DESCRIPTION', '192.168.0.6'))
    orient_clientid = LaunchConfiguration('orient_clientid', default=os.getenv('ORIENT_CLIENTID', description_name))
    nav_graph_filename = LaunchConfiguration('nav_graph_file', default=os.path.join(bringup_dir, 'nav_graphs','0.yaml'))

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'orient_clientid': orient_clientid
    }
    
    stand_params_path = ReplacePath(
        name=description_name,
        path=description_dir,
        source_file=os.path.join('params','stand.yaml'))
    
    params_file = ReplaceString(
        source_file=stand_params_path,
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

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
    ld.add_action(declare_use_namespace_cmd)

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    ld.add_action(declare_namespace_cmd)

    ld.add_action(DeclareLaunchArgument(
        'description', default_value=description_name,
        description='description name'))

    declare_map_cmd = DeclareLaunchArgument(
        'map', default_value=map_filename,
        description='Full path to map yaml file')
    ld.add_action(declare_map_cmd)

    declare_resolution_cmd = DeclareLaunchArgument(
        'resolution', default_value=resolution,
        description='resolution of the map')
    ld.add_action(declare_resolution_cmd)

    declare_publish_period_sec_cmd = DeclareLaunchArgument(
        'publish_period_sec', default_value=publish_period_sec,
        description='period of the map')
    ld.add_action(declare_publish_period_sec_cmd)

    ld.add_action(DeclareLaunchArgument(
        'hostip', default_value=hostip,
        description='hostip name'))

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    ld.add_action(declare_log_level_cmd)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    ld.add_action(declare_use_sim_time_cmd)

    declare_nav_graph_cmd = DeclareLaunchArgument(
        'nav_graph_file', default_value=nav_graph_filename,
        description='Full path to nav_graph_file yaml file')
    ld.add_action(declare_nav_graph_cmd)

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]           

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
            'hostip': hostip,
        }.items(),
    )
    ld.add_action(agent)

    robot_localization_node = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
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
        ),
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
        )
    ])
    ld.add_action(robot_localization_node)

    # cmdvel  = Node(
    #     package="orient_cmdvel",
    #     executable="udp_cmdvel",
    #     name='orient_cmdvel',
    #     remappings= remappings,
    #     arguments=['--ros-args', '--log-level', log_level],
    #     parameters=[
    #         stand_params_path
    # ])
    # ld.add_action(cmdvel)

    checkpoint_node = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
        Node(
            package="orient_checkpoint",
            executable="checkpoint",
            name='checkpoint_server',
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            parameters=[
                configured_params,
                {'use_sim_time': use_sim_time},
                {'orient_clientid': orient_clientid},
                {'nav_graph_file':nav_graph_filename}
            ]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_checkpoint',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': [
                            'checkpoint_server',
                        ]}])
    ])
    ld.add_action(checkpoint_node)

    reflector_node = GroupAction([
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
                configured_params,
                {'use_sim_time': use_sim_time},
        ]),
        Node(
            package="orient_reflector",
            executable="localization",
            name='reflector_localization_server',
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('scan', 'scan/filtered/reflector')],
            parameters=[
                configured_params,
                {'reflector_filename': map_filename},
                {'marking': True},
                {'use_sim_time': use_sim_time},
                {'publish_map': True},
            ]),
        Node(
            package="orient_reflector",
            executable="grid_mapper",
            name='reflector_grid_mapper_server',
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('scan', 'scan/filtered')],
            parameters=[
                configured_params,
                {'use_sim_time': use_sim_time},
            ]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': [
                            'reflector_localization_server',
                            'reflector_grid_mapper_server',
                        ]}])
    ])
    ld.add_action(reflector_node)

    # backpack_2d_params_path = ReplacePath(
    #     name=description_name,
    #     path=description_dir,
    #     source_file=os.path.join('params',''))
    # cartographer_nodes = GroupAction([
    #     PushRosNamespace(
    #         condition=IfCondition(use_namespace),
    #         namespace=namespace),
    #     Node(
    #         package = 'cartographer_ros',
    #         executable = 'cartographer_node',
    #         parameters = [{'use_sim_time': use_sim_time}],
    #         remappings= remappings + [
    #             ('scan', 'scan/filtered'),
    #             ('echoes', 'horizontal_laser_2d')
    #         ],
    #         arguments = [
    #             '-configuration_directory', backpack_2d_params_path,
    #             '-configuration_basename', 'backpack_2d.lua',
    #             '-minloglevel', '2'
    #         ],
    #         output = 'screen'
    #     ),
    #     Node(
    #         package = 'cartographer_ros',
    #         executable = 'cartographer_occupancy_grid_node',
    #         remappings=remappings,
    #         arguments=[
    #             '-publish_period_sec', publish_period_sec],
    #         parameters = [
    #             {'use_sim_time': use_sim_time},
    #             {'resolution': resolution},
    #         ],
    #     )
    # ])
    # ld.add_action(cartographer_nodes)


    return ld

