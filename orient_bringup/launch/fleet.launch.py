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
    orient_fleet_dir = get_package_share_directory('orient_fleet')
    description_dir = get_package_share_directory('orient_description')

    description_name = LaunchConfiguration('description', default=os.getenv('ORIENT_DESCRIPTION', 'kf2404'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    namespace = LaunchConfiguration('namespace', default='')
    log_level = LaunchConfiguration('log_level')
    nav_graph_file = LaunchConfiguration('nav_graph_file')
    agent_config_file = LaunchConfiguration('agent_config_file')
    orient_clientid = LaunchConfiguration('orient_clientid', default=os.getenv('ORIENT_CLIENTID', description_name))
    
    fleet_config_file = ReplacePath(
        name=description_name,
        path=description_dir,
        source_file=os.path.join('params','fleet.yaml'))
    
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
    
    nav_graph_file_cmd = DeclareLaunchArgument(
        'nav_graph_file',
        default_value=os.path.join(bringup_dir, 'nav_graphs','0.yaml'),
        description='The graph that this fleet should use for navigation')
    ld.add_action(nav_graph_file_cmd)
    
    agent_config_file_cmd = DeclareLaunchArgument(
        'agent_config_file',
        default_value=ReplacePath(
            name=description_name,
            path=get_package_share_directory('orient_description'),
            source_file=os.path.join('params','agent.yaml')),
        description='The agent configuration file')
    ld.add_action(agent_config_file_cmd)

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    ld.add_action(declare_log_level_cmd)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    ld.add_action(declare_use_sim_time_cmd)

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([orient_fleet_dir,'/launch','/fleet.launch.py']),
        launch_arguments={
            'namespace': namespace,
            'log_level': log_level,
            'use_sim_time': use_sim_time,
            'config_file': fleet_config_file,
            'nav_graph_file': nav_graph_file,
            'agent_config_file': agent_config_file,
            'orient_clientid': orient_clientid,
        }.items(),
    )
    ld.add_action(robot)
    return ld

