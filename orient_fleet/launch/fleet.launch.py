import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node
from orient_common.launch import ReplacePath
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import PushRosNamespace
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ParameterFile

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    log_level = LaunchConfiguration('log_level', default="info")
    namespace = LaunchConfiguration('namespace', default='')
    use_namespace = LaunchConfiguration('use_namespace')
    server_uri = LaunchConfiguration('server_uri')
    nav_graph_file = LaunchConfiguration('nav_graph_file')
    config_file = LaunchConfiguration('config_file')
    agent_config_file = LaunchConfiguration('agent_config_file')

    param_substitutions = {
        'use_sim_time': use_sim_time}

    agent_params = ParameterFile(
        RewrittenYaml(
            source_file=agent_config_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)
    
    ld = LaunchDescription()

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    ld.add_action(declare_use_sim_time_cmd)
    
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    ld.add_action(declare_log_level_cmd)

    
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
    
    declare_server_uri_cmd = DeclareLaunchArgument(
        'server_uri',
        default_value='',
        description='The URI of the api server to transmit state and task information.')
    ld.add_action(declare_server_uri_cmd)
    
    agent_config_file_cmd = DeclareLaunchArgument(
        'agent_config_file',
        default_value='',
        description='The agent configuration file')
    ld.add_action(agent_config_file_cmd)

    nav_graph_file_cmd = DeclareLaunchArgument(
        'nav_graph_file',
        default_value='',
        description='The graph that this fleet should use for navigation')
    ld.add_action(nav_graph_file_cmd)
    
    config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='The config file that provides important parameters for setting up the adapter')
    ld.add_action(config_file_cmd)
    
    mqtt_server_cmd = DeclareLaunchArgument(
        'mqtt_server',
        default_value='localhost',
        description='MQTT server url to publish GPS and other data')
    ld.add_action(mqtt_server_cmd)

    orient_fleet_node = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
        Node(
            package='rmf_traffic_ros2',
            executable='rmf_traffic_schedule',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            namespace=namespace,
            output='both',
        ),
        Node(
            package='rmf_traffic_ros2',
            executable='rmf_traffic_blockade',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            namespace=namespace,
            output='both',
        ),
        Node(
            package='rmf_fleet_adapter',
            executable='door_supervisor',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            namespace=namespace,
            output='both',
        ),
        Node(
            package='rmf_fleet_adapter',
            executable='lift_supervisor',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            namespace=namespace,
            output='both',
        ),
        Node(
            package='rmf_task_ros2',
            executable='rmf_task_dispatcher',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': use_sim_time,
                'server_uri': server_uri,
                'config_file': config_file,
            }],
            namespace=namespace,
            output='both',
        ),
        Node(
            package='orient_fleet',
            executable='fleet_manager',
            arguments=['--config_file', config_file],
            parameters=[{
                'use_sim_time': use_sim_time
            }],
            namespace=namespace,
            output='both',
        ),
        Node(
            package='orient_fleet',
            executable='fleet_adapter',
            arguments=['--config_file', config_file, '--nav_graph', nav_graph_file],
            parameters=[{
                'use_sim_time': use_sim_time,
                'server_uri': server_uri,
            }],
            namespace=namespace,
            output='both',
        ),
        Node(
            package='orient_fleet',
            executable='fleet_mqtt_bridge',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                agent_params,
                {'use_sim_time': use_sim_time}
            ],
            namespace=namespace,
            output='both',
        )   
    ])
    ld.add_action(orient_fleet_node)

    return ld
