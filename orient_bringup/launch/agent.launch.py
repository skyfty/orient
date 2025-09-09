import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from nav2_common.launch import RewrittenYaml

from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from orient_common.launch import ReplacePath
from launch_ros.descriptions import ParameterFile
from launch.conditions import UnlessCondition

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    namespace = LaunchConfiguration('namespace', default='')
    log_level = LaunchConfiguration('log_level', default="info")

    description_name = LaunchConfiguration('description', default=os.getenv('ORIENT_DESCRIPTION', 'fishbot'))  
    orient_clientid = LaunchConfiguration('orient_clientid', default=os.getenv('ORIENT_CLIENTID', description_name))

    agent_params_filename = ReplacePath(
        name=description_name,
        path=get_package_share_directory('orient_description'),
        source_file=os.path.join('params','agent.yaml'))
    
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'orient_clientid', default_value=orient_clientid,
        description='Orient client id'))
    
    ld.add_action(DeclareLaunchArgument(
        'description', default_value=description_name,
        description='description name'))

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value=log_level,
        description='log level')
    ld.add_action(declare_log_level_cmd)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    ld.add_action(declare_use_sim_time_cmd)
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace')
    ld.add_action(declare_namespace_cmd)


    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]           

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time}

    agent_params = ParameterFile(
        RewrittenYaml(
            source_file=agent_params_filename,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)
    

    node = Node(
        package='orient_mqtt',
        name='orient_agent_mqtt',
        executable='orient_mqtt',
        output='screen',
        remappings=remappings,
        namespace=namespace,
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            agent_params,
            {'orient_clientid': orient_clientid},
            {'use_sim_time': use_sim_time},
        ]
    )

    ld.add_action(node)
    return ld
