import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from orient_common.launch import ReplacePath
from nav2_common.launch import RewrittenYaml
from launch.conditions import UnlessCondition

def generate_launch_description():
    description_name = LaunchConfiguration('description', default=os.getenv('ORIENT_DESCRIPTION', 'fishbot'))
    params_path = ReplacePath(
        name=description_name,
        path=get_package_share_directory('orient_description'),
        source_file=os.path.join('params','r2000.yaml'))
    
    log_level = LaunchConfiguration('log_level', default="error")
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    namespace = LaunchConfiguration('namespace', default='')

    ld = LaunchDescription()

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]           
    param_substitutions = {
        'use_sim_time': use_sim_time}

    laser_params = ParameterFile(
        RewrittenYaml(
            source_file=params_path,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)
    

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    ld.add_action(declare_use_sim_time_cmd)
    

    ld.add_action(DeclareLaunchArgument(
        'description', default_value=description_name,
        description='description name'))
    
    node = Node(
        package='pf_driver',
        name='pf_r2000',
        executable='ros_main',
        output='screen',
        condition=UnlessCondition(use_sim_time),
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings + [('/pf/scan', '/scan')],
        parameters=[
            laser_params,
            {'use_sim_time': use_sim_time},
        ]
    )
    ld.add_action(node)

    laser_filters_node = Node(
        package="orient_laser_filters",
        executable="scan_to_scan_filter_chain",
        name='orient_laser_scan_filters_node',
        remappings=remappings,
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            laser_params,
            {'use_sim_time': use_sim_time},
        ])
    ld.add_action(laser_filters_node)
    return ld
