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
from launch.actions import OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument, RegisterEventHandler

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    namespace = LaunchConfiguration('namespace', default='')

    def save_map(context, *args, **kwargs):
        path = LaunchConfiguration('path').perform(context)
        
        # 第一个节点：map_saver_cli
        map_saver_node = Node(
            package='nav2_map_server',
            executable='map_saver_cli',
            name='map_saver_cli',
            output='screen',
            arguments=['-f', path,  '-t', '/map', '--fmt', 'png'],
            parameters=[{'use_sim_time': use_sim_time}],
            namespace=namespace,
        )
        
        # 第二个节点：reflector_map_saver
        reflector_map_saver = Node(
            package='orient_reflector',
            executable='saver',
            name='reflector_map_saver',
            output='screen',
            arguments=['-f', path],
            parameters=[{'use_sim_time': use_sim_time}],
            namespace=namespace,
        )
        
        # 事件处理器：当 map_saver_cli 退出时启动 reflector_map_saver
        map_saver_exit_handler = RegisterEventHandler(
            OnProcessExit(
                target_action=map_saver_node,
                on_exit=[reflector_map_saver]
            )
        )
        
        return [map_saver_node, map_saver_exit_handler]
    
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'path',
            default_value='reflector',
            description='Path to save the map'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace'
        ),
        OpaqueFunction(function=save_map)
    ])


    return ld
