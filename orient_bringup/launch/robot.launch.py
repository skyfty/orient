import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node
from orient_common.launch import ReplacePath
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
from launch.substitutions import TextSubstitution

def generate_launch_description():
    description_name = LaunchConfiguration('description', default=os.getenv('ORIENT_DESCRIPTION', 'fishbot'))
    orient_description_share_dir = get_package_share_directory('orient_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    log_level = LaunchConfiguration('log_level', default="info")
    namespace = LaunchConfiguration('namespace', default='')

    urdf_model_path = PathJoinSubstitution([orient_description_share_dir ,description_name, 'model.urdf.xacro'])
    ld = LaunchDescription()

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    ld.add_action(declare_use_sim_time_cmd)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_model_path])
        }],
        namespace=namespace,
        output='both',
    )   
    ld.add_action(robot_state_publisher_node)

    robot_controllers = PathJoinSubstitution(
        [
            orient_description_share_dir,
            description_name,
            'params',
            'controller.yaml',
        ]
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '-c', '/r1/controller_manager'
            ],
    )
    ld.add_action(joint_state_broadcaster_spawner)

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'drive_base_controller',
            '--param-file',
            robot_controllers,
            '-c', '/r1/controller_manager'
            ],
    )
    ld.add_action(controller_spawner)

    # laser = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([bringup_dir,'/launch','/laser.launch.py']),
    #     launch_arguments={
    #         'namespace': namespace,
    #         'use_sim_time': use_sim_time,
    #         'autostart': autostart,
    #         'log_level': log_level,
    #         'description': description_name,
    #     }.items(),
    # )
    # ld.add_action(laser)



    return ld
