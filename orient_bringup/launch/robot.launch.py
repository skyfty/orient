import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch.substitutions import TextSubstitution

def generate_launch_description():
    orient_description_share_dir = get_package_share_directory('orient_description')
    description_name = LaunchConfiguration('description', default=os.getenv('ORIENT_DESCRIPTION', 'fishbot'))

    namespace = LaunchConfiguration('namespace', default='')
    
    urdf_model_path = PathJoinSubstitution([
        orient_description_share_dir,
        description_name,
        TextSubstitution(text='model.urdf.xacro'),
    ])

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]           
    ld = LaunchDescription()

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    ld.add_action(declare_use_sim_time_cmd)
    
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot')
    ld.add_action(declare_namespace_cmd)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_model_path, ' namespace:=', namespace]),
        }],
        remappings=remappings,
        namespace=namespace,
        output='both',
    )   
    ld.add_action(robot_state_publisher_node)



    # laser = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([bringup_dir,'/launch','/laser.launch.py']),
    #     launch_arguments={
    #         'namespace': namespace,
    #         'use_sim_time': use_sim_time,
    #         'log_level': log_level,
    #         'description': description_name,
    #     }.items(),
    # )
    # ld.add_action(laser)

    robot_controllers = PathJoinSubstitution(
        [
            orient_description_share_dir,
            description_name,
            'params',
            'controller.yaml',
        ]
    )
    controller_manager_name = PathJoinSubstitution([namespace, 'controller_manager'])
    
    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        remappings=remappings,
        arguments=[ 'joint_state_broadcaster', '--param-file', robot_controllers, '-c', controller_manager_name],
    )
    ld.add_action(joint_state_broadcaster_spawner)
    

    drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        remappings=remappings,
        arguments=['drive_base_controller','--param-file',robot_controllers,'-c', controller_manager_name],
    )
    ld.add_action(drive_base_controller_spawner)
    
    
    return ld
