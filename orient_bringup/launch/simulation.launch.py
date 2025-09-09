
"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from orient_common.launch import ReplacePath
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('orient_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    description_name = LaunchConfiguration('description', default=os.getenv('ORIENT_DESCRIPTION', 'fishbot'))
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    pose = {'x': LaunchConfiguration('x_pose', default='0.0'),
            'y': LaunchConfiguration('y_pose', default='0.0'),
            'z': LaunchConfiguration('z_pose', default='0.0')}
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    ld = LaunchDescription()
    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    ld.add_action(declare_namespace_cmd)
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    ld.add_action(declare_use_namespace_cmd)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    ld.add_action(declare_use_sim_time_cmd)

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    ld.add_action(declare_autostart_cmd)

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            bringup_dir, 'rviz', 'navigation_view.rviz'),
        description='Full path to the RVIZ config file to use')
    ld.add_action(declare_rviz_config_file_cmd)

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(bringup_dir, 'worlds', 'workshop.world'),
        description='Full path to world model file to load')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='forklift',
        description='name of the robot')

    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz.launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_file}.items())
    ld.add_action(rviz_cmd)

    world = os.path.join(
        get_package_share_directory('orient_bringup'),
        'worlds',
        'workshop.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )
    ld.add_action(gzserver_cmd)

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    ld.add_action(gzclient_cmd)

    model_path = ReplacePath(
        name=description_name,
        path=get_package_share_directory('orient_description'),
        source_file=os.path.join('models','model.sdf'))
    
    robot_poses = [
        {'x': 0.946800, 'y': -2.730763, 'z': 0.01, 'yaw': -0.795974}, 
    ]
   
    # robot_poses = [
    #     {'x': 1.545648, 'y': 4.627396, 'z': 0.01, 'yaw': 3.140605}, 
    # ]

 
    for i, pose in enumerate(robot_poses):
        # 生成模型到Gazebo
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', robot_name,
                '-file', model_path,
                # '-robot_namespace', f'/robot_{i}',
                '-x', str(pose['x']), 
                '-y', str(pose['y']), 
                '-z', str(pose['z']),
                '-Y', str(pose['yaw'])
            ],
            output='screen'
        )
        ld.add_action(spawn_entity)

    return ld
