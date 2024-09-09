import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import xacro
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # moveit_cpp.yaml is passed by filename for now since it's node specific
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # map_yaml_file = LaunchConfiguration('map')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
    slam = LaunchConfiguration('slam')
    paramfilename='mpo_500.yaml'
    map_dir = LaunchConfiguration('map')
    param_dir=LaunchConfiguration('params_file')
    namespace=LaunchConfiguration('namespace')
    declare_use_sim_time=DeclareLaunchArgument(
        'use_sim_time',
        default_value='true')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')
    declare_param_file_cmd=DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(get_package_share_directory('test_mecanum'),'config',paramfilename),
            description='Full path to param file to load')
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            get_package_share_directory('test_mecanum'),
            'config',
            'aws_warehouse.yaml'),
        description='Full path to map file to load')

    
    # mapper_params_online_async=IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             get_package_share_directory('slam_toolbox'),'launch'
    #         ),'online_async_launch.py'
    #     ]),
    #     launch_arguments={
    #         'slam_params_file' : os.path.join(get_package_share_directory('test_mecanum'),'config','mapper_params_online_async.yaml'),
    #         'use_sim_time': use_sim_time
    #     }.items()
    # )
    # zm_robot_description_path = os.path.join(
    #     get_package_share_directory('test_mecanum'))

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

   

    # node_joint_state_publisher = Node(
    #         package='joint_state_publisher',
    #         executable='joint_state_publisher',
    #         output='screen',
    #         parameters=[{'use_sim_time': use_sim_time}]
    #     )


    display_rviz = Node(package='rviz2', executable='rviz2',
                        name='rviz2',
                        arguments=['-d', rviz_config_dir],
                        parameters=[{'use_sim_time': use_sim_time}],
                        output='screen')
    # bringup_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(nav2_launch_dir, 'bringup_launch.py')),
    #     launch_arguments={'slam':slam,
    #                       'map': map_dir,
    #                       'use_sim_time': use_sim_time,
    #                       'params_file': param_dir,
    #                       'autostart': 'True'}.items())
    localization_amcl_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('test_mecanum'), '/launch/localization_amcl.launch.py']),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'params_file': param_dir,
            'namespace': namespace}.items(),
    )
    # starting the navigation
    navigation_neo_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('test_mecanum'), '/launch/navigation_neo.launch.py']),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'params_file': param_dir}.items(),
    )
    return LaunchDescription([
        declare_use_sim_time,
        declare_namespace_cmd,
        declare_slam_cmd,
        declare_map_yaml_cmd,
        declare_param_file_cmd,
        # bringup_cmd,
        navigation_neo_launch_description,
        localization_amcl_launch_description,
        display_rviz,
    #   mapper_params_online_async,
    #   map_server
    ])
