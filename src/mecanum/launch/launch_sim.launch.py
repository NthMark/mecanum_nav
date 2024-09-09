import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    package_name='mecanum'
    rsp=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),'launch','rsp.launch.py'
            )
        ]),launch_arguments={'use_sim_time':'true','use_ros2_control':'true'}.items()
    )
    # twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    # twist_mux = Node(
    #         package="twist_mux",
    #         executable="twist_mux",
    #         parameters=[twist_mux_params, {'use_sim_time': True}],
    #         remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    #     )
    # gazebo_param_file=os.path.join(
    #             get_package_share_directory(package_name),'config','gazebo_params.yaml'
    #         )
    gazebo=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'
            )
        ]),
        # launch_arguments={'extra_gazebo_args': '--ros-args --params-file '+gazebo_param_file}.items()
    )
    spawn_entity=Node(package='gazebo_ros',executable='spawn_entity.py',
                      arguments=['-topic','robot_description','-entity','my_robot'],
                      output='screen')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    cmd_vel_topic_dec = DeclareLaunchArgument(
    'cmd_vel_topic',
    default_value='/cmd_vel',
    description='The name of the output command vel topic.')
    follow_node = Node(
            package='mecanum',
            executable='control',
            remappings=[('/cmd_vel',cmd_vel_topic)]
         )
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    # teleop_node=Node(
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     remappings=[('/cmd_vel','/diff_cont/cmd_vel_unstamped')]
    # )
    # nav_bringup=IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             get_package_share_directory('nav2_bringup'),'launch','navigation_launch.py'
    #         )
    #     ]),launch_arguments={'use_sim_time':'true'}.items()
    # )
    # default_params_file = os.path.join(get_package_share_directory("articubot"),
    #                                    'config', 'mapper_params_online_async.yaml')
    # declare_params_file_cmd = DeclareLaunchArgument(
    #     'params_file',
    #     default_value=default_params_file,
    #     description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    return LaunchDescription([
        rsp,
        # twist_mux,
        gazebo,
        spawn_entity,
        cmd_vel_topic_dec,
        follow_node
        # diff_drive_spawner,
        # joint_broad_spawner,
        # declare_params_file_cmd,
        # teleop_node
    ])