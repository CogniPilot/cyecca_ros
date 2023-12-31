from pathlib import Path

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, \
        DeclareLaunchArgument, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # launch substiutions
    use_sim_time = True
    logger = LaunchConfiguration('log_level')
    speed = LaunchConfiguration('speed')

    # launch arguments
    arg_log_level = DeclareLaunchArgument(
        'log_level',
        default_value=['warn'],
        description='Logging level'
    )

    arg_speed = DeclareLaunchArgument(
        'speed',
        default_value=['1.0'],
        description='Simulation Speed Factor'
    )

    node_simulator = Node(
       package='cyecca_ros',
       output='screen',
       executable='quad_simulator',
       arguments=['--ros-args', '--log-level', logger],
       parameters=[
         {'use_sim_time': use_sim_time},
         {'speed': speed},
       ],
       remappings=[
       ],
    )

    node_estimator = Node(
       package='cyecca_ros',
       output='screen',
       executable='quad_estimator',
       arguments=['--ros-args', '--log-level', logger],
       parameters=[
         {'use_sim_time': use_sim_time},
       ],
       remappings=[
       ],
    )

    node_controller = Node(
       package='cyecca_ros',
       output='screen',
       executable='quad_controller',
       arguments=['--ros-args', '--log-level', logger],
       parameters=[
         {'use_sim_time': use_sim_time},
       ],
       remappings=[
       ],
    )

    return LaunchDescription([
        arg_log_level,
        arg_speed,
        node_estimator,
        node_simulator,
        node_controller,
    ])
