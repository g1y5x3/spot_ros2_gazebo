import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('spot_bringup')
    effort_share = get_package_share_directory('spot_effort_controller')
    effort_config = os.path.join(
        effort_share, 'config', 'effort_controller.yaml')
    world_arg = DeclareLaunchArgument(
        'world_file', default_value='empty_room.sdf')
    headless_arg = DeclareLaunchArgument('headless', default_value='true')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'spot.gazebo.launch.py')),
        launch_arguments={
            'world_file': LaunchConfiguration('world_file'),
            'headless': LaunchConfiguration('headless'),
            'rviz': 'false',
            'simulator_delay': '2.0',
        }.items(),
    )
    backend = Node(
        package='spot_effort_controller',
        executable='effort_backend',
        output='screen',
        parameters=[effort_config],
    )
    controller = Node(
        package='spot_effort_controller',
        executable='standing_controller',
        output='screen',
        parameters=[effort_config],
    )

    return LaunchDescription([
        world_arg,
        headless_arg,
        backend,
        controller,
        simulation,
    ])
