import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mpc_share = get_package_share_directory('spot_ocs2_mpc')
    bringup_share = get_package_share_directory('spot_bringup')
    description_share = get_package_share_directory('spot_description')
    estimator_share = get_package_share_directory('spot_state_estimator')
    controller_config = os.path.join(
        mpc_share, 'config', 'controller.yaml')
    estimator_config = os.path.join(
        estimator_share, 'config', 'state_adapter.yaml')

    standing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_share, 'launch', 'spot.standing.launch.py')),
        launch_arguments={
            'world_file': LaunchConfiguration('world_file'),
            'headless': LaunchConfiguration('headless'),
        }.items(),
    )
    state_adapter = Node(
        package='spot_state_estimator',
        executable='state_adapter',
        output='screen',
        parameters=[estimator_config],
    )
    mpc = Node(
        package='ocs2_legged_robot_ros',
        executable='legged_robot_sqp_mpc',
        name='legged_robot_sqp_mpc',
        output='screen',
        parameters=[{
            'multiplot': False,
            'taskFile': os.path.join(mpc_share, 'config', 'task.info'),
            'referenceFile': os.path.join(
                mpc_share, 'config', 'reference.info'),
            'urdfFile': os.path.join(
                description_share, 'models', 'spot', 'model.urdf'),
        }],
    )
    bridge = Node(
        package='spot_ocs2_mpc',
        executable='observation_bridge',
        output='screen',
        parameters=[controller_config],
    )
    reference = Node(
        package='spot_ocs2_mpc',
        executable='cmd_vel_reference',
        output='screen',
        parameters=[controller_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_file', default_value='ocs2_test.sdf'),
        DeclareLaunchArgument('headless', default_value='true'),
        standing,
        state_adapter,
        mpc,
        bridge,
        reference,
    ])
