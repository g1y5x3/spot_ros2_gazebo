import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('spot_bringup')
    description_share = get_package_share_directory('spot_description')
    effort_share = get_package_share_directory('spot_effort_controller')
    estimator_share = get_package_share_directory('spot_state_estimator')
    mpc_share = get_package_share_directory('spot_ocs2_mpc')
    wbc_share = get_package_share_directory('spot_wbc')
    effort_config = os.path.join(
        effort_share, 'config', 'effort_controller.yaml')
    estimator_config = os.path.join(
        estimator_share, 'config', 'state_adapter.yaml')
    mpc_config = os.path.join(mpc_share, 'config', 'controller.yaml')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_share, 'launch', 'spot.gazebo.launch.py')),
        launch_arguments={
            'world_file': LaunchConfiguration('world_file'),
            'headless': LaunchConfiguration('headless'),
            'champ': 'false',
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
    observation_bridge = Node(
        package='spot_ocs2_mpc',
        executable='observation_bridge',
        output='screen',
        parameters=[mpc_config],
    )
    reference = Node(
        package='spot_ocs2_mpc',
        executable='cmd_vel_reference',
        output='screen',
        parameters=[mpc_config],
    )
    gait_manager = Node(
        package='spot_ocs2_mpc',
        executable='gait_manager',
        output='screen',
        parameters=[
            mpc_config,
            {
                'stationary_gait': LaunchConfiguration('stationary_gait'),
                'moving_gait': LaunchConfiguration('moving_gait'),
            },
        ],
    )
    wbc = Node(
        package='spot_wbc',
        executable='wbc',
        output='screen',
        parameters=[
            os.path.join(wbc_share, 'config', 'wbc.yaml'),
            {'enable_policy_tracking': ParameterValue(
                LaunchConfiguration('enable_policy_tracking'),
                value_type=bool)},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_file', default_value='ocs2_test.sdf'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument(
            'enable_policy_tracking', default_value='true'),
        DeclareLaunchArgument(
            'moving_gait', default_value='conservative_crawl'),
        DeclareLaunchArgument(
            'stationary_gait', default_value='stance'),
        backend,
        state_adapter,
        mpc,
        observation_bridge,
        reference,
        gait_manager,
        wbc,
        simulation,
    ])
