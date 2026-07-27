import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    world_file = LaunchConfiguration('world_file', default='simple_tunnel.sdf')
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='simple_tunnel.sdf',
        description='Name of the world file to load'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false', 
        description='Open RViz.'
    )

    headless = LaunchConfiguration('headless')
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Run only the Gazebo server (recommended for tests).'
    )

    simulator_delay = LaunchConfiguration('simulator_delay')
    simulator_delay_arg = DeclareLaunchArgument(
        'simulator_delay',
        default_value='0.0',
        description='Wall seconds to let bridges/controllers start first.'
    )

    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value='spot.rviz',
        description='RViz configuration file to use'
    )

    velodyne_adapter_arg = DeclareLaunchArgument(
        'velodyne_adapter',
        default_value='false',
        description='Publish a synthetic Velodyne-compatible point cloud.'
    )

    # Setup to launch the simulator and Gazebo world
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_spot_gazebo = get_package_share_directory('spot_gazebo')
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': [
                    '-r -s ',
                    PathJoinSubstitution([
                        pkg_spot_gazebo, 
                        'worlds',
                        world_file
                    ]),
                ],
            }.items(),
            condition=IfCondition(headless),
    )
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': [
                    '-r ',
                    PathJoinSubstitution([
                        pkg_spot_gazebo,
                        'worlds',
                        world_file
                    ]),
                ],
            }.items(),
            condition=UnlessCondition(headless),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    pkg_spot_bringup = get_package_share_directory('spot_bringup')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'config_file': os.path.join(pkg_spot_bringup, 'config', 'spot_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }]
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    pkg_spot_description = get_package_share_directory('spot_description')
    sdf_file = os.path.join(pkg_spot_description, 'models', 'spot', 'model.sdf')
    with open(sdf_file, 'r') as infp: robot_desc = infp.read()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
            {"publish_frequency": 200.0},
        ],
        remappings=[
            ('/joint_states', '/spot/joint_states')
        ]
    )

    # Visualize in RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([
            pkg_spot_bringup,
            'config',
            LaunchConfiguration('rviz_config_file')
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}]
    )

    velodyne_adapter = Node(
        package='spot_gazebo',
        executable='gazebo_velodyne_pointcloud_adapter',
        name='gazebo_velodyne_pointcloud_adapter',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'target_frame': 'base_link',
            'input_topic': '/spot/lidar/points',
            'output_topic': '/velodyne_points',
            'scan_rate': 10.0,
            'num_scan_lines': 16,
            'vertical_fov_min': -15.0,
            'vertical_fov_max': 15.0,
        }],
        condition=IfCondition(LaunchConfiguration('velodyne_adapter')),
    )

    thermal_colormap = Node(
        package='spot_gazebo',
        executable='thermal_colormap',
        name='thermal_colormap',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        world_file_arg,
        rviz_arg,
        rviz_config_file_arg,
        velodyne_adapter_arg,
        headless_arg,
        simulator_delay_arg,
        bridge,
        robot_state_publisher,
        velodyne_adapter,
        thermal_colormap,
        rviz,
        TimerAction(
            period=simulator_delay,
            actions=[gz_sim_headless, gz_sim_gui]),
    ])
