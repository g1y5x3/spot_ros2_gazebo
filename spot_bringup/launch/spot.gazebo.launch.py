import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


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

    champ_arg = DeclareLaunchArgument(
        'champ',
        default_value='false',
        description='Run the legacy CHAMP publisher (diagnostics only; the '
                    'model is configured for direct effort control).'
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

    # Controller
    config_path = get_package_share_directory("champ_config")
    links_config = PathJoinSubstitution([config_path, 'config', 'links', 'links.yaml'])
    links_param = ParameterFile(param_file=links_config, allow_substs=True)

    joints_config = PathJoinSubstitution([config_path, 'config', 'joints', 'joints.yaml'])
    joints_param = ParameterFile(param_file=joints_config, allow_substs=True) 

    gait_config = PathJoinSubstitution([config_path, 'config', 'gait', 'gait.yaml'])
    gait_param = ParameterFile(param_file=gait_config, allow_substs=True) 

    urdf_file = os.path.join(pkg_spot_description, 'models', 'spot', 'model.urdf')

    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"gazebo": True},
            {"publish_joint_states": False},
            {"publish_foot_contacts": False},
            {"publish_joint_control": True},
            {"joint_controller_topic": "/spot/joint_trajectory"},
            {"urdf": urdf_file},
            links_param,
            joints_param,
            gait_param
        ],
        remappings=[
            ("/cmd_vel/smooth", "/cmd_vel"),
        ],
        condition=IfCondition(LaunchConfiguration('champ')),
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

    # Transform point cloud from lidar_link to base_link for DLO
    pointcloud_transform = Node(
        package='spot_bringup',
        executable='pointcloud_transform',
        name='pointcloud_transform',
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
        }]
    )

    # Convert thermal camera to RGB for visualization
    thermal_to_rgb = Node(
        package='spot_bringup',
        executable='thermal_to_rgb',
        name='thermal_to_rgb',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        world_file_arg,
        rviz_arg,
        rviz_config_file_arg,
        headless_arg,
        champ_arg,
        simulator_delay_arg,
        bridge,
        robot_state_publisher,
        quadruped_controller_node,
        pointcloud_transform,
        thermal_to_rgb,
        rviz,
        TimerAction(
            period=simulator_delay,
            actions=[gz_sim_headless, gz_sim_gui]),
    ])
