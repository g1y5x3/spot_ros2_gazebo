import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('spot_ros2_bringup')
    pkg_project_gazebo = get_package_share_directory('spot_ros2_gazebo')
    pkg_project_description = get_package_share_directory('spot_ros2_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Load the SDF file from "description" package
    sdf_file = os.path.join(pkg_project_description, 'models', 'spot', 'model.sdf')
    with open(sdf_file, 'r') as infp: 
        robot_desc = infp.read()

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': [
                    PathJoinSubstitution([
                        pkg_project_gazebo, 
                        'worlds',
                        'edgar_mine_world.sdf'
                    ]),
                    # ' -v 4'   # show debug messages for gazebo
                ],
            }.items(),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'spot_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    # Visualize in RViz
    # Currenly the joint angles displayed in RVIZ is different from what the 
    # simulator has shown
    # rviz = Node(
    #    package='rviz2',
    #    executable='rviz2',
    #    arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'diff_drive.rviz')],
    #    condition=IfCondition(LaunchConfiguration('rviz'))
    # )

    return LaunchDescription([
        # DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        gz_sim,
        bridge,
        robot_state_publisher,
        # rviz
    ])
