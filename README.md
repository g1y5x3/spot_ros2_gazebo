# spot_ros2_gazebo
A ROS 2 Spot simulator using Gazebo for prototyping spot autonomous behaviors.

![gazebo_rviz_spot](assets/gazebo_rviz_spot.png)

## Included packages

* `spot_ros2_description` - holds the sdf description of the custom spot model with lidar and camera sensors.

* `spot_ros2_gazebo` - holds gazebo specific code and configurations. Including the edgar mine environment.

* `spot_ros2_bringup` - holds launch files and high level utilities.

* `spot_ros2_application` - holds locomotion controller.

## Install

### Requirements

1. Currently all packages are only tested with ROS 2 Humble + Gazebo Fortress

2. Install necessary tools

    ```bash
    sudo apt install python3-vcstool python3-colcon-common-extensions git wget
    ```

## Usage

1. Install dependencies

    ```bash
    cd ~/ros2_ws
    source /opt/ros/humble/setup.bash
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -i -y --rosdistro humble
    ```

2. Build the project

    ```bash
    colcon build --cmake-args -DBUILD_TESTING=ON
    ```

3. Source the workspace

    ```bash
    source ~/ros2_ws/install/setup.sh
    ```

4. Launch the simulation

    ```bash
    ros2 launch spot2_bringup spot.launch.py
    ```
## Acknowledgement
* [template documentation](https://gazebosim.org/docs/latest/ros_gz_project_template_guide).
