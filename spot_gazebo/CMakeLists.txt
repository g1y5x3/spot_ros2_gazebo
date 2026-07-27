cmake_minimum_required(VERSION 3.8)
project(spot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(spot_description REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_library(spot_initial_joint_position_system SHARED
  src/initial_joint_position_system.cpp
)
target_link_libraries(spot_initial_joint_position_system
  ignition-gazebo6::ignition-gazebo6
)

add_executable(gazebo_velodyne_pointcloud_adapter
  src/gazebo_velodyne_pointcloud_adapter.cpp
)
ament_target_dependencies(gazebo_velodyne_pointcloud_adapter
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
)
target_link_libraries(gazebo_velodyne_pointcloud_adapter Eigen3::Eigen)

add_executable(thermal_colormap
  src/thermal_colormap_node.cpp
)
target_include_directories(thermal_colormap PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(thermal_colormap
  cv_bridge
  rclcpp
  sensor_msgs
)
target_link_libraries(thermal_colormap ${OpenCV_LIBS})

install(TARGETS
  spot_initial_joint_position_system
  gazebo_velodyne_pointcloud_adapter
  thermal_colormap
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  install(PROGRAMS
    test/gazebo_effort_smoke_test.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME gazebo_effort_smoke_test
  )
  install(FILES
    test/worlds/effort_smoke_test.sdf
    DESTINATION share/${PROJECT_NAME}/worlds/test
  )

  ament_add_gtest(test_thermal_colormap
    test/test_thermal_colormap.cpp
  )
  target_include_directories(test_thermal_colormap PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(test_thermal_colormap ${OpenCV_LIBS})
endif()

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in"
)
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in"
)

ament_package()
