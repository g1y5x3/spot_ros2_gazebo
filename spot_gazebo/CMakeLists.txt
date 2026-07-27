cmake_minimum_required(VERSION 3.5)

# Following directive defines the project name.
project(spot_gazebo)


# Following directives find required packages and load their configuration.
# The 'set' directive defines a variable (e.g. 'GZ_PLUGIN_VER').
# Such variables can be used lateron in the CMakeLists.txt file.
find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(spot_description REQUIRED)

add_library(spot_initial_joint_position_system SHARED
  src/initial_joint_position_system.cpp
)
target_link_libraries(spot_initial_joint_position_system
  ignition-gazebo6::ignition-gazebo6
)

install(
  TARGETS spot_initial_joint_position_system
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# Following 'install' directive ensures that the 'worlds' subfolder
# will be copied to the 'share/spot_gazebo/worlds'
# subfolder of the installation directory.
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Following directives are used when testing.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Following hooks are used to ensure that the correct environment variables
# will be set by exectuting 'sourece install/setup.bash' after compilation.
# When using this template for your project, change the filenames of the
# files in the 'hooks' folder, to correspond to your project name.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

# Following directive configures ament based on the previous directives, and should
# typically be the last directive in the CMakeLists.txt file.
ament_package()
