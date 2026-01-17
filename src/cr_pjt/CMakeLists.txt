################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(cr_pjt)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(crsf_interfaces REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

################################################################################
# Include cpp header directory
################################################################################
include_directories(include)

################################################################################
# Create cpp executable
################################################################################
add_executable(robot_interface src/robot_interface.cpp)
# add_executable(robot_interface_test src/robot_interface_test.cpp)
# add_executable(simple_setposetarget src/simple_setposetarget.cpp)

################################################################################
# Export header include paths
################################################################################
target_include_directories(robot_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

################################################################################
# Declare ROS 2/MoveIt dependencies (headers, libraries, and runtime linking)
################################################################################
ament_target_dependencies(
  robot_interface
  "moveit_ros_planning_interface"
  "rclcpp"
  "rclcpp_action"
  "std_srvs"
  "control_msgs"
  "geometry_msgs"
  "crsf_interfaces"
)

# ament_target_dependencies(
#   robot_interface_test
#   "moveit_ros_planning_interface"
#   "rclcpp"
#   "rclcpp_action"
#   "std_srvs"
#   "control_msgs"
#   "geometry_msgs"
#   "crsf_interfaces"
# )

# ament_target_dependencies(
#   simple_setposetarget
#   "moveit_ros_planning_interface"
#   "rclcpp"
#   "rclcpp_action"
#   "std_srvs"
#   "control_msgs"
#   "geometry_msgs"
#   "crsf_interfaces"
# )

################################################################################
# Install cpp executable
################################################################################
install(TARGETS robot_interface
  # robot_interface_test
  # simple_setposetarget
  DESTINATION lib/${PROJECT_NAME})

################################################################################
# install python modules
################################################################################
# ament_python_install_package(${PROJECT_NAME})

################################################################################
# install python executables
################################################################################
install(PROGRAMS
  scripts/open_manipulator_x_pickandplace.py
  scripts/keyboard_trigger_node.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# install config files (calibration, yaml, npz, etc.)
################################################################################
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)

ament_package()
