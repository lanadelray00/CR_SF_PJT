################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(cr_pjt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(crsf_interfaces REQUIRED)

################################################################################
# Include directories
################################################################################
include_directories(include)

################################################################################
# Build
################################################################################

# Source files - lib, exec, target
# add_library(robot_interface src/robot_interface.cpp)

add_executable(robot_interface src/robot_interface.cpp)
add_executable(robot_interface_test src/robot_interface_test.cpp)

# target_link_libraries(robot_interface_test robot_interface)


target_include_directories(robot_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(robot_interface_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(robot_interface PUBLIC c_std_99 cxx_std_17)
target_compile_features(robot_interface_test PUBLIC c_std_99 cxx_std_17)

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

ament_target_dependencies(
  robot_interface_test
  "moveit_ros_planning_interface"
  "rclcpp"
  "rclcpp_action"
  "std_srvs"
  "control_msgs"
  "geometry_msgs"
  "crsf_interfaces"
)
################################################################################
# Install
################################################################################
install(TARGETS robot_interface
  robot_interface_test
  DESTINATION lib/${PROJECT_NAME})

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)

ament_package()
