#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose.hpp>

// Custom Interfaces
#include "crsf_interfaces/srv/robot_interface_movetopose.hpp"
#include "crsf_interfaces/srv/robot_interface_one_string.hpp"

class RobotInterface : public rclcpp::Node
{
public:
  RobotInterface();

  void initMoveGroups();
  void moveToNamedPose(const std::string &pose_name = "home");

private:
  // Service callbacks
  void emergencyStopCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  
  void moveToPoseCallback(
    const std::shared_ptr<crsf_interfaces::srv::RobotInterfaceMovetopose::Request> request,
    std::shared_ptr<crsf_interfaces::srv::RobotInterfaceMovetopose::Response> response);
  
  void GripperctrlCallback(
    const std::shared_ptr<crsf_interfaces::srv::RobotInterfaceOneString::Request> request,
    std::shared_ptr<crsf_interfaces::srv::RobotInterfaceOneString::Response> response);

  void moveToNamedCallback(
    const std::shared_ptr<crsf_interfaces::srv::RobotInterfaceOneString::Request> request,
    std::shared_ptr<crsf_interfaces::srv::RobotInterfaceOneString::Response> response);

  // Internal gripper action
  bool sendGripperCommand(double position, double effort = 0.0);

  // Member
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client_;
  // Service Member
  rclcpp::Service<crsf_interfaces::srv::RobotInterfaceMovetopose>::SharedPtr movetopose_srv_;
  rclcpp::Service<crsf_interfaces::srv::RobotInterfaceOneString>::SharedPtr movetonamed_srv_;
  rclcpp::Service<crsf_interfaces::srv::RobotInterfaceOneString>::SharedPtr gripper_ctrl_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_srv_;
  
};
