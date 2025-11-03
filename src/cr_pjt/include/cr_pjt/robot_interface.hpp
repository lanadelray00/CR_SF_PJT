#pragma once
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose.hpp>

class RobotInterface : public rclcpp::Node
{
public:
  RobotInterface();
  void initMoveGroups();

  bool moveToPose(double x, double y, double z,
                  double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0);
  void openGripper();
  void closeGripper();
  void moveHome();
  void moveToNamedPose(const std::string &pose_name);
  void emergencyStopCallback(const std_srvs::srv::Trigger::Request::SharedPtr,
                             std_srvs::srv::Trigger::Response::SharedPtr response);

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_srv_;
};
