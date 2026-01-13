#pragma once

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "control_msgs/action/gripper_command.hpp"

#include "moveit/move_group_interface/move_group_interface.h"

#include "crsf_interfaces/action/move_to_pose.hpp"
#include "crsf_interfaces/action/move_to_named.hpp"
#include "crsf_interfaces/action/gripper_control.hpp"
#include "crsf_interfaces/action/emergency_stop.hpp"

class RobotInterface : public rclcpp::Node
{
public:
  RobotInterface();
  void initMoveGroups();

private:
  using MoveToPose = crsf_interfaces::action::MoveToPose;
  using MoveToNamed = crsf_interfaces::action::MoveToNamed;
  using GripperControl = crsf_interfaces::action::GripperControl;
  using EmergencyStop = crsf_interfaces::action::EmergencyStop;

  using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;
  using GoalHandleMoveToNamed = rclcpp_action::ServerGoalHandle<MoveToNamed>;
  using GoalHandleGripper = rclcpp_action::ServerGoalHandle<GripperControl>;
  using GoalHandleEStop = rclcpp_action::ServerGoalHandle<EmergencyStop>;

  // MoveIt
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;

  // Action Servers
  rclcpp_action::Server<MoveToPose>::SharedPtr move_to_pose_server_;
  rclcpp_action::Server<MoveToNamed>::SharedPtr move_to_named_server_;
  rclcpp_action::Server<GripperControl>::SharedPtr gripper_server_;
  rclcpp_action::Server<EmergencyStop>::SharedPtr estop_server_;

  // Gripper HW Action Client
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_hw_client_;

  // ===== MoveToPose =====
  rclcpp_action::GoalResponse handleMoveToPoseGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveToPose::Goal> goal);

  void executeMoveToPose(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

  // ===== MoveToNamed =====
  rclcpp_action::GoalResponse handleMoveToNamedGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveToNamed::Goal> goal);

  void executeMoveToNamed(const std::shared_ptr<GoalHandleMoveToNamed> goal_handle);

  // ===== Gripper =====
  rclcpp_action::GoalResponse handleGripperGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GripperControl::Goal> goal);

  void executeGripper(const std::shared_ptr<GoalHandleGripper> goal_handle);

  bool sendGripperCommand(double position, double effort);

  // ===== Emergency Stop =====
  rclcpp_action::GoalResponse handleEStopGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const EmergencyStop::Goal> goal);

  void executeEmergencyStop(const std::shared_ptr<GoalHandleEStop> goal_handle);
};
