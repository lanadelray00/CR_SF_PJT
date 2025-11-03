#include "cr_pjt/robot_interface.hpp"

RobotInterface::RobotInterface()
: Node("robot_interface")
{
  RCLCPP_INFO(this->get_logger(), "🚀 RobotInterface node initialized");
  
  // Emergency Stop Service
  emergency_stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "emergency_stop",
    std::bind(&RobotInterface::emergencyStopCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
}

void RobotInterface::initMoveGroups()
{
  auto node_shared = this->shared_from_this();

  arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node_shared, "arm");
  gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node_shared, "gripper");

  RCLCPP_INFO(this->get_logger(), "✅ MoveGroup interfaces initialized (arm, gripper)");
}

bool RobotInterface::moveToPose(double x, double y, double z,
                                double qx, double qy, double qz, double qw)
{
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;
  target_pose.orientation.x = qx;
  target_pose.orientation.y = qy;
  target_pose.orientation.z = qz;
  target_pose.orientation.w = qw;

  arm_group_->setPoseTarget(target_pose);
  arm_group_->setGoalPositionTolerance(0.01);
  arm_group_->setGoalOrientationTolerance(0.01);
  arm_group_->setPlanningTime(5.0);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto error_code = arm_group_->plan(plan);

  if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "✅ Planning success, executing...");
    arm_group_->execute(plan);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "❌ Planning failed (code: %d)", error_code.val);
    arm_group_->stop();               
    arm_group_->clearPoseTargets();
    return false;
  }

  return true;
}

void RobotInterface::openGripper()
{
  gripper_group_->setNamedTarget("open");
  gripper_group_->move();
  RCLCPP_INFO(this->get_logger(), "🤖 Gripper opened");
}

void RobotInterface::closeGripper()
{
  gripper_group_->setNamedTarget("close");
  gripper_group_->move();
  RCLCPP_INFO(this->get_logger(), "✊ Gripper closed");
}

void RobotInterface::moveHome()
{
  arm_group_->setNamedTarget("init");
  arm_group_->move();
  RCLCPP_INFO(this->get_logger(), "🏠 Moved to home pose");
}

void RobotInterface::moveToNamedPose(const std::string &pose_name)
{
  arm_group_->setNamedTarget(pose_name);
  arm_group_->move();
  RCLCPP_INFO(this->get_logger(), "📍 Moved to named pose: %s", pose_name.c_str());
}

void RobotInterface::emergencyStopCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  const std_srvs::srv::Trigger::Response::SharedPtr response)
{
  arm_group_->stop();
  gripper_group_->stop();
  response->success = true;
  response->message = "Emergency stop activated — all motion halted.";
  RCLCPP_WARN(this->get_logger(), "🛑 EMERGENCY STOP TRIGGERED!");
}
