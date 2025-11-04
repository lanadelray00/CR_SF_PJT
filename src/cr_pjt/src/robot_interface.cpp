#include "cr_pjt/robot_interface.hpp"

using namespace std::chrono_literals;

// ROS2 node in
RobotInterface::RobotInterface()
: Node("robot_interface")
{
  RCLCPP_INFO(this->get_logger(), "🚀 RobotInterface node initialized");
  
  // Emergency Stop Service
  emergency_stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "emergency_stop",
    std::bind(&RobotInterface::emergencyStopCallback, this, std::placeholders::_1, std::placeholders::_2)
  );

  // ===== Gripper Action Client 초기화 =====
  gripper_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
    this, "/gripper_controller/gripper_cmd");

  while (!gripper_client_->wait_for_action_server(2s)) {
    RCLCPP_INFO(this->get_logger(), "⏳ Waiting for /gripper_controller/gripper_cmd action server...");
  }
  RCLCPP_INFO(this->get_logger(), "✅ Connected to GripperActionController");
}

// move group definition
void RobotInterface::initMoveGroups()
{
  auto node_shared = this->shared_from_this();

  arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node_shared, "arm");

  RCLCPP_INFO(this->get_logger(), "✅ MoveGroup interfaces initialized (arm)");
}

// arm move
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

  RCLCPP_INFO(this->get_logger(), "🟡 Planning path to (%.3f, %.3f, %.3f)", x, y, z);

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

// move to customized set point 
void RobotInterface::moveToNamedPose(const std::string &pose_name)
{
  arm_group_->setNamedTarget(pose_name); //(default : home, option : init)
  arm_group_->move();
  RCLCPP_INFO(this->get_logger(), "📍 Moved to named pose: %s", pose_name.c_str());
}

// Gripper open, close
bool RobotInterface::sendGripperCommand(double position, double effort)
{
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

  if (!gripper_client_->action_server_is_ready()) {
    RCLCPP_ERROR(this->get_logger(), "❌ Gripper action server not ready");
    return false;
  }

  auto goal_msg = GripperCommand::Goal();
  goal_msg.command.position = position;   // m 단위 (0.019=open, -0.01=close)
  goal_msg.command.max_effort = effort;   // 모터 토크 제한 (0=무제한)

  RCLCPP_INFO(this->get_logger(), "🚀 Sending gripper command (pos=%.3f)", position);

  auto future_goal_handle = gripper_client_->async_send_goal(goal_msg);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle)
      != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "❌ Failed to send gripper goal");
    return false;
  }

  auto goal_handle = future_goal_handle.get();
  if (!goal_handle) {
    RCLCPP_WARN(this->get_logger(), "⚠️ Gripper goal rejected");
    return false;
  }

  // 결과 대기
  auto result_future = gripper_client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future)
      != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "❌ Failed to get gripper result");
    return false;
  }

  auto result = result_future.get();
  RCLCPP_INFO(this->get_logger(), "✅ Gripper moved (reached=%d)", result.result->reached_goal);
  return result.result->reached_goal;
}

// Gripper open
void RobotInterface::openGripper()
{
  sendGripperCommand(0.019);  // SRDF 기준 open 위치 (-0.01, -0.005, 0, 0.005, 0.01, 0.01, 0.015, 0.019)
  RCLCPP_INFO(this->get_logger(), "🤖 Gripper opened");
}
// Gripper close
void RobotInterface::closeGripper(double position)
{
  sendGripperCommand(position);  // SRDF 기준 close 위치 (min : -0.01)
  RCLCPP_INFO(this->get_logger(), "✊ Gripper closed(pos=%.3f)", position);
}



void RobotInterface::emergencyStopCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  const std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (arm_group_)
  arm_group_->stop();
  
  if (gripper_client_ && gripper_client_->action_server_is_ready()) {
    RCLCPP_WARN(this->get_logger(), "🛑 Attempting to stop gripper motion (if active)");
  }
  response->success = true;
  response->message = "Emergency stop activated — all motion halted.";
  RCLCPP_WARN(this->get_logger(), "🛑 EMERGENCY STOP TRIGGERED!");
}
