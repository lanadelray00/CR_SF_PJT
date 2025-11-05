#include "cr_pjt/robot_interface.hpp"

using namespace std::chrono_literals;

// ROS2 node in
RobotInterface::RobotInterface()
: Node("robot_interface")
{
  RCLCPP_INFO(this->get_logger(), "🚀 RobotInterface node initialized");
  
  // moveToPose  service initialization
  movetopose_srv_ = this->create_service<crsf_interfaces::srv::RobotInterfaceMovetopose>(
    "move_to_pose",
    std::bind(&RobotInterface::moveToPoseCallback, this, std::placeholders::_1, std::placeholders::_2)
  );

  // moveToNamedPose (home, init) service initialization
  movetonamed_srv_ = this->create_service<crsf_interfaces::srv::RobotInterfaceOneString>(
    "move_to_named",
    std::bind(&RobotInterface::moveToNamedCallback, this, std::placeholders::_1, std::placeholders::_2)
  );

  // Gripper Action Client initialization
  gripper_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
    this, "/gripper_controller/gripper_cmd");

  while (!gripper_client_->wait_for_action_server(2s)) {
    RCLCPP_INFO(this->get_logger(), "⏳ Waiting for /gripper_controller/gripper_cmd action server...");
  }
  RCLCPP_INFO(this->get_logger(), "✅ Connected to GripperActionController");
  
  // gripper control service initialization
  gripper_ctrl_srv_ = this->create_service<crsf_interfaces::srv::RobotInterfaceOneString>(
    "gripper_control",
    std::bind(&RobotInterface::GripperctrlCallback, this, std::placeholders::_1, std::placeholders::_2)
  );

  // Emergency Stop Service initialization
  emergency_stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "emergency_stop",
    std::bind(&RobotInterface::emergencyStopCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
}


// Initialize MoveIt interface to control the 'arm'
void RobotInterface::initMoveGroups()
{
  auto node_shared = this->shared_from_this();

  arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node_shared, "arm");

  RCLCPP_INFO(this->get_logger(), "✅ MoveGroup interfaces initialized (arm)");
}

// arm move
void RobotInterface::moveToPoseCallback(
  const std::shared_ptr<crsf_interfaces::srv::RobotInterfaceMovetopose::Request> request,
  std::shared_ptr<crsf_interfaces::srv::RobotInterfaceMovetopose::Response> response)
{
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = request->x;
  target_pose.position.y = request->y;
  target_pose.position.z = request->z;
  target_pose.orientation.x = request->qx;
  target_pose.orientation.y = request->qy;
  target_pose.orientation.z = request->qz;
  target_pose.orientation.w = request->qw;

  arm_group_->setPoseTarget(target_pose);
  arm_group_->setGoalPositionTolerance(0.01);
  arm_group_->setGoalOrientationTolerance(0.01);
  arm_group_->setPlanningTime(5.0);

  RCLCPP_INFO(this->get_logger(), "🟡 Planning path to (%.3f, %.3f, %.3f)", 
              request->x, request->y, request->z);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto error_code = arm_group_->plan(plan);

  if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "✅ Planning success, executing...");
    arm_group_->execute(plan);
    response->success = true;
    response->message = "✅ MoveToPose executed successfully";
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "❌ Planning failed (code: %d)", error_code.val);
    arm_group_->stop();               
    arm_group_->clearPoseTargets();
    
    response->success = false;
    response->message = "❌ MoveToPose planning failed";
  }
}

// move to customized set point 
void RobotInterface::moveToNamedCallback(
  const std::shared_ptr<crsf_interfaces::srv::RobotInterfaceOneString::Request> request,
  std::shared_ptr<crsf_interfaces::srv::RobotInterfaceOneString::Response> response)
{
  std::string cmd = request->command;

  if (cmd == "home" || cmd == "init") {
    arm_group_->setNamedTarget(cmd); //(default : home, option : init)
    arm_group_->move();
    RCLCPP_INFO(this->get_logger(), "📍 Moved to named pose: %s", cmd.c_str());
    response->success = true;
    response->message = "✅ Moved to " + cmd + " pose";
  }
  else {
    RCLCPP_WARN(this->get_logger(), "⚠️ Unknown named pose: %s", cmd.c_str());
    response->success = false;
    response->message = "❌ Invalid pose name (use: 'home' or 'init')";
  }
}

//////////////////////////////////////gripper//////////////////////////////////////////////////////
// Gripper control connector to ros_control
bool RobotInterface::sendGripperCommand(double position, double effort)
{
  using GripperCommand = control_msgs::action::GripperCommand;

  if (!gripper_client_->action_server_is_ready()) {
    RCLCPP_ERROR(this->get_logger(), "❌ Gripper action server not ready");
    return false;
  }

  auto goal_msg = GripperCommand::Goal();
  goal_msg.command.position = position;
  goal_msg.command.max_effort = effort;

  RCLCPP_INFO(this->get_logger(), "🚀 Sending gripper command (pos=%.3f)", position);

  auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();

  send_goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<GripperCommand>::WrappedResult &result)
    {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        RCLCPP_INFO(this->get_logger(), "✅ Gripper moved successfully");
      else
        RCLCPP_WARN(this->get_logger(), "⚠️ Gripper action failed or was canceled");
    };

  gripper_client_->async_send_goal(goal_msg, send_goal_options);
  return true;
}

///////////////////////actual gripper controller, open & close & depth
void RobotInterface::GripperctrlCallback(
  const std::shared_ptr<crsf_interfaces::srv::RobotInterfaceOneString::Request> request,
  std::shared_ptr<crsf_interfaces::srv::RobotInterfaceOneString::Response> response)
{
  std::string cmd = request->command;
  RCLCPP_INFO(this->get_logger(), "📨 Received gripper command: %s", cmd.c_str());

  if (cmd == "open") {
    sendGripperCommand(0.019);
    response->success = true;
    response->message = "🤖 Gripper opened";
  } 
  else if (cmd == "close") {
    sendGripperCommand(-0.01);
    response->success = true;
    response->message = "✊ Gripper closed";
  }
  else {
    try {
      double pos = std::stod(cmd);  // 문자열을 double로 변환 시도
      sendGripperCommand(pos);
      response->success = true;
      response->message = "✊ Gripper moved to custom position";
    } catch (const std::invalid_argument &) {
      response->success = false;
      response->message = "❌ Invalid gripper command. Enter range -0.01 ~ 0.019";
      RCLCPP_WARN(this->get_logger(), "⚠️ Unknown command: %s", cmd.c_str());
    }
  }
}
//////////////////////////////////////gripper//////////////////////////////////////////////////////
// E-Stop
void RobotInterface::emergencyStopCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  const std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (arm_group_)
  arm_group_->stop();
  arm_group_->clearPoseTargets();
  rclcpp::sleep_for(500ms);
  
  if (gripper_client_ && gripper_client_->action_server_is_ready()) {
    gripper_client_->async_cancel_all_goals();
    RCLCPP_INFO(this->get_logger(), "✋ Sent cancel_all_goals() to gripper controller");
  }
  response->success = true;
  response->message = "Emergency stop activated — all motion halted.";
  RCLCPP_WARN(this->get_logger(), "🛑 EMERGENCY STOP TRIGGERED!");
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotInterface>();

  // MoveGroup 초기화 (필수)
  node->initMoveGroups();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}