#include <memory>

#include "cr_pjt/robot_interface.hpp"
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // 노드 생성
  auto node = rclcpp::Node::make_shared("move_position_only_node");

  // MoveGroupInterface 생성
  moveit::planning_interface::MoveGroupInterface arm_group(
    node,
    "arm"   // ← 네 로봇 planning group 이름
  );

  // 목표 위치 (translation only)
  geometry_msgs::msg::Pose target_pose;
  // target_pose.position.x = 0.30;
  // target_pose.position.y = 0.00;
  // target_pose.position.z = 0.25;


  // ❗ orientation은 아예 설정하지 않음
  // → MoveIt이 IK 가능한 orientation으로 처리

  // Pose target 설정
  arm_group.setPositionTarget(0.2, 0.05, 0.05);
  arm_group.setGoalPositionTolerance(0.05);
  arm_group.setGoalOrientationTolerance(0.05);

  // 계획 + 실행
  auto result = arm_group.move();

  if (result == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Move succeeded");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Move failed");
  }

  rclcpp::shutdown();
  return 0;
}
