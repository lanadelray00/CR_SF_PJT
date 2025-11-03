#include "cr_pjt/robot_interface.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotInterface>();

  node->initMoveGroups();  // ✅ 반드시 호출 (MoveGroup 초기화)

  node->moveHome();
  node->openGripper();
  
  if (!node->moveToPose(0.2, 0.0, 0.25))
    RCLCPP_WARN(node->get_logger(), "➡️ Skip: Unreachable pose 1");

  if (!node->moveToPose(0.2, 0.0, 0.2))
    RCLCPP_WARN(node->get_logger(), "➡️ Skip: Unreachable pose 2");

  if (!node->moveToPose(0.2, 0.0, 0.17))
    RCLCPP_WARN(node->get_logger(), "➡️ Skip: Unreachable pose 3");

  if (!node->moveToPose(0.15, 0.0, 0.2))
    RCLCPP_WARN(node->get_logger(), "➡️ Skip: Unreachable pose 4");

  node->closeGripper();

  rclcpp::shutdown();
  return 0;
}
