#include "test_pkg/robot_interface.hpp"

using namespace std::chrono_literals;

class RobotInterfaceTester : public rclcpp::Node
{
public:
  RobotInterfaceTester() : Node("robot_interface_tester")
  {
    RCLCPP_INFO(this->get_logger(), "🧪 RobotInterface Tester Node initialized");

    // 클라이언트 초기화
    move_to_pose_client_   = this->create_client<crsf_interfaces::srv::RobotInterfaceMovetopose>("move_to_pose");
    move_to_named_client_  = this->create_client<crsf_interfaces::srv::RobotInterfaceOneString>("move_to_named");
    gripper_client_        = this->create_client<crsf_interfaces::srv::RobotInterfaceOneString>("gripper_control");
    emergency_client_      = this->create_client<std_srvs::srv::Trigger>("emergency_stop");
  }

  void runTest()
  {
    waitForServers();

    // 1️⃣ Home 자세로 이동
    // callMoveToNamed("home");

    // 2️⃣ 그리퍼 열기
    callGripper("open");
    RCLCPP_INFO(this->get_logger(), "1");
    rclcpp::sleep_for(500ms);   // 0.5초 대기

    // 3️⃣ 특정 Pose로 이동
    callMoveToPose(0.251, 0.039, 0.012, 0.702, 0.083, -0.702, 0.083);
    // callMoveToPose(0.3, 0, 0.1, 0, 0, 0, 1);
    
    rclcpp::sleep_for(5000ms);   // 0.5초 대기

    // 4️⃣ 그리퍼 닫기
    callGripper("-0.003");
    RCLCPP_INFO(this->get_logger(), "2");
    rclcpp::sleep_for(500ms);   // 0.5초 대기

    // 5️⃣ 다시 init 자세로 이동
    callMoveToNamed("ground_2");
    RCLCPP_INFO(this->get_logger(), "3");
    rclcpp::sleep_for(3000ms);   // 0.5초 대기

    // 1️⃣ Home 자세로 이동
    callMoveToNamed("home");
    RCLCPP_INFO(this->get_logger(), "4");
    rclcpp::sleep_for(2000ms);   // 0.5초 대기

    // 2️⃣ 그리퍼 열기
    callGripper("open");
    RCLCPP_INFO(this->get_logger(), "5");

    // 6️⃣ (테스트 종료 후) E-Stop 실행
    // callEmergencyStop();

    RCLCPP_INFO(this->get_logger(), "✅ All service tests completed.");
  }

private:
  // Wait for all service servers
  void waitForServers()
  {
    RCLCPP_INFO(this->get_logger(), "⏳ Waiting for all service servers...");
    move_to_pose_client_->wait_for_service();
    move_to_named_client_->wait_for_service();
    gripper_client_->wait_for_service();
    emergency_client_->wait_for_service();
    RCLCPP_INFO(this->get_logger(), "✅ All service servers available.");
  }

  void callMoveToPose(double x, double y, double z, double qx, double qy, double qz, double qw)
  {
    auto req = std::make_shared<crsf_interfaces::srv::RobotInterfaceMovetopose::Request>();
    req->x = x; req->y = y; req->z = z;
    req->qx = qx; req->qy = qy; req->qz = qz; req->qw = qw;
    auto result = move_to_pose_client_->async_send_request(req);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = result.get();
      RCLCPP_INFO(this->get_logger(), "🤖 move_to_pose: x=%.3f, y=%.3f, z=%.3f", x, y, z);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to get response");
    }
  }

  void callMoveToNamed(const std::string &name)
  {
    auto req = std::make_shared<crsf_interfaces::srv::RobotInterfaceOneString::Request>();
    req->command = name;
    auto result = move_to_named_client_->async_send_request(req);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == 
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = result.get();
      RCLCPP_INFO(this->get_logger(), "📍 move_to_named ( %s )", name.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "❌ Invalid pose name (use: 'home' or 'init')");
    }
  }

  void callGripper(const std::string &cmd)
  {
    auto req = std::make_shared<crsf_interfaces::srv::RobotInterfaceOneString::Request>();
    req->command = cmd;
    auto result = gripper_client_->async_send_request(req);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = result.get();
      RCLCPP_INFO(this->get_logger(), "🦾 gripper_control (%s): %s", 
      cmd.c_str(), response->message.c_str());
    }
    else 
    {
      RCLCPP_WARN(this->get_logger(), "❌ Failed to get response");
    }
  }

  void callEmergencyStop()
  {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = emergency_client_->async_send_request(req);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(this->get_logger(), "🛑 emergency_stop: %s", result.get()->message.c_str());
    }
  }

  // 클라이언트 핸들
  rclcpp::Client<crsf_interfaces::srv::RobotInterfaceMovetopose>::SharedPtr move_to_pose_client_;
  rclcpp::Client<crsf_interfaces::srv::RobotInterfaceOneString>::SharedPtr move_to_named_client_;
  rclcpp::Client<crsf_interfaces::srv::RobotInterfaceOneString>::SharedPtr gripper_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr emergency_client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto tester = std::make_shared<RobotInterfaceTester>();
  tester->runTest();
  rclcpp::shutdown();
  return 0;
}

