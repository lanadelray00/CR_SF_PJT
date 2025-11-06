import rclpy
from rclpy.node import Node
from crsf_interfaces.srv import RobotInterfaceMovetopose, RobotInterfaceOneString
from std_srvs.srv import Trigger

class RobotInterfaceTester(Node):
    def __init__(self):
        super().__init__('robot_interface_pytester')
        self.get_logger().info("🧪 RobotInterface Tester Node initialized")

        # 서비스 클라이언트 생성
        self.move_to_pose_client = self.create_client(RobotInterfaceMovetopose, 'move_to_pose')
        self.move_to_named_client = self.create_client(RobotInterfaceOneString, 'move_to_named')
        self.gripper_client = self.create_client(RobotInterfaceOneString, 'gripper_control')
        self.emergency_client = self.create_client(Trigger, 'emergency_stop')

        self.wait_for_servers()

    def wait_for_servers(self):
        self.get_logger().info("⏳ Waiting for all service servers...")
        self.move_to_pose_client.wait_for_service()
        self.move_to_named_client.wait_for_service()
        self.gripper_client.wait_for_service()
        self.emergency_client.wait_for_service()
        self.get_logger().info("✅ All service servers available.")

    def call_move_to_pose(self, x, y, z, qx, qy, qz, qw):
        req = RobotInterfaceMovetopose.Request()
        req.x, req.y, req.z = float(x), float(y), float(z)
        req.qx, req.qy, req.qz, req.qw = float(qx), float(qy), float(qz), float(qw)
        future = self.move_to_pose_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f"🤖 move_to_pose result: {future.result().message}")
        else:
            self.get_logger().error("❌ Failed to get move_to_pose response")

    def call_move_to_named(self, name):
        req = RobotInterfaceOneString.Request()
        req.command = name
        future = self.move_to_named_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f"📍 move_to_named ({name}): {future.result().message}")
        else:
            self.get_logger().error("❌ Invalid pose name (use: 'home' or 'init')")

    def call_gripper(self, cmd):
        req = RobotInterfaceOneString.Request()
        req.command = cmd
        future = self.gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f"🦾 gripper_control ({cmd}): {future.result().message}")
        else:
            self.get_logger().warn(f"❌ Failed to get gripper response ({cmd})")

    def call_emergency_stop(self):
        req = Trigger.Request()
        future = self.emergency_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().warn(f"🛑 emergency_stop: {future.result().message}")

    def run_test(self):
        # 1️⃣ Home 자세로 이동
        self.call_move_to_named("home")

        # 2️⃣ 그리퍼 열기
        self.call_gripper("open")

        # 3️⃣ 특정 Pose로 이동
        self.call_move_to_pose(0.2, 0.0, 0.25, 0, 0, 0, 1)

        # 4️⃣ 그리퍼 닫기
        self.call_gripper("close")

        # 5️⃣ init 자세로 이동
        self.call_move_to_named("init")

        # 6️⃣ Emergency stop
        self.call_emergency_stop()

        self.get_logger().info("✅ All service tests completed.")


def main(args=None):
    rclpy.init(args=args)
    tester = RobotInterfaceTester()
    tester.run_test()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
