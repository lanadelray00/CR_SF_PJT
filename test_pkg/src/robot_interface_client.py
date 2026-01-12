import rclpy
from rclpy.node import Node
from crsf_interfaces.srv import RobotInterfaceMovetopose, RobotInterfaceOneString, RobotInterfaceBusy
from std_srvs.srv import Trigger

from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R

# The robot_interface_client node communicates with the robot_interface node 
# to send service requests for controlling the OpenManipulator 
class RobotInterfaceClient(Node):
    def __init__(self):
        super().__init__('robot_interface_client')
        self.get_logger().info("🧪 robot_interface_client Node initialized")

        # 서비스 클라이언트 생성
        self.move_to_pose_client = self.create_client(RobotInterfaceMovetopose, 'move_to_pose')
        self.check_ik_client = self.create_client(RobotInterfaceMovetopose, 'check_ik')
        self.move_to_named_client = self.create_client(RobotInterfaceOneString, 'move_to_named')
        self.is_busy_cli = self.create_client(RobotInterfaceBusy, "is_busy")
        self.gripper_client = self.create_client(RobotInterfaceOneString, 'gripper_control')
        self.emergency_client = self.create_client(Trigger, 'emergency_stop')

        # ✅ FK 서비스 클라이언트 & JointState 구독자 추가
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        # 현재 EE pose 저장용
        self.current_position = None
        self.current_orientation = None  # roll, pitch, yaw

        self.wait_for_servers()

    def wait_for_servers(self):
        self.get_logger().info("⏳ Waiting for all service servers...")
        for c in [self.move_to_pose_client, self.check_ik_client, self.move_to_named_client, self.gripper_client, self.emergency_client, self.fk_client]:
            c.wait_for_service()
        self.get_logger().info("✅ All service servers available.")

    def call_move_to_pose(self, x, y, z, qx, qy, qz, qw):
        req = RobotInterfaceMovetopose.Request()
        req.x, req.y, req.z = float(x), float(y), float(z)
        req.qx, req.qy, req.qz, req.qw = float(qx), float(qy), float(qz), float(qw)
        future = self.move_to_pose_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f"✅ move_to_pose result: {future.result().message}")
        else:
            self.get_logger().error("❌ Failed to get move_to_pose response")

    def call_check_ik(self, x, y, z):
        req = RobotInterfaceMovetopose.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)

        future = self.check_ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f"✅ IK reachable at ({x:.3f}, {y:.3f}, {z:.3f})")
                return True
            else:
                self.get_logger().warn(
                    f"❌ IK NOT reachable at ({x:.3f}, {y:.3f}, {z:.3f}) : "
                    f"{future.result().message}"
                )
                return False
        else:
            self.get_logger().error("❌ Failed to call check_ik service")
            return False

    def call_move_to_named(self, name):
        req = RobotInterfaceOneString.Request()
        req.command = name
        future = self.move_to_named_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f"✅ move_to_named result: {future.result().message}")
        else:
            self.get_logger().error("❌ Failed to get call_move_to_named response")


    def is_robot_busy(self) -> bool:
        req = RobotInterfaceBusy.Request()
        future = self.is_busy_cli.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result().busy
        else:
            self.get_logger().warn("❌ Failed to call is_busy service")
            return True  # 안전하게 busy 처리



    def call_gripper(self, cmd):
        req = RobotInterfaceOneString.Request()
        req.command = cmd
        future = self.gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f"🤖 call_gripper result: {future.result().message}")
        else:
            self.get_logger().error("❌ Failed to get call_gripper response")


    def call_emergency_stop(self):
        req = Trigger.Request()
        future = self.emergency_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f"🛑 call_emergency_stop result: {future.result().message}")
            
    # ✅ FK 계산 콜백
    def joint_callback(self, msg):
        if not self.fk_client.service_is_ready():
            return

        request = GetPositionFK.Request()
        request.header.frame_id = 'world'   # base 좌표계 이름
        request.fk_link_names = ['end_effector_link']  # EE 링크 이름 수정 필요
        robot_state = RobotState()
        robot_state.joint_state = msg
        request.robot_state = robot_state

        future = self.fk_client.call_async(request)
        future.add_done_callback(self.fk_response_callback)

    def fk_response_callback(self, future):
        try:
            response = future.result()
            if len(response.pose_stamped) > 0:
                pose = response.pose_stamped[0].pose
                x, y, z = pose.position.x, pose.position.y, pose.position.z
                qx, qy, qz, qw = (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w
                )

                # 업데이트
                self.current_position = [x, y, z]
                self.current_orientation = [qx, qy, qz, qw]

            else:
                self.get_logger().warn("No FK result returned.")
        except Exception as e:
            self.get_logger().error(f"FK call failed: {e}")