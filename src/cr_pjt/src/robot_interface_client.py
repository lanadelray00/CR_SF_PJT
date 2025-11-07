import rclpy
from rclpy.node import Node
from crsf_interfaces.srv import RobotInterfaceMovetopose, RobotInterfaceOneString
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
        self.move_to_named_client = self.create_client(RobotInterfaceOneString, 'move_to_named')
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
        for c in [self.move_to_pose_client, self.move_to_named_client, self.gripper_client, self.emergency_client, self.fk_client]:
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


    def call_move_to_named(self, name):
        req = RobotInterfaceOneString.Request()
        req.command = name
        future = self.move_to_named_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f"✅ move_to_named result: {future.result().message}")
        else:
            self.get_logger().error("❌ Failed to get call_move_to_named response")


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

                # quaternion → euler 변환
                r = R.from_quat([qx, qy, qz, qw])
                roll, pitch, yaw = r.as_euler('xyz', degrees=False)

                # 업데이트
                self.current_position = [x, y, z]
                self.current_orientation = [roll, pitch, yaw]

            else:
                self.get_logger().warn("No FK result returned.")
        except Exception as e:
            self.get_logger().error(f"FK call failed: {e}")