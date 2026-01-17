import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

from crsf_interfaces.action import (
    MoveToPose,
    MoveToNamed,
    GripperControl,
    EmergencyStop,
)


class RobotInterfaceClient(Node):
    def __init__(self):
        super().__init__('robot_interface_client')
        self.get_logger().info("🧪 robot_interface_client (Action-based) initialized")

        # ===============================
        # Action Clients
        # ===============================
        self.move_to_pose_client = ActionClient(self, MoveToPose, 'move_to_pose')
        self.move_to_named_client = ActionClient(self, MoveToNamed, 'move_to_named')
        self.gripper_client = ActionClient(self, GripperControl, 'gripper_control')
        self.emergency_client = ActionClient(self, EmergencyStop, 'emergency_stop')

        # ===============================
        # FK Service & JointState
        # ===============================
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )

        self.current_position = None
        self.current_orientation = None

        self.wait_for_servers()

    # ======================================================
    # Wait for Action Servers
    # ======================================================
    def wait_for_servers(self):
        self.get_logger().info("⏳ Waiting for action servers...")

        self.move_to_pose_client.wait_for_server()
        self.move_to_named_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.emergency_client.wait_for_server()
        self.fk_client.wait_for_service()

        self.get_logger().info("✅ All action servers available")

    # ======================================================
    # MoveToPose Action
    # ======================================================
    def call_move_to_pose(self, x, y, z, qx, qy, qz, qw):
        goal = MoveToPose.Goal()
        goal.target_pose.position.x = float(x)
        goal.target_pose.position.y = float(y)
        goal.target_pose.position.z = float(z)
        goal.target_pose.orientation.x = float(qx)
        goal.target_pose.orientation.y = float(qy)
        goal.target_pose.orientation.z = float(qz)
        goal.target_pose.orientation.w = float(qw)

        send_goal_future = self.move_to_pose_client.send_goal_async(
            goal, feedback_callback=self.move_to_pose_feedback
        )
        send_goal_future.add_done_callback(self.move_to_pose_response)

    def move_to_pose_feedback(self, feedback):
        self.get_logger().info(f"📡 MoveToPose feedback: {feedback.feedback.state}")

    def move_to_pose_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ MoveToPose goal rejected")
            return

        self.get_logger().info("✅ MoveToPose goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.move_to_pose_result)

    def move_to_pose_result(self, future):
        result = future.result().result
        self.get_logger().info(f"🏁 MoveToPose result: {result.message}")

    def move_to_pose_and_wait(self, x, y, z, qx, qy, qz, qw):
        goal = MoveToPose.Goal()
        goal.target_pose.position.x = x
        goal.target_pose.position.y = y
        goal.target_pose.position.z = z
        goal.target_pose.orientation.x = qx
        goal.target_pose.orientation.y = qy
        goal.target_pose.orientation.z = qz
        goal.target_pose.orientation.w = qw
        return self.send_and_wait(self.move_to_pose_client, goal)


    # ======================================================
    # MoveToNamed Action
    # ======================================================
    def call_move_to_named(self, name: str):
        goal = MoveToNamed.Goal()
        goal.name = name

        future = self.move_to_named_client.send_goal_async(goal)
        future.add_done_callback(self.move_to_named_response)

    def move_to_named_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ MoveToNamed goal rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.move_to_named_result)

    def move_to_named_result(self, future):
        result = future.result().result
        self.get_logger().info(f"🏁 MoveToNamed result: {result.message}")

    def move_to_named_and_wait(self, name):
        goal = MoveToNamed.Goal()
        goal.name = name
        return self.send_and_wait(self.move_to_named_client, goal)


    # ======================================================
    # Gripper Action
    # ======================================================
    def call_gripper(self, position: float, effort: float = 5.0):
        goal = GripperControl.Goal()
        goal.position = float(position)
        goal.max_effort = float(effort)

        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(self.gripper_response)

    def gripper_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Gripper goal rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.gripper_result)

    def gripper_result(self, future):
        result = future.result().result
        self.get_logger().info(f"✊ Gripper result: {result.message}")

    def gripper_and_wait(self, position, effort=5.0):
        goal = GripperControl.Goal()
        goal.position = position
        goal.max_effort = effort
        return self.send_and_wait(self.gripper_client, goal)


    # ======================================================
    # Emergency Stop Action
    # ======================================================
    def call_emergency_stop(self):
        goal = EmergencyStop.Goal()
        goal.stop = True

        future = self.emergency_client.send_goal_async(goal)
        future.add_done_callback(self.emergency_response)

    def emergency_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ EmergencyStop rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.emergency_result)

    def emergency_result(self, future):
        result = future.result().result
        self.get_logger().warn(f"🛑 Emergency stop: {result.message}")

    # ======================================================
    # FK Calculation (unchanged)
    # ======================================================
    def joint_callback(self, msg):
        if not self.fk_client.service_is_ready():
            return

        request = GetPositionFK.Request()
        request.header.frame_id = 'world'
        request.fk_link_names = ['end_effector_link']
        robot_state = RobotState()
        robot_state.joint_state = msg
        request.robot_state = robot_state

        future = self.fk_client.call_async(request)
        future.add_done_callback(self.fk_response_callback)

    def fk_response_callback(self, future):
        try:
            response = future.result()
            if response.pose_stamped:
                pose = response.pose_stamped[0].pose
                self.current_position = [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                ]
                self.current_orientation = [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ]
        except Exception as e:
            self.get_logger().error(f"FK call failed: {e}")
    
    # ======================================================
    # Send_and_wait
    # ======================================================
    def send_and_wait(self, action_client, goal):
        future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected")
            return False

        # 2️⃣ 결과 대기
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        wrapped_result = result_future.result()
        status = wrapped_result.status 
        result = wrapped_result.result

        # 3️⃣ 상태 코드 확인 (가장 중요)
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"❌ Action failed (status={status}, message={getattr(result, 'message', '')})"
            )
            return False        

        # 4️⃣ result.success 확인 (MoveToPose / Gripper 등)
        if hasattr(result, "success") and not result.success:
            self.get_logger().error(
                f"❌ Action reported failure: {result.message}"
            )
            return False

        self.get_logger().info(f"✅ Action succeeded: {getattr(result, 'message', '')}")
        return True


def main():
    rclpy.init()
    node = RobotInterfaceClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
