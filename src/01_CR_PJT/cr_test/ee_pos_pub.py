import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK

class EEPositionNode(Node):
    def __init__(self):
        super().__init__('ee_position_node')

        # FK 서비스 클라이언트 생성
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ Waiting for /compute_fk service...')

        # /joint_states 구독
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.get_logger().info('✅ EE Position Node Started')

    def joint_callback(self, msg):
        if not msg.name or not msg.position:
            self.get_logger().warn("⚠️ Empty joint state message received")
            return

        # 요청 구성
        request = GetPositionFK.Request()
        request.header.frame_id = 'world'
        request.fk_link_names = ['end_effector_link']
        request.robot_state.joint_state.name = msg.name
        request.robot_state.joint_state.position = msg.position

        # 비동기 서비스 호출
        future = self.fk_client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.pose_stamped:
                pose = response.pose_stamped[0].pose
                self.get_logger().info(
                    f"🦾 EE Pose → x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}"
                )
            else:
                self.get_logger().warn("❌ FK failed or returned empty pose")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = EEPositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
