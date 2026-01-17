#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import Bool
import threading


from robot_interface_client import RobotInterfaceClient
from ArUco_coord_extractor import MarkerPoseProcessor

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('Openmanipulator_pick_and_place_node')

        # Robot
        self.robot = RobotInterfaceClient()
        self.get_logger().info("🚀 Moving to initial pose: ground_2")
        success = self.robot.move_to_named_and_wait("ground_2")
        if not success:
            self.get_logger().error("❌ Failed to move to ground_2 at startup")
            raise RuntimeError("Initial pose move failed")
        
        # Marker Processor
        self.processor = MarkerPoseProcessor(self.robot)

        # Camera
        self.cap = cv2.VideoCapture('/dev/video3')
        if not self.cap.isOpened():
            self.get_logger().error("Camera open failed")
            raise RuntimeError("Camera open failed")

        # ★ ROS timer
        self.timer = self.create_timer(1.0 / 30.0, self.loop)
        
        self.start_requested = False

        self.is_executing = False

        self.camera_running = True
        threading.Thread(target=self.camera_loop, daemon=True).start()

        self.get_logger().info("🟢 Pick & Place node started")
        self.get_logger().info("👉 Waiting for /pick_and_place/start trigger")

 ########################################################################
        # Trigger
        self.trigger_sub = self.create_subscription(
            Bool,
            '/pick_and_place/start',
            self.trigger_cb,
            10
        )
    # Trigger method
    def trigger_cb(self, msg):
        if msg.data:
            self.get_logger().info("▶ Start trigger received")
            self.start_requested = True
            self.processor.start_recording()
########################################################################

    def camera_loop(self):
        while self.camera_running:
            ret, frame = self.cap.read()
            if not ret:
                continue

            self.processor.process_frame(frame)
            cv2.imshow("camera", frame)
            cv2.waitKey(1)


    def loop(self):
        if not self.start_requested:
            return
        if self.is_executing:
            return
        
        if self.processor.is_ready():
            self.start_requested = False
            self.is_executing = True
            pose = self.processor.get_refined_pose()
            self.execute_pick_and_place(pose)
            self.is_executing = False

    # ======================================================
    # Pick & Place Scenario
    # ======================================================
    def execute_pick_and_place(self, pose):
        x, y, z, qx, qy, qz, qw = pose

        self.get_logger().info(
            f"🎯 Target pose: "
            f"{x:.3f}, {y:.3f}, {z:.3f}"
        )

        # Pick
        self.robot.gripper_and_wait(0.019)
        success = self.robot.move_to_pose_and_wait(
            x, y, z, qx, qy, qz, qw
        )
        if not success:
            self.get_logger().error("❌ Move to target failed")
            return
        self.robot.gripper_and_wait(-0.004)

        # Place
        self.robot.move_to_named_and_wait("pick_1")
        self.robot.move_to_named_and_wait("place_2")
        self.robot.move_to_named_and_wait("place_1") # place position
        self.robot.gripper_and_wait(0.019)
        self.robot.move_to_named_and_wait("ground_2") # return
        self.get_logger().info("✅ Pick & Place completed")


def main():
    rclpy.init()
    node = PickAndPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutdown requested")
        pass
    finally:
        node.get_logger().info("Cleaning up resources")
        node.timer.cancel()
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()