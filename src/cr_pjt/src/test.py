#!/usr/bin/env python3

import rclpy
from cr_pjt.scripts.robot_interface_client import RobotInterfaceClient

def main():
    rclpy.init()
    robot = RobotInterfaceClient()

    # 1️⃣ 반드시 초기자세
    robot.call_move_to_named("home")

    # 2️⃣ EE가 충분히 도달 가능한 좌표
    x = 0.16
    y = 0.0
    z = 0.20

    # orientation은 의미 없음
    robot.call_move_to_pose(
        x, y, z,
        0.0, 0.0, 0.0, 1.0
    )

    rclpy.shutdown()

if __name__ == "__main__":
    main()
