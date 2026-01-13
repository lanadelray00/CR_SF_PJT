import cv2
import numpy as np
import cv2.aruco as aruco
import threading
from collections import deque
import math
import rclpy
import tf_transformations
from tf_transformations import quaternion_from_euler
from robot_interface_client import RobotInterfaceClient
from scipy.spatial.transform import Rotation
import signal, os


def run_aruco_detector(stop_event, shared_data, robot):
    # openCV & ArUco_marker initialization
    cv2.setLogLevel(0)
    cap = cv2.VideoCapture('/dev/video2')

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()

    data = np.load('/home/choigh/ws_cr_sf/src/cr_pjt/src/config/calib_data.npz')
    camera_matrix = data['mtx']
    dist_coeffs = data['dist']
    # set size of Marker
    marker_length = 0.025
    robot.get_logger().info("📸 ArUco Detector Thread Started (ESC or Ctrl+C to exit)")

    # Hand-Eye Calibration Param T(c→g)
    R_cam2gripper = np.array([
        [-0.09837893,  0.16064060,  0.98209785],
        [-0.99123926, -0.10321322, -0.08241218],
        [ 0.08812674, -0.98160156,  0.16938727]
    ])

    t_cam2gripper = np.array([
        -0.05113446,
        -0.00675610,
        0.04876112
    ]).reshape(3, 1)
    
    T_cam2gripper = np.eye(4)
    T_cam2gripper[:3, :3] = R_cam2gripper
    T_cam2gripper[:3,  3] = t_cam2gripper.reshape(3)
    
    # === Main loop ===
    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
 
        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

            for i in range(len(ids)):
                aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_length)

                # Tmarker2cam
                rvec, tvec = rvecs[i], tvecs[i]       # 3x1, 3x1
                t_target2cam = tvec.reshape(3, 1)
                R_target2cam, _ = cv2.Rodrigues(rvec) # 3x3

                T_target2cam = np.eye(4)
                T_target2cam[:3, :3] = R_target2cam
                T_target2cam[:3,  3] = t_target2cam.reshape(3)

                # Tgripper2Base
                pose = robot.current_position
                orient = robot.current_orientation  # Quaternian (qx, qy, qz, qw)

                if orient is None or len(orient) != 4:
                    continue

                R_gripper2base = Rotation.from_quat(orient).as_matrix()
                T_gripper2base = np.eye(4)
                T_gripper2base[:3, :3] = R_gripper2base
                T_gripper2base[:3,  3] = np.array(pose)

                # === coordinate change (Target to Base) ===
                T_cam2base = T_gripper2base @ T_cam2gripper
                T_target2base = T_cam2base @ T_target2cam
                
                bx, by, bz = T_target2base[:3, 3]
                R_base2target = T_target2base[:3, :3]
                quat = Rotation.from_matrix(R_base2target).as_quat()
                qx, qy, qz, qw = quat
                # r_euler = Rotation.from_matrix(R_base2target)
                # roll, pitch, yaw = r_euler.as_euler('xyz', degrees=True)

                ################# terminal 정보 출력
                # robot.get_logger().info(f"ID {ids[i][0]} | X={bx:.3f} Y={by:.3f} Z={bz:.3f}")
                # robot.get_logger().info(f"{roll, pitch, yaw}")
                
                if shared_data["record_mode"]:
                    shared_data["positions"].append((bx, by, bz, qx, qy, qz, qw))

                # 화면 표시용 텍스트
                cX, cY = int(corners[i][0][0][0]), int(corners[i][0][0][1])
                cv2.putText(frame, f"ID:{ids[i][0]} X={bx:.3f}m Y={by:.3f}m Z={bz:.3f}m", (cX, cY - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        cv2.imshow("Aruco Detection", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC to exit
            stop_event.set()  # ESC 키 누르면 스레드 종료
            cap.release()                # 카메라 해제
            cv2.destroyAllWindows()      # 모든 OpenCV 창 닫기
            os.kill(os.getpid(), signal.SIGINT)
            break
        elif key == 32 and not shared_data["record_mode"]: # space 누르면 기록 시작
            robot.get_logger().info("🟢 Recording marker position for 60 frames...")
            shared_data["positions"].clear()
            shared_data["record_mode"] = True

        # 기록 중일 때 프레임 수 세기
        if shared_data["record_mode"]:
            if len(shared_data["positions"]) >= 30:
                shared_data["record_mode"] = False
                shared_data["trigger"] = True  # 평균 계산 트리거

    cap.release()
    cv2.destroyAllWindows()



def main():
    rclpy.init()
    robot = RobotInterfaceClient()

    shared_data = {
        "positions": deque(maxlen=30),
        "trigger": False,
        "record_mode": False,
    }

    stop_event = threading.Event()
    aruco_thread = threading.Thread(target=run_aruco_detector, args=(stop_event, shared_data, robot))
    aruco_thread.start()

    try:
        while True:
            if shared_data["trigger"]:
                shared_data["trigger"] = False  # 트리거 초기화

                if len(shared_data["positions"]) < 30:
                    robot.get_logger().info("⚠️ Not enough frames collected.")
                    continue

                # 60개 좌표의 평균 계산
                xs, ys, zs, qx, qy, qz, qw = zip(*shared_data["positions"])
                mean_x, mean_y, mean_z = round(np.mean(xs), 3), round(np.mean(ys), 3), round(np.mean(zs), 3)
                mean_z = mean_z + float(0.01)
                yaw = math.atan2(mean_y, mean_x)
                pitch = math.pi / 2

                # 2️⃣ EE orientation 구성
                #   Pitch = +π/2 (지면 향하게), `````````````````````````````````````````````````````````````````````````Yaw = 마커 yaw 방향 정렬
                q_ee = tf_transformations.quaternion_from_euler(0, pitch, yaw)
                
                # 로봇 이동 명령
                robot.get_logger().info(f"🎯 {mean_x}, {mean_y}, {mean_z}, {q_ee[0]:.3f}, {q_ee[1]:.3f}, {q_ee[2]:.3f}, {q_ee[3]:.3f}")
                robot.call_move_to_pose(mean_x, mean_y, mean_z, q_ee[0], q_ee[1], q_ee[2], q_ee[3])
                # robot.call_move_to_pose(mean_x, mean_y, mean_z, 0, 0, 0, 1)

                

            rclpy.spin_once(robot, timeout_sec=0.1)

    except KeyboardInterrupt:
        robot.get_logger().info("🛑 Program interrupted by user (ESC OR Ctrl+C)")

    stop_event.set()
    robot.get_logger().info("✅ All service tests completed.")
    aruco_thread.join()
    rclpy.shutdown()


if __name__ == '__main__':
    main()