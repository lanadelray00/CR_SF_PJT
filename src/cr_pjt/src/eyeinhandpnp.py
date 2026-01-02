import cv2
import numpy as np
import cv2.aruco as aruco
import threading
from collections import deque
import rclpy
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
    marker_length = 0.03
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
                obj_points = np.array([
                    [-marker_length/2,  marker_length/2, 0],
                    [ marker_length/2,  marker_length/2, 0],
                    [ marker_length/2, -marker_length/2, 0],
                    [-marker_length/2, -marker_length/2, 0]
                ], dtype=np.float32)

                # 2D 이미지 좌표 (detectMarkers() 결과)
                img_points = corners[i][0].astype(np.float32)

                # --- SolvePnP으로 pose 계산 ---
                success, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    img_points,
                    camera_matrix,
                    dist_coeffs
                )

                # cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_length)

                # marker2cam
                rvec, tvec = rvecs[i], tvecs[i]       # 3x1, 3x1
                t_target2cam = tvec.reshape(3, 1)
                R_target2cam, _ = cv2.Rodrigues(rvec) # 3x3

                T_target2cam = np.eye(4)
                T_target2cam[:3, :3] = R_target2cam
                T_target2cam[:3,  3] = t_target2cam.reshape(3)

                # gripper2Base
                pose = robot.current_position
                orient = robot.current_orientation  # Quaternian (qx, qy, qz, qw)

                if orient is None or len(orient) != 3:
                    continue

                R_gripper2base = Rotation.from_euler('xyz', orient, degrees=False).as_matrix()

                T_gripper2base = np.eye(4)
                T_gripper2base[:3, :3] = R_gripper2base
                T_gripper2base[:3,  3] = np.array(pose)

                # === 변환 ===
                T_cam2base = T_gripper2base @ T_cam2gripper
                T_target2base = T_cam2base @ T_target2cam
                
                bx, by, bz = T_target2base[:3, 3]
                R_base2target = T_target2base[:3, :3]
                r_euler = Rotation.from_matrix(R_base2target)
                roll, pitch, yaw = r_euler.as_euler('xyz', degrees=True)
                
                ################# terminal 정보 출력
                robot.get_logger().info(f"ID {ids[i][0]} | X={bx:.3f} Y={by:.3f} Z={bz:.3f}")
                
                if shared_data["record_mode"]:
                    shared_data["positions"].append((bx, by, bz))



                # 화면 표시용 텍스트
                cX, cY = int(corners[i][0][0][0]), int(corners[i][0][0][1])
                cv2.putText(frame, f"ID:{ids[i][0]} X={bx:.2f}m Y={by:.2f}m Z={bz:.2f}m", (cX, cY - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

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
            shared_data["frame_count"] = 0

        # 기록 중일 때 프레임 수 세기
        if shared_data["record_mode"]:
            shared_data["frame_count"] += 1
            if shared_data["frame_count"] >= 30:
                shared_data["record_mode"] = False
                shared_data["trigger"] = True  # 평균 계산 트리거

    cap.release()
    cv2.destroyAllWindows()



def main():
    rclpy.init()
    robot = RobotInterfaceClient()

    shared_data = {
        "positions": deque(maxlen=60),
        "trigger": False,
        "record_mode": False,
        "frame_count": 0
    }

    stop_event = threading.Event()
    aruco_thread = threading.Thread(target=run_aruco_detector, args=(stop_event, shared_data, robot))
    aruco_thread.start()

    try:
        while True:
            if shared_data["trigger"]:
                shared_data["trigger"] = False  # 트리거 초기화

                if len(shared_data["positions"]) < 10:
                    robot.get_logger().info("⚠️ Not enough frames collected.")
                    continue

                # 60개 좌표의 평균 계산
                xs, ys, zs = zip(*shared_data["positions"])
                mean_x, mean_y, mean_z = np.mean(xs), np.mean(ys), np.mean(zs)
                
                # 로봇 이동 명령
                robot.get_logger().info(f"🎯 Coordinate acquired: X={mean_x:.3f}, Y={mean_y:.3f}, Z={mean_z:.3f}")
                robot.call_move_to_pose(mean_x, mean_y, mean_z, 0, 0, 0, 1)
                

            rclpy.spin_once(robot, timeout_sec=0.1)

    except KeyboardInterrupt:
        robot.get_logger().info("🛑 Program interrupted by user (ESC OR Ctrl+C)")

    stop_event.set()
    robot.get_logger().info("✅ All service tests completed.")
    aruco_thread.join()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
