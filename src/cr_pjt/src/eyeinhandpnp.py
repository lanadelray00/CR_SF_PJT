import cv2
import numpy as np
import cv2.aruco as aruco
import threading
from collections import deque
import rclpy
from robot_interface_client import RobotInterfaceClient
from scipy.spatial.transform import Rotation as R


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
    marker_length = 0.08
    robot.get_logger().info("📸 ArUco Detector Thread Started (ESC or Ctrl+C to exit)")

    # Hand-Eye Calibration Param
    R_cam2gripper = np.array([
        [-0.06043361, -0.01568252,  0.99804902],
        [-0.99776994, -0.02743505, -0.06084781],
        [ 0.02833577, -0.99950056, -0.01398955]
    ])
    t_cam2gripper = np.array([[-0.06823299], [-0.01539078], [0.05938006]])  # [m]

    R_ee2cam = R_cam2gripper.T
    t_ee2cam = -R_cam2gripper.T @ t_cam2gripper
    T_ee2cam = np.vstack((np.hstack((R_ee2cam, t_ee2cam)), [0,0,0,1]))

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
                
                rvec, tvec = rvecs[i], tvecs[i]       # 3x1, 3x1
                R_cam2marker, _ = cv2.Rodrigues(rvec) # 3x3
                T_cam2marker = np.vstack((np.hstack((R_cam2marker, tvec.T)), [0, 0, 0, 1]))

                # base2EE
                pose = robot.current_position
                orient = robot.current_orientation  # roll, pitch, yaw
                r = R.from_euler('xyz', orient, degrees=False)
                R_base2ee = r.as_matrix()
                t_base2ee = np.array(pose).reshape(3,1)
                T_base2ee = np.vstack((np.hstack((R_base2ee, t_base2ee)), [0,0,0,1]))
                T_base2cam = T_base2ee @ T_ee2cam

                # === 변환 ===
                T_base2marker = T_base2cam @ T_cam2marker
                bx, by, bz = T_base2marker[:3, 3]
                R_base2marker = T_base2marker[:3, :3]
                r_euler = R.from_matrix(R_base2marker)
                roll, pitch, yaw = r_euler.as_euler('xyz', degrees=True)
                # 정보 출력
                robot.get_logger().info(f"ID {ids[i][0]} | X={bx:.3f} Y={by:.3f} Z={bz:.3f}")
                
                if shared_data["record_mode"]:
                    shared_data["positions"].append((bx, by, bz))



                # 화면 표시용 텍스트
                cX, cY = int(corners[i][0][0][0]), int(corners[i][0][0][1])
                cv2.putText(frame, f"ID:{ids[i][0]} Z={bz:.2f}m", (cX, cY - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("Aruco Detection", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC to exit
            stop_event.set()  # ESC 키 누르면 스레드 종료
            break
        elif key == 32 and not shared_data["record_mode"]:
            robot.get_logger().info("🟢 Recording marker position for 60 frames...")
            shared_data["positions"].clear()
            shared_data["record_mode"] = True
            shared_data["frame_count"] = 0

        # 기록 중일 때 프레임 수 세기
        if shared_data["record_mode"]:
            shared_data["frame_count"] += 1
            if shared_data["frame_count"] >= 60:
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
        while not stop_event.is_set():
            if shared_data["trigger"]:
                shared_data["trigger"] = False  # 트리거 초기화

                if len(shared_data["positions"]) < 60:
                    robot.get_logger().info("⚠️ Not enough frames collected.")
                    continue

                # 60개 좌표의 평균 계산
                xs, ys, zs = zip(*shared_data["positions"])
                mean_x, mean_y, mean_z = np.mean(xs), np.mean(ys), np.mean(zs)
                robot.get_logger().info(f"🎯 Moving to averaged position: X={mean_x:.3f}, Y={mean_y:.3f}, Z={mean_z:.3f}")

                # 로봇 이동 명령
                robot.call_move_to_pose(mean_x, mean_y, mean_z, 0, 0, 0, 1)

            rclpy.spin_once(robot, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    stop_event.set()
    robot.get_logger().info("✅ All service tests completed.")
    aruco_thread.join()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
