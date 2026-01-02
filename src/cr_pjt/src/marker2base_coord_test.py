import cv2
import numpy as np
import cv2.aruco as aruco
import threading
from collections import deque
import rclpy
from robot_interface_client import RobotInterfaceClient
from scipy.spatial.transform import Rotation
import signal, os


def run_aruco_detector(stop_event, robot):
    # === Camera / ArUco init ===
    cap = cv2.VideoCapture('/dev/video2')

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()

    data = np.load('/home/choigh/ws_cr_sf/src/cr_pjt/src/config/calib_data.npz')
    camera_matrix = data['mtx']
    dist_coeffs = data['dist']
    # set size of Marker
    marker_length = 0.08  # [m]
    robot.get_logger().info("📸 ArUco detector started")

    # === Hand–Eye result: T(c→g) ===
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
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, marker_length, camera_matrix, dist_coeffs
            )

            for i in range(len(ids)):
                # === T(t→c) ===
                rvec = rvecs[i]
                tvec = tvecs[i].reshape(3, 1)

                R_target2cam, _ = cv2.Rodrigues(rvec)

                T_target2cam = np.eye(4)
                T_target2cam[:3, :3] = R_target2cam
                T_target2cam[:3,  3] = tvec.reshape(3)

                # === T(g→b) ===
                pose = robot.current_position           # [x,y,z]
                orient = robot.current_orientation      # [roll,pitch,yaw] (rad)

                if orient is None or len(orient) != 3:
                    continue

                R_gripper2base = Rotation.from_euler(
                    'xyz', orient, degrees=False
                ).as_matrix()

                T_gripper2base = np.eye(4)
                T_gripper2base[:3, :3] = R_gripper2base
                T_gripper2base[:3,  3] = np.array(pose)

                # === Chain ===
                T_cam2base = T_gripper2base @ T_cam2gripper
                T_target2base = T_cam2base @ T_target2cam

                bx, by, bz = T_target2base[:3, 3]

                # === Display ===
                aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.putText(
                    frame,
                    f"ID {ids[i][0]}  X={bx:.3f}  Y={by:.3f}  Z={bz:.3f} (base)",
                    (20, 30 + 30*i),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

                robot.get_logger().info(
                    f"[Marker {ids[i][0]}] base coord: "
                    f"x={bx:.3f}, y={by:.3f}, z={bz:.3f}"
                )

        cv2.imshow("Aruco Base Coordinate", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC
            stop_event.set()
            os.kill(os.getpid(), signal.SIGINT)
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    rclpy.init()
    robot = RobotInterfaceClient()

    stop_event = threading.Event()
    th = threading.Thread(target=run_aruco_detector, args=(stop_event, robot))
    th.start()

    try:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    stop_event.set()
    th.join()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
