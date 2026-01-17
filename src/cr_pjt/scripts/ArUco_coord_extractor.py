import cv2
import numpy as np
import cv2.aruco as aruco
from scipy.spatial.transform import Rotation
import math
import tf_transformations
from ament_index_python.packages import get_package_share_directory
import os

class MarkerPoseProcessor:
    def __init__(self, robot):

        # Camera Calibration
        package_share = get_package_share_directory('cr_pjt')
        calib_path = os.path.join(package_share, 'config', 'calib_data.npz')
        data = np.load(calib_path)
        
        self.camera_matrix = data['mtx']
        self.dist_coeffs = data['dist']
        self.marker_length = 0.025

        # ArUco
        self.aruco_dict = aruco.getPredefinedDictionary(
            aruco.DICT_4X4_50
        )
        self.parameters = aruco.DetectorParameters_create()

        # Eye-in-hand calibration
        R_cam2gripper = np.array([
            [-0.09837893,  0.16064060,  0.98209785],
            [-0.99123926, -0.10321322, -0.08241218],
            [ 0.08812674, -0.98160156,  0.16938727]
        ])
        t_cam2gripper = np.array([
            -0.05113446,
            -0.00675610,
            0.04876112
        ])
        self.T_cam2gripper = np.eye(4)
        self.T_cam2gripper[:3, :3] = R_cam2gripper
        self.T_cam2gripper[:3, 3] = t_cam2gripper

        # Recording buffer
        self.buffer = []
        self.buffer_size = 30
        self.recording = False

        # Robot Object
        self.robot = robot

    # ======================================================
    # Trigger
    # ======================================================
    def start_recording(self):
        # buffer clear 명시
        if self.recording:
            return
        self.buffer.clear()
        self.recording = True
    
    def is_recording(self):
        # 외부 상태 확인용
        return self.recording

    # Robot pose
    def get_T_gripper2base(self) -> np.ndarray:
        pose = self.robot.current_position
        orient = self.robot.current_orientation  # (qx, qy, qz, qw)

        if pose is None or orient is None:
            return None

        R_gripper2base = Rotation.from_quat(orient).as_matrix()
        T_gripper2base = np.eye(4)
        T_gripper2base[:3, :3] = R_gripper2base
        T_gripper2base[:3,  3] = np.array(pose)
        return T_gripper2base
    # ======================================================
    # CV update (frame 단위)
    # ======================================================
    def process_frame(self, frame):
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.parameters
        )

        if ids is None:
            return

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_length,
            self.camera_matrix,
            self.dist_coeffs
        )

        for i in range(len(ids)):
            aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], self.marker_length)

            # T_target2cam
            rvec, tvec = rvecs[i], tvecs[i]
            t_target2cam = tvec.reshape(3, 1)
            R_target2cam, _ = cv2.Rodrigues(rvec)

            T_target2cam = np.eye(4)
            T_target2cam[:3, :3] = R_target2cam
            T_target2cam[:3,  3] = t_target2cam.reshape(3)

            # T_gripper2base
            T_gripper2base = self.get_T_gripper2base()
            if T_gripper2base is None:
                return

            # T_target2base
            T_cam2base = T_gripper2base @ self.T_cam2gripper
            T_target2base = T_cam2base @ T_target2cam

            pos = T_target2base[:3, 3]
            quat = Rotation.from_matrix(
                T_target2base[:3, :3]
            ).as_quat()
            
            x, y, z, qx, qy, qz, qw = (*pos, *quat)
            if self.recording:
                self.buffer.append((*pos, *quat))

            # 화면 표시용 텍스트
            cX, cY = int(corners[i][0][0][0]), int(corners[i][0][0][1])
            cv2.putText(frame, f"ID:{ids[i][0]} X={x:.3f}m Y={y:.3f}m Z={z:.3f}m", (cX, cY - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            if self.recording and len(self.buffer) >= self.buffer_size:
                self.recording = False
                break
    # ======================================================
    # Result
    # ======================================================
    def is_ready(self):
        return len(self.buffer) == self.buffer_size

    def get_refined_pose(self):
        # outlier 방어 (간단한 z-score)
        data = np.array(self.buffer)
        mean = np.mean(data[:, :3], axis=0)
        std = np.std(data[:, :3], axis=0) + 1e-6

        z_scores = np.abs((data[:, :3] - mean) / std)
        mask = np.all(z_scores < 2.5, axis=1)
        filtered = data[mask]

        xs, ys, zs = filtered[:, 0], filtered[:, 1], filtered[:, 2]

        mean_x = float(np.mean(xs))
        mean_y = float(np.mean(ys))
        mean_z = float(np.mean(zs)) + 0.01

        pitch = math.pi / 2
        yaw = math.atan2(mean_y, mean_x)

        q = tf_transformations.quaternion_from_euler(
            0.0, pitch, yaw
        )

        # [ADDED] buffer clear는 여기서 책임
        self.buffer.clear()

        return mean_x, mean_y, mean_z, *q