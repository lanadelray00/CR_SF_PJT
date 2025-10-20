import cv2
import cv2.aruco as aruco
import numpy as np
from collections import deque
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from tf_transformations import euler_from_quaternion
import threading
import os

# ArUco Marker coord
cap = cv2.VideoCapture('/dev/video2')
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
parameters = aruco.DetectorParameters()
parameters.adaptiveThreshConstant = 7      # 기본값: 7, 조명 강하면 ↑ 조정
parameters.minMarkerPerimeterRate = 0.02   # 기본값: 0.03, 작게 조정 시 작은 마커도 감지
parameters.maxMarkerPerimeterRate = 4.0    # 너무 큰 값 제한
parameters.polygonalApproxAccuracyRate = 0.03
parameters.minCornerDistanceRate = 0.05
parameters.minMarkerDistanceRate = 0.02    # 가까운 마커 구분도
parameters.minOtsuStdDev = 5.0             # 잡음 줄이기
parameters.perspectiveRemoveIgnoredMarginPerCell = 0.13


marker_length = 0.08

# rvecs, tvecs save
rvecs_list, tvecs_list = [], []
ee_pose = []
save_dir = "t_rvec_ee_list"
count = 1
os.makedirs(f"/home/choigh/WS/CR_SF_PJT/src/01_CR_PJT/cr_test/{save_dir}", exist_ok=True)
data = np.load('/home/choigh/WS/Test_Tools_code/calib_data.npz')
camera_matrix = data['mtx']
dist_coeffs = data['dist']


# ee pose
class FKClient(Node):
    def __init__(self):
        super().__init__('fk_client')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        # 최신 EE Pose 저장 변수
        self.current_position = None
        self.current_orientation = None
        
        # 서비스가 준비될 때까지 대기
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_fk service...')

    def joint_callback(self, msg):
        # FK 서비스 요청 생성
        request = GetPositionFK.Request()
        request.header.frame_id = 'world'  # 기준 좌표계
        request.fk_link_names = ['end_effector_link']    # FK를 계산할 링크 이름 (EE link 이름 확인 필요!)
        robot_state = RobotState()
        robot_state.joint_state = msg
        request.robot_state = robot_state

        # 비동기 서비스 호출
        future = self.fk_client.call_async(request)
        future.add_done_callback(self.fk_response_callback)

    def fk_response_callback(self, future):
        try:
            response = future.result()
            if len(response.pose_stamped) > 0:
                pose = response.pose_stamped[0].pose

                x, y, z = pose.position.x, pose.position.y, pose.position.z

                qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
                quat = [qx, qy, qz, qw]
                roll, pitch, yaw = euler_from_quaternion(quat)

                # print(f"EE position → x: {x:.3f}, y: {y:.3f}, z: {z:.3f}")
                # print(f"RPY → roll: {roll:.3f}, pitch: {pitch:.3f}, yaw: {yaw:.3f}")
                # 최신 EE Pose 저장 변수
                self.current_position = [x, y, z]
                self.current_orientation = [roll, pitch, yaw]

            else:
                print("No FK result returned.")
        except Exception as e:
            print(f"FK call failed: {e}")

if not rclpy.ok():
    rclpy.init()

fk_node = FKClient()

ros_thread = threading.Thread(target=rclpy.spin, args=(fk_node,), daemon=True)
ros_thread.start()


# ArUco Marker
while True:
    ret, frame = cap.read()
    key = cv2.waitKey(1) & 0xFF
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        for i in range(len(ids)):
            aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.02)

            # 화면 표시용 텍스트
            cX, cY = int(corners[i][0][0][0]), int(corners[i][0][0][1])
            

            cv2.putText(frame, f"ID:{ids[i][0]} Z={tvecs[i][0][2]:.2f}m", (cX, cY - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # 스페이스바 누르면 저장
            if key == 32:  # Spacebar
                tvecs_list.append([count, ids[i][0], tvecs[i][0][0], tvecs[i][0][1], tvecs[i][0][2]])
                rvecs_list.append([count, rvecs[i][0][0], rvecs[i][0][1], rvecs[i][0][2]])
                print(f"Num {count} | X={tvecs[i][0][0]:.3f}  Y={tvecs[i][0][1]:.3f}  Z={tvecs[i][0][2]:.3f}")
                
                if fk_node.current_position is not None:
                    x, y, z = fk_node.current_position
                    roll, pitch, yaw = fk_node.current_orientation
                    ee_pose.append([x, y, z, roll, pitch, yaw])
                    print(f"EE position → x: {x:.3f}, y: {y:.3f}, z: {z:.3f}")
                    print(f"RPY → roll: {roll:.3f}, pitch: {pitch:.3f}, yaw: {yaw:.3f}")
                count += 1

    cv2.imshow("Aruco Detection", frame)

    if key == 27:  # ESC to exit
        np.savez(os.path.join(f"/home/choigh/WS/CR_SF_PJT/src/01_CR_PJT/cr_test/{save_dir}", "t_rvec_ee_list.npz"),
             tvecs=np.array(tvecs_list),
             rvecs=np.array(rvecs_list),
             ee_pose=np.array(ee_pose))

        print(f"\nSaved {len(tvecs_list)} marker + EE samples to {save_dir}/t_rvec_ee_list.npz")
        break

cap.release()
cv2.destroyAllWindows()

# 🧹 ROS 노드 안전 종료
print("\nShutting down ROS node...")
fk_node.destroy_node()
rclpy.shutdown()

# 스레드도 종료 (데몬이지만 안전하게 join)
if ros_thread.is_alive():
    ros_thread.join(timeout=1.0)

print("✅ Node terminated cleanly.")