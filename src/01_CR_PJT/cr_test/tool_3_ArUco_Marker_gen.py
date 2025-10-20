import cv2
import cv2.aruco as aruco

# 최신 버전 (OpenCV 4.7 이상)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
save_path = "/home/choigh/WS/CR_SR_PJT/src/01_CR_PJT/cr_test/aruco_marker"
marker_number = 5 # 필요 마커 개수, 최대 50까지 가능

for i in range(marker_number):
    marker_id = i+1
    marker_size = 354  # 픽셀 단위
    marker_image = aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
    cv2.imwrite(f"{save_path}/marker_{marker_id}.png", marker_image)
