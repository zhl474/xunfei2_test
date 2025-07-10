import cv2
import numpy as np

# 1. 加载针孔摄像头标定结果
def load_pinhole_calibration(calib_file):
    """加载针孔摄像头标定参数"""
    fs = cv2.FileStorage(calib_file, cv2.FILE_STORAGE_READ)
    
    # 获取相机内参矩阵
    camera_matrix_node = fs.getNode("camera_matrix")
    if camera_matrix_node.empty():
        raise ValueError("未找到相机内参矩阵")
    K = camera_matrix_node.mat()
    
    # 获取畸变系数
    dist_coeffs_node = fs.getNode("distortion_coefficients")
    if dist_coeffs_node.empty():
        # 可能是旧格式的标定文件
        dist_coeffs_node = fs.getNode("dist_coeffs") or fs.getNode("D")
    
    if dist_coeffs_node.empty():
        # 如果找不到畸变系数，使用零数组
        D = np.zeros((5, 1), dtype=np.float64)
        print("警告：未找到畸变系数，使用默认零值")
    else:
        D = dist_coeffs_node.mat().reshape(1, -1)  # 确保维度正确
        
    # 获取分辨率
    width = int(fs.getNode("image_width").real() or 640)
    height = int(fs.getNode("image_height").real() or 480)
    
    fs.release()
    return K, D, (width, height)

# 2. 针孔摄像头畸变校正
def undistort_frame(frame, K, D, resolution):
    """应用针孔畸变校正"""
    # 确保畸变系数的正确维度 (1x5)
    if D.size == 5:
        D = D.reshape(1, 5)
    elif D.size > 5:
        D = D[:5].reshape(1, 5)
    else:
        D = np.zeros((1, 5), dtype=np.float64)
    
    # 高效校正方法：使用映射图
    if not hasattr(undistort_frame, 'map1') or undistort_frame.resolution != resolution:
        # 缓存映射图，只有当分辨率改变时才重新计算
        new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
            K, D, resolution, 1, resolution
        )
        map1, map2 = cv2.initUndistortRectifyMap(
            K, D, None, new_camera_matrix, resolution, cv2.CV_16SC2
        )
        undistort_frame.map1 = map1
        undistort_frame.map2 = map2
        undistort_frame.resolution = resolution
        undistort_frame.new_camera_matrix = new_camera_matrix
    
    # 使用预计算的映射图进行快速校正
    undistorted = cv2.remap(
        frame, 
        undistort_frame.map1, 
        undistort_frame.map2, 
        cv2.INTER_LINEAR
    )
    
    return undistorted

# 初始化摄像头
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    print("无法打开摄像头！")
    exit()

# 读取第一帧
ret, frame = cap.read()
if not ret:
    print("无法读取摄像头画面！")
    cap.release()
    exit()

# 预设原始梯形区域坐标（示例坐标，根据实际场景调整）
# 格式为 [左上, 右上, 右下, 左下]
src_points = np.float32([[260, 230], [381, 230], [487, 335], [150, 335]])

dst_points = np.float32([[150, 230], [487, 230], [487, 335], [150, 335]])


# 计算透视变换矩阵
M = cv2.getPerspectiveTransform(src_points, dst_points)
print("透视变换矩阵：")
print(M)

CALIB_FILE = "/home/ucar/ucar_car/src/line_follow/camera_info/pinhole.yaml"  # 针孔标定文件路径
K, D, calib_resolution = load_pinhole_calibration(CALIB_FILE)

# 创建窗口
cv2.namedWindow("Original")
cv2.namedWindow("Transformed (Bird's-eye View)")

# 在原始画面上标记梯形区域
marked_frame = frame.copy()
for i, point in enumerate(src_points):
    cv2.circle(marked_frame, tuple(point.astype(int)), 8, (0, 0, 255), -1)
    cv2.line(marked_frame, 
             tuple(src_points[i].astype(int)), 
             tuple(src_points[(i+1) % 4].astype(int)), 
             (0, 255, 0), 2)

cv2.putText(marked_frame, "Press 'q' to exit", 
            (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 50, 200), 2)
cv2.putText(marked_frame, "Source points:", 
            (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
for i, point in enumerate(src_points):
    cv2.putText(marked_frame, f"P{i+1}: {point.astype(int)}", 
                (20, 90 + i*30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 255), 1)
RESOLUTION = (640, 480)
# 主循环
while True:
    # 读取当前帧
    ret, frame = cap.read()
    if not ret:
        break
    frame = undistort_frame(frame, K, D, RESOLUTION)
    
    # 在原始画面上显示标记点
    display_frame = frame.copy()
    for point in src_points:
        cv2.circle(display_frame, tuple(point.astype(int)), 5, (0, 0, 255), -1)
    
    # 应用透视变换
    transformed = cv2.warpPerspective(frame, M,(640,480))
    
    # 显示结果
    cv2.imshow("Original", display_frame)
    cv2.imshow("Transformed (Bird's-eye View)", transformed)
    
    # 退出条件
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.imwrite('original_frame.jpg', frame)
        cv2.imwrite('transformed_view.jpg', transformed)
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()