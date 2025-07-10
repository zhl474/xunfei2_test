import cv2
import numpy as np

# ========== 鼠标选择四个点的模块 ==========
def select_points_on_frame(frame):
    """
    显示图像并让用户用鼠标选择4个点
    返回：4个点的坐标列表 [[x1,y1], [x2,y2], ...]
    """
    points = []
    img_display = frame.copy()

    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(points) < 4:
                points.append([x, y])
                print(f"选择了第 {len(points)} 个点: ({x}, {y})")
                cv2.circle(img_display, (x, y), 5, (0, 255, 0), -1)
                cv2.putText(img_display, str(len(points)), (x+10, y-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                if len(points) == 4:
                    pts = np.array(points, np.int32)
                    cv2.polylines(img_display, [pts], True, (255, 0, 0), 2)
                cv2.imshow('选择四个点', img_display)

    cv2.namedWindow('选择四个点')
    cv2.setMouseCallback('选择四个点', mouse_callback)

    while True:
        cv2.imshow('选择四个点', img_display)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or len(points) == 4:
            break

    cv2.destroyWindow('选择四个点')
    return points

# ========== 原有功能代码 ==========
def load_pinhole_calibration(calib_file):
    fs = cv2.FileStorage(calib_file, cv2.FILE_STORAGE_READ)
    camera_matrix_node = fs.getNode("camera_matrix")
    if camera_matrix_node.empty():
        raise ValueError("未找到相机内参矩阵")
    K = camera_matrix_node.mat()
    
    dist_coeffs_node = fs.getNode("distortion_coefficients")
    if dist_coeffs_node.empty():
        dist_coeffs_node = fs.getNode("dist_coeffs") or fs.getNode("D")
    
    if dist_coeffs_node.empty():
        D = np.zeros((5, 1), dtype=np.float64)
        print("警告：未找到畸变系数，使用默认零值")
    else:
        D = dist_coeffs_node.mat().reshape(1, -1)
        
    width = int(fs.getNode("image_width").real() or 640)
    height = int(fs.getNode("image_height").real() or 480)
    fs.release()
    return K, D, (width, height)

def undistort_frame(frame, K, D, resolution):
    if D.size == 5:
        D = D.reshape(1, 5)
    elif D.size > 5:
        D = D[:5].reshape(1, 5)
    else:
        D = np.zeros((1, 5), dtype=np.float64)
    
    if not hasattr(undistort_frame, 'map1') or undistort_frame.resolution != resolution:
        new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(K, D, resolution, 1, resolution)
        map1, map2 = cv2.initUndistortRectifyMap(
            K, D, None, new_camera_matrix, resolution, cv2.CV_16SC2
        )
        undistort_frame.map1 = map1
        undistort_frame.map2 = map2
        undistort_frame.resolution = resolution
        undistort_frame.new_camera_matrix = new_camera_matrix
    
    undistorted = cv2.remap(frame, undistort_frame.map1, undistort_frame.map2, cv2.INTER_LINEAR)
    return undistorted

# ========== 主程序开始 ==========
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("无法打开摄像头！")
    exit()

# 加载标定参数
CALIB_FILE = "/home/ucar/ucar_car/src/line_follow/camera_info/pinhole.yaml"
K, D, calib_resolution = load_pinhole_calibration(CALIB_FILE)

# 先读取一帧用于选择点
ret, frame = cap.read()
if not ret:
    print("无法读取初始画面！")
    exit()

print("请在弹出窗口中选择4个点用于透视变换...")
points = select_points_on_frame(frame)

if len(points) != 4:
    print("未正确选择4个点，程序退出。")
    exit()

# 将选择的点转换为浮点数组
src_points = np.float32(points)
dst_points = np.float32([[40, 480], [600, 480], [40, 0], [600, 0]])

# 计算透视变换矩阵
M = cv2.getPerspectiveTransform(src_points, dst_points)
print("透视变换矩阵：")
print(M)

# 创建窗口
cv2.namedWindow("Original")
cv2.namedWindow("Transformed (Bird's-eye View)")

# 主循环
while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # 去畸变
    frame = undistort_frame(frame, K, D, calib_resolution)
    
    # 标记选择的点
    display_frame = frame.copy()
    for i, point in enumerate(src_points):
        cv2.circle(display_frame, tuple(point.astype(int)), 5, (0, 0, 255), -1)
        cv2.putText(display_frame, f"{i+1}", tuple(point.astype(int)+[10,-10]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.polylines(display_frame, [np.array(src_points, np.int32)], True, (255, 0, 0), 2)

    # 应用透视变换
    transformed = cv2.warpPerspective(frame, M, (640, 480))
    
    # 显示结果
    cv2.imshow("Original", display_frame)
    cv2.imshow("Transformed (Bird's-eye View)", transformed)
    
    # 按 q 退出并保存当前帧
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.imwrite('original_frame.jpg', frame)
        cv2.imwrite('transformed_view.jpg', transformed)
        break

# 清理资源
cap.release()
cv2.destroyAllWindows()