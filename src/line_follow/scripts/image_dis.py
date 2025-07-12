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

# 3. 主程序
if __name__ == "__main__":
    # 配置参数（根据实际修改）
    CALIB_FILE = "/home/ucar/ucar_car/src/line_follow/camera_info/pinhole.yaml"  # 针孔标定文件路径
    CAMERA_DEVICE = 0  # 摄像头设备号，/dev/video0对应0
    RESOLUTION = (640, 480)  # 视频分辨率
    
    # 加载标定参数
    try:
        K, D, calib_resolution = load_pinhole_calibration(CALIB_FILE)
        print("标定参数加载成功")
        print(f"相机内参 K:\n{K}")
        print(f"畸变系数 D: {D.flatten()}")
        print(f"标定分辨率: {calib_resolution}")
        
        # 如果标定分辨率与使用分辨率不同，提示警告
        if calib_resolution != RESOLUTION:
            print(f"警告：标定分辨率 {calib_resolution} 与使用分辨率 {RESOLUTION} 不同！")
            print("这可能导致校正效果不佳")
    
    except Exception as e:
        print(f"加载标定文件失败: {e}")
        # 使用默认参数（无校正）
        K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]], dtype=np.float64)
        D = np.zeros((1, 5), dtype=np.float64)
        print("使用默认无畸变参数")
    
    # 打开摄像头
    cap = cv2.VideoCapture(CAMERA_DEVICE)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, RESOLUTION[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])
    
    # 创建显示窗口
    cv2.namedWindow("原始图像", cv2.WINDOW_NORMAL)
    cv2.namedWindow("校正图像", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("原始图像", RESOLUTION[0]//2, RESOLUTION[1]//2)
    cv2.resizeWindow("校正图像", RESOLUTION[0]//2, RESOLUTION[1]//2)
    
    print("按ESC键退出程序")
    
    # 主循环
    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法获取视频帧")
            break
        
        # 应用畸变校正
        undistorted = undistort_frame(frame, K, D, RESOLUTION)
        
        # 显示结果
        cv2.imshow("原始图像", frame)
        cv2.imshow("校正图像", undistorted)
        
        # 按ESC键退出
        if cv2.waitKey(1) == 27:
            break
    
    # 清理资源
    cap.release()
    cv2.destroyAllWindows()
    print("程序已退出")