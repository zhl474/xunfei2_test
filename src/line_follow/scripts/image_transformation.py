import cv2
import numpy as np

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0
        
    def compute(self, error):
        self.integral += error
        integral_limited = np.clip(self.integral, -1000, 1000)
        
        derivative = error - self.last_error
        output = (self.Kp * error) + (self.Ki * integral_limited) + (self.Kd * derivative)
        
        self.last_error = error
        return output
    
    def reset(self):
        self.integral = 0
        self.last_error = 0

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

# 3. 获取车道线起始位置（直方图法）
def get_linebase(binary_mask):
    """通过直方图找到车道线的起始位置"""
    height, width = binary_mask.shape[:2]
    
    # 计算下半部分图像的直方图（列方向求和）
    histogram = np.sum(binary_mask[height//2:, :], axis=0)
    
    # 找到直方图峰值位置
    midpoint = len(histogram) // 2
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    
    return leftx_base, rightx_base

# 4. 滑动窗口搜索车道线
def sliding_window_search(binary_mask, leftx_base, rightx_base):
    """使用滑动窗口搜索车道线像素"""
    height, width = binary_mask.shape
    
    # 滑动窗口参数
    nwindows = 9  # 窗口数量
    window_height = height // nwindows  # 每个窗口的高度
    margin = 100  # 窗口宽度的一半
    minpix = 50   # 重新定位窗口所需的最小像素数
    
    # 创建输出图像（三通道）
    out_img = np.dstack((binary_mask, binary_mask, binary_mask)) * 255
    
    # 初始化当前位置
    leftx_current = leftx_base
    rightx_current = rightx_base
    
    # 存储车道线像素的索引
    left_lane_inds = []
    right_lane_inds = []
    
    # 获取所有非零像素的位置
    nonzero = binary_mask.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    
    # 遍历所有窗口
    for window in range(nwindows):
        # 计算窗口边界
        win_y_low = height - (window + 1) * window_height
        win_y_high = height - window * window_height
        
        # 左车道线窗口边界
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        
        # 右车道线窗口边界
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        
        # 在输出图像上绘制窗口
        cv2.rectangle(out_img, (int(win_xleft_low), int(win_y_low)), 
                      (int(win_xleft_high), int(win_y_high)), (0, 255, 0), 2)
        cv2.rectangle(out_img, (int(win_xright_low), int(win_y_low)), 
                      (int(win_xright_high), int(win_y_high)), (0, 255, 0), 2)
        
        # 找到窗口内的非零像素
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
        
        # 添加这些索引到列表中
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        
        # 如果找到足够的像素，重新计算窗口中心
        if len(good_left_inds) > minpix:
            leftx_current = int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = int(np.mean(nonzerox[good_right_inds]))
    
    # 连接数组（将列表中的索引连接成一维数组）
    if left_lane_inds:
        left_lane_inds = np.concatenate(left_lane_inds)
    if right_lane_inds:
        right_lane_inds = np.concatenate(right_lane_inds)
    
    # 提取左右车道线像素位置
    leftx = nonzerox[left_lane_inds] if left_lane_inds.size > 0 else np.array([])
    lefty = nonzeroy[left_lane_inds] if left_lane_inds.size > 0 else np.array([])
    rightx = nonzerox[right_lane_inds] if right_lane_inds.size > 0 else np.array([])
    righty = nonzeroy[right_lane_inds] if right_lane_inds.size > 0 else np.array([])
    
    # 给车道线像素着色
    if leftx.size > 0 and lefty.size > 0:
        out_img[lefty, leftx] = [255, 0, 0]  # 红色表示左车道线
    if rightx.size > 0 and righty.size > 0:
        out_img[righty, rightx] = [0, 0, 255]  # 蓝色表示右车道线
    
    return leftx, lefty, rightx, righty, out_img

# 5. 多项式拟合车道线
def fit_polynomial(leftx, lefty, rightx, righty, img_shape):
    """对车道线像素进行二次多项式拟合"""
    height = img_shape[0]
    left_fit = right_fit = None
    left_fitx = right_fitx = ploty = None
    
    if leftx.size > 0:
        left_fit = np.polyfit(lefty, leftx, 2)
    if rightx.size > 0:
        right_fit = np.polyfit(righty, rightx, 2)
    
    if left_fit is not None or right_fit is not None:
        ploty = np.linspace(0, height - 1, height)
        if left_fit is not None:
            left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        if right_fit is not None:
            right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    
    return left_fit, right_fit, left_fitx, right_fitx, ploty

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
# 格式为 [左下, 右下, 左上, 右上]
src_points = np.float32([[56, 344], [600, 356], [114, 300], [540, 300]])

dst_points = np.float32([[40, 480], [600, 480], [40, 0], [600, 0]])


# 计算透视变换矩阵
M = cv2.getPerspectiveTransform(src_points, dst_points)
print("透视变换矩阵：")
print(M)

CALIB_FILE = "/home/ucar/ucar_car/src/line_follow/camera_info/pinhole.yaml"  # 针孔标定文件路径
K, D, calib_resolution = load_pinhole_calibration(CALIB_FILE)

# 创建窗口
# cv2.namedWindow("Original")
# cv2.namedWindow("Transformed (Bird's-eye View)")

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
    
    # 转换到 HSV 颜色空间
    hsv = cv2.cvtColor(transformed, cv2.COLOR_BGR2HSV)
    
    # 设置白色阈值
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 70, 255])
    
    # 创建掩膜
    mask = cv2.inRange(hsv, lower_white, upper_white)
    
    # 形态学操作去噪
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    binary_lane = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

     # 滑动窗口检测车道线
    leftx_base, rightx_base = get_linebase(binary_lane)
    leftx, lefty, rightx, righty, window_img = sliding_window_search(binary_lane, leftx_base, rightx_base)
    left_fit, right_fit, left_fitx, right_fitx, ploty = fit_polynomial(leftx, lefty, rightx, righty, binary_lane.shape)

# 绘制最终结果
    result = transformed.copy()
    if left_fitx is not None and right_fitx is not None:
        # # 绘制车道区域
        # pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        # pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        # pts = np.hstack((pts_left, pts_right))
        # cv2.fillPoly(result, np.int32([pts]), (0, 255, 0))
            
        # 绘制车道线
        for i in range(1, len(ploty)):
            cv2.line(result, (int(left_fitx[i-1]), int(ploty[i-1])), 
                    (int(left_fitx[i]), int(ploty[i])), (0,0,255), 3)
            cv2.line(result, (int(right_fitx[i-1]), int(ploty[i-1])), 
                    (int(right_fitx[i]), int(ploty[i])), (255,0,0), 3)
    

    # 显示结果
    cv2.imshow("Lane Detection", result)
    # cv2.imshow("Binary Lane Mask", binary_lane)  # 显示二值化图像
    cv2.imshow("Sliding Window Search", window_img)
    
    
    # 退出条件
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.imwrite('original_frame.jpg', frame)
        cv2.imwrite('transformed_view.jpg', transformed)
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()