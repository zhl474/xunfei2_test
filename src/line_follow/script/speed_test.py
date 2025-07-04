#!/usr/bin/env python3
# coding=utf-8

import rospy
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import time

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

def fixed_threshold_binarization(image, threshold=160, max_value=255, threshold_type=cv2.THRESH_BINARY_INV):
    try:
        # 转换为灰度图像
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 只保留图像下半部分
        height = gray.shape[0]
        cropped_gray = gray[height//2+70:, :]  # 裁剪下半部分
        
        # 高斯滤波降噪
        blur = cv2.GaussianBlur(cropped_gray, (5,5), 0)
        
        # 固定阈值二值化
        _, binary = cv2.threshold(blur, threshold, max_value, threshold_type)
        
        return binary  
        
    except Exception as e:
        rospy.logwarn(f"二值化异常: {str(e)}")
        return np.zeros((1, 1), dtype=np.uint8)  # 返回空图像

        
def detect_center_line(binary_image):
    """
    使用OpenCV高效函数实现的赛道中线检测算法
    """
    height, width = binary_image.shape
    
    #使用投影法快速找到左右边界
    # 计算每行的最小和最大非零列索引
    nonzero_y, nonzero_x = np.nonzero(binary_image == 0)  # 找到所有黑色像素的位置
    
    # 如果没有找到任何黑色像素，使用默认中线
    if len(nonzero_y) == 0:
        center_line_x = width // 2
        visual_img = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
        cv2.line(visual_img, (center_line_x, 0), (center_line_x, height-1), (0, 255, 255), 2)
        return center_line_x, visual_img
    
    # 按行分组，找到每行的左右边界
    min_x = np.zeros(height, dtype=np.int32) + width  # 初始化为最大宽度
    max_x = np.zeros(height, dtype=np.int32)  # 初始化为0
    
    # 使用向量化操作更新每行的最小和最大x值
    np.minimum.at(min_x, nonzero_y, nonzero_x)
    np.maximum.at(max_x, nonzero_y, nonzero_x)
    
    print("min_x (每行最左黑点):")
    print(min_x)
    print("\nmax_x (每行最右黑点):")
    print(max_x)
    # 计算每行的中线
    centers = (min_x + max_x) // 2
    print("\ncenter (每行中间点):")
    print(centers)

    valid_mask = (min_x < max_x) & (min_x < width) & (max_x > 0)
    print("\nvalid_mask:")
    print(valid_mask)

    # 计算加权中线（下方行权重更高）
    weights = np.arange(height, 0, -1) * 2  # 行号越大（图像下方）权重越高
    weighted_sum = np.sum(centers[valid_mask] * weights[valid_mask])
    total_weight = np.sum(weights[valid_mask])
    center_line_x = int(weighted_sum / total_weight)
    print("\ncenterline:")
    print(center_line_x)

    # 创建可视化图像
    visual_img = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
    
    # 在可视化图像上绘制中线
    cv2.line(visual_img, (center_line_x, 0), (center_line_x, height-1), (0, 255, 255), 2)
    
    # 标记所有有效行的中线点
    for y in range(height):
        if valid_mask[y]:
            cv2.circle(visual_img, (centers[y], y), 1, (0, 0, 255), -1)
   
    # 环岛处理：检测左侧车道线上的角点
    handle_roundabout(binary_image, visual_img)    
    return center_line_x, visual_img

def handle_roundabout(binary_image, visual_img):
    """
    处理环岛场景：检测左侧车道线上的角点并拟合
    """
    height, width = binary_image.shape
    
    # 提取左侧车道线（假设左侧车道线在图像左半部分）
    left_lane_mask = np.zeros_like(binary_image)
    left_lane_mask[:, :width//2] = binary_image[:, :width//2]  # 只考虑左半部分
    left_lane_points = np.column_stack(np.where(left_lane_mask == 0))  # 左侧车道线的黑色像素点
    # print("\nleft_lane_points:")
    # print(left_lane_points)

    if len(left_lane_points) > 100:  # 如果左侧车道线足够多，进行角点检测
        gray = (left_lane_mask).astype(np.uint8)  # 转为灰度图
        gray = np.float32(gray)
        
        # 角点检测
        corners = cv2.cornerHarris(gray, blockSize=2, ksize=3, k=0.04)
        corners = cv2.dilate(corners, None)
        
        # 提取角点
        corner_threshold = 0.7*corners.max()
        corner_points = np.column_stack(np.where(corners > corner_threshold))
        print("\ncorner_point:")
        print(corner_points)

        if len(corner_points) > 0:
            # 使用角点进行线性拟合
            coefficients = np.polyfit(corner_points[:, 1], corner_points[:, 0], 1)  # 拟合直线方程 y = ax + b
            a, b = coefficients
            
            # 绘制拟合的直线
            x1, y1 = 0, int(b)
            x2, y2 = width//2, int(a * (width//2) + b)
            cv2.line(visual_img, (x1, y1), (x2, y2), (255, 0, 0), 2)  # 绘制拟合的左侧车道线
            
            # 标记角点
            for corner in corner_points:
                cv2.circle(visual_img, (int(corner[1]), int(corner[0])), 3, (0, 255, 0), -1)


# 初始化节点
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
params = {
    "帧宽度": cv2.CAP_PROP_FRAME_WIDTH,
    "帧高度": cv2.CAP_PROP_FRAME_HEIGHT,
    "帧率": cv2.CAP_PROP_FPS,
    "亮度": cv2.CAP_PROP_BRIGHTNESS,
    "对比度": cv2.CAP_PROP_CONTRAST,
    "饱和度": cv2.CAP_PROP_SATURATION,
    "色调": cv2.CAP_PROP_HUE,
    "增益": cv2.CAP_PROP_GAIN,
    "曝光": cv2.CAP_PROP_EXPOSURE,
    "自动白平衡": cv2.CAP_PROP_AUTO_WB,
    "白平衡色温": cv2.CAP_PROP_WB_TEMPERATURE,
    "蓝通道增益": cv2.CAP_PROP_WHITE_BALANCE_BLUE_U,  # 部分相机支持
    "红通道增益": cv2.CAP_PROP_WHITE_BALANCE_RED_V   # 部分相机支持
}

print("===== 当前摄像头参数 =====")
for name, prop_id in params.items():
    value = cap.get(prop_id)
    print(f"{name}: {value}")
if not cap.isOpened():
    rospy.logerr("打开摄像头失败")
rospy.loginfo("视觉巡线节点已启动!")

# pid = PIDController(Kp=0.03, Ki=0.001, Kd=0.015)

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        rospy.logerr("获取图片失败")
        continue

    cv2.imshow('frame',frame)
    cv2.waitKey(1)