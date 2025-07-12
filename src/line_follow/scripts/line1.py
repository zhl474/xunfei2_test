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

def fixed_threshold_binarization(image, threshold=180, max_value=255, threshold_type=cv2.THRESH_BINARY_INV):
    try:
        # 转换为灰度图像
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 只保留图像下半部分
        height = gray.shape[0]
        cropped_gray = gray[height//2+30:, :]  # 裁剪下半部分
        
        # 高斯滤波降噪
        blur = cv2.GaussianBlur(cropped_gray, (5,5), 0)
        
        # 固定阈值二值化
        _, binary = cv2.threshold(blur, threshold, max_value, threshold_type)
        
        return binary  
        
    except Exception as e:
        rospy.logwarn(f"二值化异常: {str(e)}")
        return np.zeros((1, 1), dtype=np.uint8)  # 返回空图像

        
def detect_center_line(binary_image):
    height, width = binary_image.shape
    visual_img = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
    mid_x = width // 2

    # --- 步骤1：种子点选择 ---
    # 在图像底部扫描，找到左右车道线的种子点
    seed_y = height - 1  # 从最底部开始
    left_seed = None
    right_seed = None

    # 找左侧种子点（左半区第一个黑色像素）
    for x in range(mid_x - 1, 0, -1):
        if binary_image[seed_y, x] == 0:
            left_seed = (x, seed_y)
            break

    # 找右侧种子点（右半区第一个黑色像素）
    for x in range(mid_x, width - 1):
        if binary_image[seed_y, x] == 0:
            right_seed = (x, seed_y)
            break

    # --- 步骤2：种子生长法提取左右边线 ---
    def region_growing(seed, mask):
        boundary = []
        queue = [seed]
        mask[seed[1], seed[0]] = True  # 标记已访问

        # 8邻域方向
        directions = [(-1, -1), (-1, 0), (-1, 1),
                      (0, -1),          (0, 1),
                      (1, -1),  (1, 0), (1, 1)]

        while queue:
            x, y = queue.pop(0)
            boundary.append((x, y))

            # 检查8邻域
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if (0 <= nx < width and 0 <= ny < height and 
                    binary_image[ny, nx] == 0 and not mask[ny, nx]):
                    mask[ny, nx] = True
                    queue.append((nx, ny))
        return boundary

    # 初始化掩膜
    mask = np.zeros_like(binary_image, dtype=bool)
    
    # 提取左边线
    left_boundary = []
    if left_seed:
        left_boundary = region_growing(left_seed, mask)
    
    # 提取右边线
    right_boundary = []
    if right_seed:
        right_boundary = region_growing(right_seed, mask)

    # --- 步骤3：计算中线 ---
    center_line = []
    if left_boundary and right_boundary:
        # 按行对齐左右边界点
        left_dict = {y: x for x, y in left_boundary}
        right_dict = {y: x for x, y in right_boundary}
        common_ys = set(left_dict.keys()) & set(right_dict.keys())
        
        for y in sorted(common_ys):
            cx = (left_dict[y] + right_dict[y]) // 2
            center_line.append((cx, y))
    else:
        # 回退到默认中线
        center_line = [(mid_x, y) for y in range(height)]

    # --- 可视化 ---
    for x, y in left_boundary:
        visual_img[y, x] = (255, 0, 0)  # 左边线：红色
    for x, y in right_boundary:
        visual_img[y, x] = (0, 0, 255)  # 右边线：蓝色
    for x, y in center_line:
        visual_img[y, x] = (0, 255, 0)  # 中线：绿色

    return center_line, visual_img

def detect_stopping_line(binary_image):
    height, width = binary_image.shape
    # 检查图像底部10%的区域
    stop_section = binary_image[int(height * 0.99):, :]
    
    for row in stop_section:
        black_pixels = np.sum(row == 0)  # 统计黑色像素数量
        if black_pixels > width * 0.25:   # 黑色像素超过80%
            return True
    return False

# 初始化节点
rospy.init_node('line', anonymous=True)
vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)


if not cap.isOpened():
    rospy.logerr("打开摄像头失败")
rospy.loginfo("视觉巡线节点已启动!")

pid = PIDController(Kp=-0.3, Ki=0, Kd=-0.02)

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        rospy.logerr("获取图片失败")
        continue
    # cv2.imshow('o', frame)
    frame = cv2.flip(frame, 1)
    
    binary_image = fixed_threshold_binarization(frame)
    
    if detect_stopping_line(binary_image):
        rospy.loginfo("检测到停车线，停止小车!")
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        vel_publisher.publish(vel_msg)
        break  # 退出循环

    height, width = binary_image.shape
    # 检测中线并获取中心点位置
    center_line,visual_img = detect_center_line(binary_image)

    if center_line:  # 如果检测到中线
        center_line_x = center_line[-1][0]  # 取最后一个点的x坐标
        error = center_line_x - width // 2
    else:  # 无中线时默认居中
        error = 0

    cv2.imshow('Center Line Detection', visual_img)
    cv2.waitKey(1)
    
    height, width = frame.shape[:2]
    error = center_line_x - width // 2
    angular_z = pid.compute(error)
    angular_z = np.clip(angular_z, -0.27, 0.27)  # 限制角速度范围
    
    # 动态调整线速度
    max_speed = 0.2
    max_error = width // 2  # 图像中心到边缘的最大误差
    decay_factor = max(0.0, 1 - abs(error) / max_error)
    linear_x = max_speed * decay_factor

    vel_msg = Twist()
    vel_msg.linear.x = linear_x
    vel_msg.angular.z = angular_z  # 方向修正
    
    # vel_publisher.publish(vel_msg)
    rate.sleep()
