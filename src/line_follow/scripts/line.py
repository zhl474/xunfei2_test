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

def detect_curvature(nonzero_x, nonzero_y):
    """计算曲率 [[9]]"""
    if len(nonzero_x) < 10:
        return float('inf')
    
    x = nonzero_x.astype(np.float32) / max(nonzero_x)
    y = nonzero_y.astype(np.float32) / max(nonzero_y)
    
    # 二次多项式拟合
    A = np.vstack([y, np.ones(len(y))]).T
    m, c = np.linalg.lstsq(A, x, rcond=None)[0]
    curvature = abs(m)  # 简化曲率计算
    return curvature
        
def detect_center_line(binary_image):
    height, width = binary_image.shape
    
    # 获取所有黑色像素位置
    nonzero_y, nonzero_x = np.nonzero(binary_image == 0) 
    
    # 分割左右半区
    mid_x = width // 2
    left_mask = (nonzero_x < mid_x)
    right_mask = (nonzero_x >= mid_x)
    
    # 分别提取左右侧点
    nonzero_y_left = nonzero_y[left_mask]
    nonzero_x_left = nonzero_x[left_mask]
    nonzero_y_right = nonzero_y[right_mask]
    nonzero_x_right = nonzero_x[right_mask]
    
    # 计算曲率 [[9]]
    left_curvature = detect_curvature(nonzero_x_left, nonzero_y_left) if len(nonzero_x_left) > 0 else float('inf')
    right_curvature = detect_curvature(nonzero_x_right, nonzero_y_right) if len(nonzero_x_right) > 0 else float('inf')
    print("left_curvature")
    print(left_curvature)
    print("\nright_curvature")
    print(right_curvature)
    # 判断使用哪侧作为基准线
    use_right = len(nonzero_y_right) >= len(nonzero_y_left)
    
    k1 = 1.0
    k2 =1.0
    # 初始化基准线数据
    if use_right and len(nonzero_y_right) > 0:
        # 右侧基准线处理
        max_x_right = np.zeros(height, dtype=np.int32)
        np.maximum.at(max_x_right, nonzero_y_right, nonzero_x_right)
        valid_rows = (max_x_right > mid_x) & (max_x_right < width)
        y_coords = np.where(valid_rows)[0]
        x_coords = max_x_right[valid_rows]
          
            
        # 计算偏移中线（右侧减偏移）
        centers = np.zeros(height, dtype=np.int32)
        centers[valid_rows] = max_x_right[valid_rows] - (420 - k1 * (210 - y_coords))
        
    elif len(nonzero_y_left) > 0:
        # 左侧基准线处理
        min_x_left = np.full(height, width, dtype=np.int32)
        np.minimum.at(min_x_left, nonzero_y_left, nonzero_x_left)
        valid_rows = (min_x_left > 0) & (min_x_left < mid_x)
        y_coords = np.where(valid_rows)[0]
        x_coords = min_x_left[valid_rows]
        
            
        # 计算偏移中线（左侧加偏移）
        centers = np.zeros(height, dtype=np.int32)
        centers[valid_rows] = min_x_left[valid_rows] + (420 - k2 * (210 - y_coords))
        
    else:
        # 默认中线
        center_line_x = mid_x
        visual_img = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
        cv2.line(visual_img, (center_line_x, 0), (center_line_x, height-1), (0, 255, 255), 2)
        return center_line_x, visual_img

    # 限制中线范围
    centers = np.clip(centers, 0, width-1)

    # 加权平均计算最终中线
    weights = np.arange(height, 0, -1) * 2
    # weights = np.arange(1, height + 1)  # 上方权重更高

    if np.any(valid_rows):
        weighted_sum = np.sum(centers[valid_rows] * weights[valid_rows])
        total_weight = np.sum(weights[valid_rows])

        if total_weight != 0:
            center_line_x = int(weighted_sum / total_weight)
        else:
            center_line_x = mid_x
    else:
        center_line_x = mid_x

    
    # 可视化
    visual_img = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
    cv2.line(visual_img, (center_line_x, 0), (center_line_x, height-1), (0, 255, 255), 2)
    
    for y in range(height):
        if valid_rows[y]:
            cv2.circle(visual_img, (centers[y], y), 2, (0, 0, 255), -1)

    in_curve = (right_curvature > 0.005 or left_curvature > 0.005) if (right_curvature != float('inf') and left_curvature != float('inf')) else False

    return center_line_x, visual_img,in_curve

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

in_corner = False
rotation_complete = False
corner_detection_active = False

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
    center_line_x,visual_img,, in_curve = detect_center_line(binary_image)


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
  