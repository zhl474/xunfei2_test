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
        cropped_gray = gray[height//2:, :]  # 裁剪下半部分
        
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
    
    #  使用投影法快速找到左右边界
    # 计算每行的最小和最大非零列索引
    nonzero_y, nonzero_x = np.nonzero(binary_image == 0)  # 找到所有黑色像素的位置
    
    # 如果没有找到任何黑色像素，使用默认中线
    if len(nonzero_y) == 0:
        center_line_x = width // 2
        visual_img = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
        cv2.line(visual_img, (center_line_x, 0), (center_line_x, height-1), (0, 255, 255), 2)
        return center_line_x, visual_img
    
    # 按行分组，找到每行的左右边界
    min_x = np.zeros(height, dtype=np.int32)  # 初始化为0
    max_x = np.zeros(height, dtype=np.int32) + width  # 初始化为最大宽度
    
    # 使用向量化操作更新每行的最小和最大x值
    np.minimum.at(min_x, nonzero_y, nonzero_x)
    np.maximum.at(max_x, nonzero_y, nonzero_x)
    
    # 计算每行的中线
    centers = (min_x + max_x) // 2
    valid_mask = (min_x < max_x) & (max_x < width) & (min_x > 0)
    

    # 计算加权中线（下方行权重更高）
    weights = np.arange(height, 0, -1) * 2  # 行号越大（图像下方）权重越高
    weighted_sum = np.sum(centers[valid_mask] * weights[valid_mask])
    total_weight = np.sum(weights[valid_mask])
    center_line_x = int(weighted_sum / total_weight)
    
    # 创建可视化图像
    visual_img = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
    
    # 在可视化图像上绘制中线
    cv2.line(visual_img, (center_line_x, 0), (center_line_x, height-1), (0, 255, 255), 2)
    
    # 标记所有有效行的中线点
    for y in range(height):
        if valid_mask[y]:
            cv2.circle(visual_img, (centers[y], y), 1, (0, 0, 255), -1)
    
    # 添加调试信息
    cv2.putText(visual_img, f"Center: {center_line_x}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    print(f"Min_x: {min_x[:5]}...")  # 查看前几行左边界
    print(f"Max_x: {max_x[:5]}...")  # 查看前几行右边界
    print(f"Weights: {weights[:5]}...")  # 查看前几行权重

    return center_line_x, visual_img


# 初始化节点
rospy.init_node('line', anonymous=True)
vel_publisher = rospy.Publisher('cmdvel', Twist, queue_size=10)
rate = rospy.Rate(10)
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)

if not cap.isOpened():
    rospy.logerr("打开摄像头失败")
rospy.loginfo("视觉巡线节点已启动!")

# pid = PIDController(Kp=0.03, Ki=0.001, Kd=0.015)

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        rospy.logerr("获取图片失败")
        continue

    frame = cv2.flip(frame, 1)
    
    binary_image = fixed_threshold_binarization(frame)
    
    height, width = binary_image.shape
    cv2.imshow('frame',frame)
    cv2.imshow('Binary', binary_image)
    cv2.waitKey(1)
    
    start=time.time()
    # 检测中线并获取中心点位置
    center_line_x,visual_img = detect_center_line(binary_image)
    end=time.time()
    print(end-start)

    cv2.imshow('Center Line Detection', visual_img)
    cv2.waitKey(1)
    
    # height, width = frame.shape[:2]
    # error = center_line_x - width // 2
    #angular_z = pid.compute(error)
    
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0  # 方向修正
    
    vel_publisher.publish(vel_msg)
    rate.sleep()