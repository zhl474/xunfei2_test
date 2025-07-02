#!/usr/bin/env python3
# coding=utf-8

import rospy
from geometry_msgs.msg import Twist
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
    基于逐行扫描的赛道中线检测算法
    """
    height, width = binary_image.shape
    
    L_black = np.zeros(height, dtype=np.int32)
    R_black = np.zeros(height, dtype=np.int32)
    LCenter = np.zeros(height, dtype=np.int32)

    baseline = width // 2
    
    for y in range(height-1, -1, -1):
        # 向左扫描找左边界
        L_black[y] = 0
        left_found = False
        for x in range(baseline, 0, -1):
            if x >= 1 and binary_image[y, x-1] == 255 and binary_image[y, x] == 0:
                L_black[y] = x
                left_found = True
                break

        # 向右扫描找右边界
        R_black[y] = width
        right_found = False
        for x in range(baseline, width-1):
            if x <= width-2 and binary_image[y, x+1] == 255 and binary_image[y, x] == 0:
                R_black[y] = x
                right_found = True
                break

        # 计算中线
        if left_found and right_found and L_black[y] < R_black[y]:
            LCenter[y] = (L_black[y] + R_black[y]) // 2

    
   # 计算中线（使用加权平均，下方行权重更高）
    weighted_sum = 0
    total_weight = 0
    valid_count = 0
    
    for y in range(height):
        if LCenter[y] >= 0:  # 只处理有效行
            # 行位置作为权重（y越大权重越高）
            weight = (y + 1) * 2  # 增加权重差异
            weighted_sum += LCenter[y] * weight
            total_weight += weight
            valid_count += 1
    
    if valid_count > 0:
        center_line_x = int(weighted_sum / total_weight)
    else:
        center_line_x = width // 2  # 默认中线

    
    # 创建可视化图像
    visual_img = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
    
    # 在可视化图像上绘制中线
    cv2.line(visual_img, (center_line_x, 0), (center_line_x, height-1), (0, 255, 255), 2)
    
    # 标记所有有效行的中线点
    for y in range(height):
        if LCenter[y] >= 0:
            cv2.circle(visual_img, (LCenter[y], y), 1, (0, 0, 255), -1)
    
    
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
    
    # 检测中线并获取中心点位置
    center_line_x,visual_img = detect_center_line(binary_image)
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