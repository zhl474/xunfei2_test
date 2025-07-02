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

def fixed_threshold_binarization(image, threshold=200, max_value=255, threshold_type=cv2.THRESH_BINARY_INV):
    try:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        _, binary = cv2.threshold(blur, threshold, max_value, threshold_type)
        debug_img = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        return binary, debug_img
    except Exception as e:
        rospy.logwarn(f"二值化异常: {str(e)}")
        height, width = image.shape[:2]
        return np.zeros((height, width), dtype=np.uint8), np.zeros((height, width, 3), dtype=np.uint8)

def detect_center_line(binary_image):
    """
    基于逐行扫描的赛道中线检测算法
    """
    height, width = binary_image.shape
    # debug_img = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
    
    L_black = np.zeros(height, dtype=np.int32)
    R_black = np.zeros(height, dtype=np.int32)
    LCenter = np.zeros(height, dtype=np.int32)

    baseline = width // 2
    
    for y in range(height):
        # 向左扫描找左边界
        L_black[y] = 0
        for x in range(baseline, 0, -1):
            if x >= 1 and binary_image[y, x-1] == 255 and binary_image[y, x] == 0:
                L_black[y] = x
                break

        # 向右扫描找右边界
        R_black[y] = width
        for x in range(baseline, width-1):
            if x <= width-2 and binary_image[y, x+1] == 255 and binary_image[y, x] == 0:
                R_black[y] = x
                break

        # 计算中线
        LCenter[y] = (L_black[y] + R_black[y]) // 2

    # 绘制检测结果
    # for y in range(height):
    #     if L_black[y] > 0:
    #         cv2.circle(debug_img, (L_black[y], y), 1, (0, 0, 255), -1)
    #     if R_black[y] < width:
    #         cv2.circle(debug_img, (R_black[y], y), 1, (255, 0, 0), -1)
    #     if LCenter[y] > 0 and LCenter[y] < width:
    #         cv2.circle(debug_img, (LCenter[y], y), 1, (0, 255, 0), -1)
    
    # 计算中线
    valid_lines = []
    for y in range(height):
        if L_black[y] > 0 and R_black[y] < 255:
            valid_lines.append(LCenter[y])
    
    center_line_x = int(np.mean(valid_lines)) if valid_lines else width // 2
    # cv2.line(debug_img, (center_line_x, 0), (center_line_x, height), (255, 255, 0), 2)

    return center_line_x, debug_img

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
    cv2.imshow('frame',frame)

    frame = cv2.flip(frame, 1)
    
    binary_image, threshold_debug = fixed_threshold_binarization(frame)
    # center_line_x, line_debug = detect_center_line(binary_image)
    
    cv2.imshow('Binary', binary_image)
    # cv2.imshow('Debug', line_debug)
    cv2.waitKey(1)
    
    # height, width = frame.shape[:2]
    # error = center_line_x - width // 2
    #angular_z = pid.compute(error)
    
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0  # 方向修正
    
    vel_publisher.publish(vel_msg)
    rate.sleep()