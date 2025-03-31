#!/usr/bin/env python
#coding=utf-8

import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

Kp = 0.00175  # 0.003125 kp=1/240，其中240是最大误差像素，1是希望最大误差时角速度是 ±1.0 弧度/秒
Kd = 0.0006 
previous_error = 0

bridge = CvBridge()
twist = Twist()

# 中线计算
def mid(follow, mask):
    # 获取图像宽度的一半，作为初始搜索中线的左侧边界
    # print(follow.shape[0]) 图像高度
    # print(follow.shape[1]) 图像宽度
    halfWidth= follow.shape[1] // 2
    # 初始化为图像宽度的一半，后续可能会根据找到的中线进行调整
    half = halfWidth  
    # 从图像底部向上遍历每一行
    for y in range(follow.shape[0] - 1, -1, -1):
        # 检查左半部分是否全为0（即无道路标记） 
        if (mask[y][max(0,half-halfWidth):half] == np.zeros_like(mask[y][max(0,half-halfWidth):half])).all():
            left = max(0,half-halfWidth) # 如果是，则左侧边界为当前左边界    
        else:
            left = np.average(np.where(mask[y][0:half] == 255)) # 否则，计算左侧道路标记的平均位置
         # 检查右半部分是否全为0
        if (mask[y][half:min(follow.shape[1],half+halfWidth)] == np.zeros_like(mask[y][half:min(follow.shape[1],half+halfWidth)])).all():
            right = min(follow.shape[1],half+halfWidth) # 如果是，则右侧边界为当前右边界  
            
        else:
            right = np.average(np.where(mask[y][half:follow.shape[1]] == 255)) + half # 否则，计算右侧道路标记的平均位置，并加上中线位置（half）得到绝对位置  
        # 白线的x中线
        #if (left <=10 ):
        #    mid = right - 220
        #elif(right >=630):
        #    mid = left + 220
        #else:
        #    mid = (left + right) // 2
        mid = (left + right) // 2
        #if y == 160:
            # roadwidth = right + left
            # print("left=%d" % left)
            # print("right=%d" % right)
            # print("roadwidth=%d" % roadwidth)
        half = int(mid)
        # 绘制出中线
        follow[y, int(mid)] = 255

        if y == 170:
             mid_output = int(mid)
    cv.circle(follow, (mid_output, 170), 5, 255, -1)
    # 相机y坐标中点与实际线的中点差
    error = follow.shape[1] // 2 - mid_output
    # img_x=follow.shape[1] // 2
    # print(img_x)
    return follow,error
                                              
def image_callback(msg):
    global integral, previous_error
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    # img = cv.resize(img, (640, 480))
    # 裁剪像素y的大小，防止干扰
    y_start = (640 - 300) // 2
    y_end = y_start + 270 
    cropped_img = img[y_start:y_end,:]

    img_hsv = cv.cvtColor(cropped_img, cv.COLOR_BGR2HSV)
    # hsv阈值设置
    mask = cv.inRange(img_hsv, np.array([0, 0, 210]), np.array([179, 30, 255]))
 
    follow = mask.copy()
    follow,error= mid(follow, mask)
    
    derivative = error - previous_error
    control_signal = Kp * error + Kd * derivative  #比例微分控制
    if (-5<=error<=5):
        error = 0
    
    # control_signal = Kp * error     # 比例控制
    print("error=%d" % error)
    # print(control_signal)
    previous_error = error
    # 根据差值进行p调节或者pd调节
    twist.linear.x = 0.1
    twist.angular.z = -control_signal

    cmd_pub.publish(twist)
    # cv.imshow("img", img)
    # cv.imshow("mask", mask)
    # cv.imshow("follow", follow)
    cv.waitKey(1)
 
if __name__ == '__main__':
    rospy.init_node('lane_follower', anonymous=True)
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback,queue_size=1) 
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.spin()

