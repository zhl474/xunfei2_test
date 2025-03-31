#!/usr/bin/env python
#coding=utf-8

import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

Kp = 0.00695  # 0.003125 kp=1/240，其中240是最大误差像素，1是希望最大误差时角速度是 ±1.0 弧度/秒
Kd = 0.00035
previous_error = 0
stop_flag = 0

bridge = CvBridge()
twist = Twist()

# 中线计算
def mid(follow, mask):
    # print(follow.shape[0]) 图像高度
    # print(follow.shape[1]) 图像宽度
    
    # 获取图像宽度的一半，作为初始搜索中线的左侧边界
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
        if y ==245:
            WhiteCount = np.sum(mask[y, :] == 255)  # 计算白色像素点数量
            if WhiteCount >= 180:
                stop_flag = 1
                print(f"stop_flag = {stop_flag}")

        mid = (left + right) // 2
        half = int(mid)

        # 绘制出中线
        follow[y, int(mid)] = 255

        if y == 235:
             mid_output = int(mid)
    cv.circle(follow, (mid_output, 235), 5, 255, -1)
    # 相机y坐标中点与实际线的中点差
    error = follow.shape[1] // 2 - mid_output
    return follow,error
                                              
def image_callback(message):
    global stop_flag
    global previous_error
    rate = rospy.Rate(30)
    while message == 1:
        rate.sleep()
        cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        global integral, previous_error
        cap = cv.VideoCapture('/dev/video0', cv.CAP_V4L2)
        rec, frame = cap.read()
        img = frame
        # img = cv.resize(img, (640, 480))
        # 裁剪像素y的大小，防止干扰
        y_start = 170
        y_end = y_start + 310
        cropped_img = img[y_start:y_end,:]

        img_hsv = cv.cvtColor(cropped_img, cv.COLOR_BGR2HSV)
        # hsv阈值设置
        mask = cv.inRange(img_hsv, np.array([0, 0, 210]), np.array([179, 30, 255]))

        follow = mask.copy()
        follow,error= mid(follow, mask)

        derivative = error - previous_error
        control_signal = Kp * error + Kd * derivative  #比例微分控制
        if (-10 <= error <= 10):
            error = 0

        # control_signal = Kp * error     # 比例控制
        # print("error=%d" % error)
        # print(control_signal)
        previous_error = error
        # 根据差值进行p调节或者pd调节
        if stop_flag:
            stop_flag = stop_flag+1
            if stop_flag >= 10:
                twist.linear.x = 0
                twist.angular.z = 0
            else:
                twist.linear.x = 0.4
                twist.angular.z = -control_signal
            print(stop_flag)
        # rospy.info("Publishing Twist: linear.x=%f, angular.z=%f", twist.linear.x, twist.angular.z);
        cmd_pub.publish(twist)
        # cv.imshow("img", img)
        # cv.imshow("mask", mask)
        # cv.imshow("follow", follow)
        # cv.waitKey(1)
 
if __name__ == '__main__':

    rospy.init_node('lane_follower', anonymous=True)
    # rospy.Subscriber('/usb_cam/image_raw', Image, image_callback,queue_size=1)
    image_callback(1)
    # cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    #rate = rospy.Rate(30)

    #while not rospy.is_shutdown():
    #    rate.sleep()
    #rospy.spin()

