#!/usr/bin/env python3
#coding=utf-8

import rospy
from geometry_msgs.msg import Twist
import cv2


# 初始化节点，名称为 "line"
rospy.init_node('line', anonymous=True)
# 创建Publisher，话题为 'cmdvel'，消息类型为Twist速度消息
vel_publisher = rospy.Publisher('cmdvel', Twist, queue_size=10)
# 设置循环频率 (20Hz)
rate = rospy.Rate(10)
#打开摄像头
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
if not cap.isOpened():
    rospy.logerr("打开摄像头失败")
rospy.loginfo("视觉巡线节点已启动!")

while not rospy.is_shutdown():
    rec, frame = cap.read()
    if not rec:
        rospy.logerr("获取图片失败")
    frame = cv2.flip(frame, 1)
    cv2.imshow('Detection', frame)
    cv2.waitKey(1)
    
    # 创建Twist速度消息
    vel_msg = Twist()
    vel_msg.linear.x = 0.1  # 设置线速度 (可修改)
    vel_msg.angular.z = 0.0  # 设置角速度 (可修改)
    
    # 发布速度消息
    vel_publisher.publish(vel_msg)
    
    # 按照设定的频率循环
    rate.sleep()
