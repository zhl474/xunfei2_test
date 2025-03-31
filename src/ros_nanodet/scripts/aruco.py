#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import time
import cv2
import cv2.aruco as aruco
import rospy
from sensor_msgs.msg import Image
import rospy
from geometry_msgs.msg import TransformStamped, Twist
import math
from ros_nanodet.srv import aruco_cmd,aruco_cmdRequest,aruco_cmdResponse

kp=0.00275


def Stopcar(req):
    pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1000)
    dis=0
    yaw=0
    cap = cv2.VideoCapture('/dev/video0')
    ret_val, frame = cap.read()
    twist = Twist()
    while abs(253-dis) > 3 and abs(412-yaw) > 3:
        ret_val, frame = cap.read()

        frame = cv2.flip(frame,1)

    #将图像转换为灰度图
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	
    #加载aruco字典，本次比赛使用的是4x4的aruco码
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
	
    #建立aruco检测参数，默认即可
        parameters =  aruco.DetectorParameters_create()
    
    #检测aruco码的角点信息
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
	
    #如果检测到aruco码，输出其编号
        if ids is not None:
            dis = corners[0][0][1][0]-corners[0][0][0][0]
            y = corners[0][0][1][0]
            print("distance is",dis)
            print("yaw is", y)

            twist.linear.x =  kp*(253-dis)
            twist.linear.y =  kp*(412-y)
            pub.publish(twist)
            
            #绘制出aruco码的外框    
            #aruco.drawDetectedMarkers(frame, corners, ids)
            #cv2.imshow("frame",frame)
    
            #按键1关闭程序
            #key = cv2.waitKey(1)
    twist.linear.x =  0
    twist.linear.y =  0
    pub.publish(twist)
    aruco_cmdResponse(1)
    return 1


if __name__ == '__main__':
    rospy.init_node("aruco_cmd1")

    server = rospy.Service("aruco_cmd",aruco_cmd,Stopcar)

    rospy.spin()
    


