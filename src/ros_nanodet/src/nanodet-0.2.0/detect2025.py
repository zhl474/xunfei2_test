#!/usr/bin/env python3
#coding=utf-8

import rospy
from ros_nanodet.msg import detect_result
from demo.demo import detect
from demo.demo import init
import cv2

predictor = init()
rospy.init_node("hdu_detect", anonymous=True)
pub = rospy.Publisher('detect_result', detect_result, queue_size=3)
print('start detect')
frame = cv2.imread('/home/ucar/ucar_car/src/ros_nanodet/src/nanodet-0.2.0/2/1.png')
res = detect(frame,predictor)# 识别
frame = cv2.imread('/home/ucar/ucar_car/src/ros_nanodet/src/nanodet-0.2.0/2/1.png')
res = detect(frame,predictor)# 识别2次
print("warm up done")

rate = rospy.Rate(1)
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
while not rospy.is_shutdown():
    rec, frame = cap.read()
    print(rec)
    if not rec:
        rospy.logerr("Failed to read frame from video capture device.")
        continue
    res = detect(frame, predictor)
    p = detect_result()
    for label in res:
        for bbox in res[label]:
            score = bbox[-1]
            if score>0.7:
                print("find object")
                x0, y0, x1, y1 = [int(i) for i in bbox[:4]]
                p.class_name = label
                p.x0 = x0
                p.y0 = y0
                p.x1 = x1
                p.y1 = y1
                pub.publish(p)
cap.release()
