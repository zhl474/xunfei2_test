#! /usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import detect
import rospy
import numpy as np
from sensor_msgs.msg import Image

import sys
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
import ctypes
libgcc_s = ctypes.CDLL('libgcc_s.so.1')

# from cv_bridge import CvBridge

class yolo_detect():
    def __init__(self):
        self.a = detect.detectapi(weights = '/home/ucar/ucar_car/src/my_loyo/scripts/runs/train/exp13/weights/best.pt')
        im_sub = rospy.Subscriber('/camera/image', Image, self.detectimg)
        self.img_pub = rospy.Publisher('/yolov5/detect' , Image, queue_size = 1)
        self.bridge = CvBridge() #OpenCV与ROS的消息转换类

    def detectimg(self, img):
        frame = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        result, names = self.a.detect([frame])
        image_detect = result[0][0]
        print(result[0][1])
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(image_detect, "bgr8"))
        

            

        
if __name__ == '__main__':
    rospy.init_node("yolo_detect")
    rospy.loginfo("yolo_detect node started")
    yolo_detect()
    rospy.spin()

# while True:
#     rec, img = cap.read()
#     result, names = a.detect([img])
#     img = result[0][0]
#     '''
#     for cls,(x1,y1,x2,y2),conf in result[0][1]: #第一张图片的处理结果标签。
#         print(cls,x1,y1,x2,y2,conf)
#         cv2.rectangle(img,(x1,y1),(x2,y2),(0,255,0))
#         cv2.putText(img,names[cls],(x1,y1-20),cv2.FONT_HERSHEY_DUPLEX,1.5,(255,0,0))
#     '''
#     cv2.imshow("vedio", img)
#     if cv2.waitKey(1)==ord('q'):
#         break

