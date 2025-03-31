#! /usr/bin/env python
# coding=utf-8

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2



global i,keyword
i=0
keyword=-1



def callback(msg):
    global keyword,i
    keyword=input(":")
    bridge = CvBridge()
    cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    resize_img = cv2.resize(cv_img, dsize=(960, 540))
    if keyword == 's':      # 按s保存当前图片
        i += 1
        imgname = "/home/ucar/ucar_car/src/usb_cam/pic/"+str(i)+".jpg"
        cv2.imwrite(imgname, resize_img)
    elif keyword == 'q':     # 按q退出
        cv2.destroyAllWindows()

    
 
rospy.init_node('img_sub_node')
rospy.Subscriber('/usb_cam/image_raw', Image, callback)
rospy.spin()