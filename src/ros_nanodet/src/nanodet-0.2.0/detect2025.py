#!/usr/bin/env python3
#coding=utf-8

import rospy
from ros_nanodet.srv import detect_result_srv, detect_result_srvRequest, detect_result_srvResponse
from demo.demo import detect
from demo.demo import init
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


predictor = init()
rospy.init_node("nanodet_detect")

frame = cv2.imread('/home/ucar/ucar_car/src/ros_nanodet/src/nanodet-0.2.0/2/1.png')
res = detect(frame,predictor)# 识别
res = detect(frame,predictor)# 识别2次
print("warm up done")


def detect_start(req):
    response = detect_result_srvResponse()
    if not predictor.latest_img:
        rospy.logwarn("nanodet没有可用图像")
        return None
    frame = predictor.cv_bridge.imgmsg_to_cv2(predictor.latest_img, "bgr8")
    frame = cv_bridge.imgmsg_to_cv2(img_ptr, "bgr8")
    
    res = self.detect(frame, predictor)
    if not res:
        rospy.loginfo("未检测到目标")
        return response
    max_score = -1.0
    best_bbox = [-1] * 5 
    for label in res:
        for bbox in res[label]:
            score = bbox[-1]
            if score > max_score and score > 0.6:
                max_score = score
                print("find object")
                best_bbox = bbox[:5]
    x0, y0, x1, y1, name= [int(coord) for coord in best_bbox]
    response.x0 = x0
    response.y0 = y0
    response.x1 = x1
    response.y1 = y1
    response.class_name = name
    return response

server = rospy.Service("nanodet_detect",detect_result_srv,detect_start)
print("目标检测就绪")
rospy.spin()



# cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
# while not rospy.is_shutdown():
#     rec, frame = cap.read()
#     print(rec)
#     if not rec:
#         rospy.logerr("Failed to read frame from video capture device.")
#         continue
#     res = detect(frame, predictor)
#     p = detect_result()
#     for label in res:
#         for bbox in res[label]:
#             score = bbox[-1]
#             if score>0.7:
#                 print("find object")
#                 x0, y0, x1, y1 = [int(i) for i in bbox[:4]]
#                 p.class_name = label
#                 p.x0 = x0
#                 p.y0 = y0
#                 p.x1 = x1
#                 p.y1 = y1
#                 pub.publish(p)