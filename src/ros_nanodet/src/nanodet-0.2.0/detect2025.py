#!/usr/bin/env python3
#coding=utf-8

import rospy
from ros_nanodet.srv import detect_result_srv, detect_result_srvRequest, detect_result_srvResponse
from demo.demo import detect
from demo.demo import init
import cv2

predictor = init()
rospy.init_node("hdu_detect", anonymous=True)

frame = cv2.imread('/home/ucar/ucar_car/src/ros_nanodet/src/nanodet-0.2.0/2/1.png')
res = detect(frame,predictor)# 识别
res = detect(frame,predictor)# 识别2次
print("warm up done")

# 全局变量管理摄像头状态
camera_active = False
cap = None

def shutdown_cap(response):
    global camera_active, cap
    if cap and cap.isOpened():
        cap.release()
        response.x0 = -1
        response.y0 = -1
        response.x1 = -1
        response.y1 = -1
        response.class_name = -1
        rospy.loginfo("关闭摄像头")
def open_cap():
    global camera_active, cap
    if camera_active:
        rospy.logwarn("摄像头被重复打开")
    else:
        cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        if not cap.isOpened():
            rospy.logerr("打开摄像头失败")
            return 0
        camera_active = True
        rospy.loginfo("摄像头成功打开")

#首次启动要发个2启动摄像头，发送0关闭摄像头防止冲突
def detect_start(req):
    global camera_active, cap
    response = detect_result_srvResponse()
    if req.detect_start==2:
        open_cap()
    if req.detect_start==0:
        shutdown_cap(response)
        return response
    rec, frame = cap.read()
    print(rec)
    if not rec:
        rospy.logerr("获取图片失败")
    res = detect(frame, predictor)
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

server = rospy.Service("detect_result",detect_result_srv,detect_start)
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