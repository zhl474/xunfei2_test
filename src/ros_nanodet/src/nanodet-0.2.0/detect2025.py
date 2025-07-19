#!/usr/bin/env python3
#coding=utf-8

import rospy
from ros_nanodet.srv import detect_result_srv, detect_result_srvRequest, detect_result_srvResponse
from demo.demo import detect
from demo.demo import init
import cv2
import numpy as np
import time

predictor = init()
rospy.init_node("nanodet_detect", anonymous=True)

frame = cv2.imread('/home/ucar/ucar_car/ypicture/picture_14.jpg')
res = detect(frame,predictor)# 识别
res = detect(frame,predictor)# 识别2次
print("warm up done")

# 全局变量管理摄像头状态
camera_active = False
cap = None

output_filename = "/home/ucar/ucar_car/src/ztestnav2025/nanodet_debug/nanodet.avi"#录制视频防止可视化卡顿
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # MP4格式，其他选项：'XVID'->AVI, 'MJPG'->MJPEG
fps = 5.0
frame_size = (640, 480)  # 必须和实际帧尺寸一致
out = cv2.VideoWriter(output_filename, fourcc, fps, frame_size)
# 检查是否成功创建
if not out.isOpened():
    print("无法创建视频文件！")

def shutdown_cap(response):
    global camera_active, cap
    if cap and cap.isOpened():
        cap.release()
        camera_active = False
        response.x0 = -1
        response.y0 = -1
        response.x1 = -1
        response.y1 = -1
        response.class_name = -1
        rospy.loginfo("关闭摄像头")
def open_cap():
    global camera_active, cap
    if camera_active:
        # rospy.logwarn("摄像头被重复打开")
        return
    else:
        cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        if not cap.isOpened():
            rospy.logerr("打开摄像头失败")
            return 0
        camera_active = True
        rospy.loginfo("摄像头成功打开")

def visualize(image, x0, y0, x1, y1, name, conf):
    global out
    color = (0, 255, 0)  # 绿色边框
    cv2.rectangle(image, (x0, y0), (x1, y1), color, 2)  # 2是边框粗细
    label = f"{name}: {conf:.2f}"
    (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
    cv2.rectangle(
        image, 
        (x0, y0 - text_height - 10),  # 左上角
        (x0 + text_width, y0),        # 右下角
        color,
        cv2.FILLED                     # 填充矩形
    )
    cv2.putText(
        image,
        label,
        (x0, y0 - 5),  # 文本位置
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,            # 字体大小
        (0, 0, 0),      # 黑色文本
        2               # 文本粗细
    )
    # cv2.imshow('Detection', image)
    # cv2.waitKey(1)
    # out.write(image)
    

#首次启动要发个-1启动摄像头，发送-2关闭摄像头防止冲突
def detect_start(req):
    start_time = time.time()
    global camera_active, cap, out
    response = detect_result_srvResponse()
    if req.detect_start==-1:
        open_cap()
    if req.detect_start==-2:
        shutdown_cap(response)
        return response
    rec, frame = cap.read()
    if not rec:
        rospy.logerr("获取图片失败")
    frame = cv2.flip(frame, 1)
    res = detect(frame, predictor)
    # print("目标检测耗时")
    # print(time.time()-start_time)
    max_score = -1.0
    best_bbox = [-1] * 5 
    target = -1
    for label in res:
        for bbox in res[label]:
            score = bbox[-1]
            if score > max_score and score > 0.6:
                if label >= (req.detect_start-1)*3 and label <req.detect_start*3:
                    max_score = score
                    best_bbox = bbox
                    target = label
                    # print("检测到目标")
                    # print(target)
    x0, y0, x1, y1, conf = [int(coord) for coord in best_bbox]
    response.x0 = x0
    response.y0 = y0
    response.x1 = x1
    response.y1 = y1
    response.class_name = target
    # print((x0+x1)/2)
    # print("标签筛选耗时")
    # print(time.time()-start_time)
    if best_bbox[0] != -1:
        visualize(frame,x0, y0, x1, y1,target,conf)
    # print("可视化耗时")
    # print(time.time()-start_time)
    out.write(frame)
    # print("完整服务耗时")
    # print(time.time()-start_time)
    return response

server = rospy.Service("nanodet_detect",detect_result_srv,detect_start)
print("目标检测就绪")
rospy.spin()
out.release()
print("结束")



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