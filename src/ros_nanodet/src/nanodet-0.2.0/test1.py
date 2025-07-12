#!/usr/bin/env python3
#coding=utf-8

import rospy
import cv2
from std_msgs.msg import Header, Float64, Int8
from std_msgs.msg import String
from demo.demo import detect
from demo.demo import init
from calibrate_helper import Calibrator


# 初始化类别及对应序号, 序号与config保持一致
bulletproof_vest = 0
first_aid_kit = 1
spontoon = 2
teargas = 3
terrorist = 4  # 1个恐怖分子
terrorist2 = 5  # 2个恐怖分子
terrorist3 = 6  # 3个恐怖分子
goal = 3  #目标物
mid = 480 

# target_detected = None
# target_detected_fb = None
# target_detected_x = None

predictor = init()
def main():
    rospy.init_node("hdu_detect", anonymous=True)
    print('start subscribe')
    rospy.Subscriber("start_detect1", Int8, detect_callback1, queue_size=1, buff_size=52428800)
    rospy.loginfo('Subscriber success')

    frame = cv2.imread('picture0.jpg')
    res = detect(frame,predictor)# 识别
    print("The first one finished!")
    # res = detect(frame,predictor)
    # data,res = detect(cap,predictor)
    # predictor = init()
    frame = cv2.imread('picture1.jpg')
    res = detect(frame,predictor)# 识别2次
    print("The second one finished")
    rospy.spin()

def detect_callback1(message):
    target_detected  = rospy.Publisher("target_detected", Int8,queue_size=1)
    target_detected_fb  = rospy.Publisher("target_detected_fb", Int8,queue_size=1)
    target_detected_x  = rospy.Publisher("target_detected_x", Float64,queue_size=1)
    # 设置循环频率
    rate = rospy.Rate(2)
    global predictor
    # global target_detected
    # global target_detected_fb
    # global target_detected_x
    is_detected = 0
    if message.data:
        cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        rospy.loginfo('detect begin')
        # data = 1
        # target_detected_fb.publish(data)
        while (is_detected == 0):
            rospy.loginfo('detect is_detected')
            rec, frame = cap.read()
            if not rec:
                rospy.logerr("Failed to read frame from video capture device.")
                continue
            res = detect(frame, predictor)
            class_names = list(res.keys())
            values = list(res.values())  # 置信度列表
            i = 0
            # detect_flag=1
            for item in values:  # 对于每次识别结果
                if len(item) != 0:  # 当值不为0时
                    score_thres = round(item[0][4], 3)  # 取置信度
                    bboxes = item[0]
                    if score_thres > 0.630 and score_thres < 1.000:
                        print(score_thres)
                        # detect_flag = 0 # 若此列表中有对应的类别，则再次给detect_flag保持为0,检测到了
                        # 判断识别到的类别
                        if class_names[i] == goal:  # 如果识别到的类别索引 class_names[i] 等于 0，即识别到了 "core_veg" 类别，
                            target_detected.publish(1)  # 发布一个消息，消息的值为 1。                
                            object_center = (bboxes[0] + bboxes[3]) // 2 # 偏移找板，检测框x值计算
                            offset = object_center - mid    # 正值右偏，负值左偏
                            if offset <= 5 and offset >= -5:
                                error = 0
                            elif offset < -5:
                                error = -2
                            else:
                                error = 2
                            target_detected_x.publish(error)
                            rospy.loginfo('find')  # 记录日志信息
                            is_detected = 1
                            break              
                i=i+1
            if is_detected != 1:
                target_detected.publish(2)
                rospy.loginfo('not find')
            rate.sleep()
if __name__ == '__main__':
    main()

