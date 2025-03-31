#!/usr/bin/env python3
#coding=utf-8

import rospy
import cv2
from std_msgs.msg import Header, Float64, Int8
from std_msgs.msg import String
from demo.demo import detect
from demo.demo import init
from calibrate_helper import Calibrator
from geometry_msgs.msg import Twist

# 初始化类别及对应序号, 序号与config保持一致
bulletproof_vest = 0
first_aid_kit = 1
spontoon = 2
teargas = 3
terrorist = 4  # 1个恐怖分子
terrorist2 = 5  # 2个恐怖分子
terrorist3 = 6  # 3个恐怖分子
goal = 3  #目标物

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
    pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1000)
    # 设置循环频率
    rate = rospy.Rate(2)
    msg = Twist()
    global predictor
    is_detected = 0
    if message.data:
        cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        rospy.loginfo('detect begin')
        while (is_detected == 0):
            rospy.loginfo('detect is_detected')
            rec, frame = cap.read()
            res = detect(frame, predictor)
            class_names = list(res.keys())
            values = list(res.values())  # 置信度列表
            i = 0
            for item in values:  # 对于每次识别结果
                if len(item) != 0:  # 当值不为0时
                    score_thres = round(item[0][4], 3)  # 取置信度
                    if score_thres > 0.500 and score_thres < 1.000:
                        print(score_thres)
                        # detect_flag = 0 # 若此列表中有对应的类别，则再次给detect_flag保持为0,检测到了
                        # 判断识别到的类别
                        if class_names[i] == goal:  # 如果识别到的类别索引 class_names[i] 等于 0，即识别到了 "core_veg" 类别，
                            msg.angular.z = 0
                            target_detected.publish(1)  # 发布一个消息，消息的值为 1。
                            # pub.publish(msg)
                            is_detected = 1
                            break
                            # rate.sleep()
                        else:
                            msg.angular.z = 0.1
                            # pub.publish(msg)
                            rate.sleep()
                    else:
                        msg.angular.z = 0.1
                        # pub.publish(msg)
                        rate.sleep()
                i=i+1
            rate.sleep()

if __name__ == '__main__':
    main()


