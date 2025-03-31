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
terrorist3 = 6  # 2个恐怖分子
goal = -1  #目标物

predictor = init()
def main():
    rospy.init_node("hdu_detect", anonymous=True)
    print('start subscribe')
    rospy.Subscriber("start_detect", Int8, detect_callback, queue_size=1, buff_size=52428800)
    rospy.loginfo('Subscriber success')

    frame = cv2.imread('picture0.jpg')
    res = detect(frame,predictor)# 识别
    print("The first one finished")
    # res = detect(frame,predictor)
    # data,res = detect(cap,predictor)
    # predictor = init()
    frame = cv2.imread('picture1.jpg')
    res = detect(frame,predictor)# 识别2次
    print("The second one finished")
    rospy.spin()

def detect_callback(message):
    terrorist_data  = rospy.Publisher("terrorist_data", Int8,queue_size=1)
    global predictor
    is_detected = 0
    if message.data:
        cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        rospy.loginfo('detect begin')
        while (is_detected == 0):
            rospy.loginfo('detect is_detected')
            rec, frame = cap.read()
            # if rec:
            #     print("get imgs")
            # else:
            #     print("no imgs")
            res = detect(frame, predictor)
            print(res)
            class_names = list(res.keys())
            values = list(res.values())  # 置信度列表
            i = 0
            # detect_flag=1
            for item in values:  # 对于每次识别结果
                if len(item) != 0:  # 当值不为0时
                    score_thres = round(item[0][4], 3)  # 取置信度
                    if score_thres > 0.630 and score_thres < 1.000:
                        print(score_thres)
                        # detect_flag = 0 # 若此列表中有对应的类别，则再次给detect_flag保持为0,检测到了
                        # 判断识别到的类别
                        if class_names[i] == 0:  # 如果识别到的类别索引 class_names[i] 等于 0，即识别到了 "core_veg" 类别，
                            # area_msg_pub.publish(bulletproof_vest)  # 发布一个消息，消息的值为 core_veg。
                            rospy.loginfo('0bulletproof_vest')  # 记录日志信息
                            print("bulletproof_vest")
                            is_detected = 1
                            # break
                        elif class_names[i] == 1:
                            # area_msg_pub.publish(first_aid_kit)
                            rospy.loginfo('1first_aid_kit')
                            print("first_aid_kit")
                            is_detected = 1
                            # break
                        elif class_names[i] == 2:
                            # area_msg_pub.publish(spontoon)
                            rospy.loginfo('2spontoon')
                            print("spontoon")
                            is_detected = 1
                            # break
                        elif class_names[i] == 3:
                            # area_msg_pub.publish(teargas)
                            print("teargas")
                            rospy.loginfo('3teargas')
                            is_detected = 1
                            # break
                        elif class_names[i] == 4:
                            # area_msg_pub.publish(terrorist)
                            print("terrorist")
                            rospy.loginfo('4terrorist')
                            terrorist_data.publish(1)
                            goal = spontoon
                            rospy.loginfo(goal)
                            is_detected = 1
                            # break
                        elif class_names[i] == 5:
                            # area_msg_pub.publish(terrorist2)
                            print("terrorist2")
                            rospy.loginfo('5terrorist2')
                            terrorist_data.publish(2)
                            goal = bulletproof_vest
                            rospy.loginfo(goal)
                            is_detected = 1
                            # break
                        elif class_names[i] == 6:
                            # area_msg_pub.publish(terrorist2)
                            print("terrorist3")
                            rospy.loginfo('6terrorist3')
                            terrorist_data.publish(3)
                            goal = teargas
                            rospy.loginfo(goal)
                            is_detected = 1
                            # break
                        else :
                            print("未识别到")
                            # break

                i=i+1
if __name__ == '__main__':
    main()

