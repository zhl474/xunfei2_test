import rospy
from std_msgs.msg import Header, Float64, Int8
from std_msgs.msg import String
from demo.demo import detect
from demo.demo import init
from calibrate_helper import Calibrator

import cv2

# 初始化类别及对应序号, 序号与config保持一致
bulletproof_vest = 0
first_aid_kit = 1
spontoon = 2
teargas = 3
terrorist = 4  # 1个恐怖分子
terrorist2 = 5  # 2个恐怖分子

# global m,cap,predictor,frame
# m=0

predictor = init()


def main():
    rospy.init_node('hdu_detect', anonymous=True)
    # '''这行代码的目的是创建一个ROS节点,并将其命名为'hdu_detect'，同时确保节点名称的唯一性。'''
    print('start subscribe')
    detect_callback(1)
    # rospy.Subscriber("start_detect", Int8, detect_callback, queue_size=1, buff_size=52428800)
    # '''创建一个订阅者对象，订阅名为"start_detect"的主题,接收Int8类型的消息,并在收到新消息时调用名为detect_callback的回调函数进行处理。'''
    # cap = cv2.VideoCapture('/dev/video0')

    # frame = cv2.imread('picture0.jpg')
    # res = detect(frame,predictor)# 识别
    # print("The first one finished")
    # # res = detect(frame,predictor)
    # # data,res = detect(cap,predictor)
    # # predictor = init()
    # frame = cv2.imread('picture1.jpg')
    # res = detect(frame,predictor)# 识别2次
    # print("The second one finished")

    # area_msg_pub.publish(11)

    # rospy.spin()  # 节点将进入一个无限循环，等待接收消息或服务请求。当有消息到达时，ROS将调用相应的回调函数来处理消息，然后继续等待下一个消息。


def detect_callback(message):  # 回调函数
    # global m,cap,predictor
    global predictor

    area_msg_pub = rospy.Publisher("area_msg", Int8, queue_size=1)
    area_msg_pubf = rospy.Publisher("area_msgf", String, queue_size=1)
    # rospy.Subscriber("video", Int8, video_callback, queue_size=1, buff_size=52428800)
    # fruit_amount_pub = rospy.Publisher("fruit_msg",  Int8, queue_size=10)
    # area_msg_pub.publish(10)  # 先发布一次

    # cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
    cap = cv2.VideoCapture(0)

    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
    judge = cap.isOpened()
    if judge:
        print("camera is open")
    else:
        print("camera not open")
    # print(cap.isOpened())
    end_count = 0
    is_detected = 0
    rospy.loginfo('detect begin')
    while (is_detected == 0):
        rospy.loginfo('detect is_detected')
        rec, frame = cap.read()
        if rec:
            print("get imgs")
        else:
            print("no imgs")
        cv2.imshow("frame", frame)
        cv2.waitKey(1)
        res = detect(frame, predictor)
        print(res)
        class_names = list(res.keys())
        # print(class_names)
        values = list(res.values())  # 置信度列表
        # print(values)
        i = 0
        # detect_flag=1
        for item in values:  # 对于每次识别结果
            if len(item) != 0:  # 当值不为0时
                score_thres = round(item[0][4], 3)  # 取置信度
                if score_thres > 0.130 and score_thres < 1.000:
                    print(score_thres)
                    # detect_flag = 0 # 若此列表中有对应的类别，则再次给detect_flag保持为0,检测到了
                    # 判断识别到的类别
                    if class_names[i] == 0:  # 如果识别到的类别索引 class_names[i] 等于 0，即识别到了 "core_veg" 类别，
                        area_msg_pub.publish(bulletproof_vest)  # 发布一个消息，消息的值为 core_veg。
                        rospy.loginfo('0bulletproof_vest')  # 记录日志信息
                        print("bulletproof_vest")
                        is_detected = 1
                        break
                    elif class_names[i] == 1:
                        area_msg_pub.publish(first_aid_kit)
                        rospy.loginfo('1first_aid_kit')
                        print("first_aid_kit")
                        is_detected = 1
                        break
                    elif class_names[i] == 2:
                        area_msg_pub.publish(spontoon)
                        rospy.loginfo('2spontoon')
                        print("spontoon")
                        is_detected = 1
                        break
                    elif class_names[i] == 3:
                        area_msg_pub.publish(teargas)
                        print("teargas")
                        rospy.loginfo('3teargas')
                        is_detected = 1
                        break
                    elif class_names[i] == 4:
                        area_msg_pub.publish(terrorist)
                        print("terrorist")
                        rospy.loginfo('4terrorist')
                        is_detected = 1
                        break
                    elif class_names[i] == 5:
                        area_msg_pub.publish(terrorist2)
                        print("terrorist2")
                        rospy.loginfo('5terrorist2')
                        is_detected = 1
                        break


if __name__ == '__main__':
    main()
