#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header, Float64,Int8
from std_msgs.msg import String
from demo.demo import detect
from demo.demo import init

repeat = [0]*7
diningroom = 0
bedroom = 1
livingroom = 2
not_only = 0
last_class_names = 10
cap = 0
predictor = 0
def main():
    global cap,predictor
    rospy.init_node('hdu_detect',anonymous=True)
    rospy.Subscriber('start_dectect', Int8, detect_callback, queue_size=1, buff_size=52428800)
    room_msg_pub = rospy.Publisher('room_msg',Float64,queue_size=20)
    cap,predictor = init()
    rospy.spin()
def detect_callback(message): 
    global not_only,repeat,last_class_names,cap,predictor
    if message:
        rospy.loginfo('detect begin')
        data,res = detect(cap,predictor)
        class_names = list(res.keys())
        i = 0
        for item in res.values():
            if len(item)!=0:
                score = round(item[0][4],2)
                if item[0][4]>0.6:#阈值设置
                    print(class_names[i])
                    print(score)
                    print(repeat)
                    if class_names[i] != last_class_names:
                        print(not_only)
                        if class_names[i] == 0:
                            repeat[i] = repeat[i] + 1
                            if repeat[i] == 15:
                                last_class_names = class_names[i]
                                room_msg_pub.publish(diningroom)
                                repeat = [0]*7
                            not_only = 0
                            break
                        if class_names[i] == 6:
                            repeat[i] = repeat[i] + 1
                            if repeat[i] == 15:
                                last_class_names = class_names[i]
                                room_msg_pub.publish(diningroom)
                                repeat = [0]*7
                            not_only = 0
                            break
                        if class_names[i] == 2:
                            repeat[i] = repeat[i] + 1
                            if repeat[i] == 15:
                                last_class_names = class_names[i]
                                room_msg_pub.publish(bedroom)
                                repeat = [0]*7
                            not_only = 0
                            break
                        if class_names[i] == 3:
                            repeat[i] = repeat[i] + 1
                            if repeat[i] == 15:
                                last_class_names = class_names[i]
                                room_msg_pub.publish(livingroom)
                                repeat = [0]*7
                            not_only = 0
                            break
                        if class_names[i] == 5:
                            repeat[i] = repeat[i] + 1
                            if repeat[i] == 15:
                                last_class_names = class_names[i]
                                room_msg_pub.publish(livingroom)
                                repeat = [0]*7
                            break
                        last_class_names = class_names[i]
                        not_only = not_only+1
                        print(not_only)
                        if not_only == 2:
                            last_class_names = class_names[i]
                            room_msg_pub.publish(bedroom)
                            not_only = 0
                            break
            i = i+1

if __name__ == '__main__':
    main()
