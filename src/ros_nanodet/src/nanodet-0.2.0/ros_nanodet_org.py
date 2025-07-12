#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header, Float64,Int8
from std_msgs.msg import String
from demo.demo import detect
from demo.demo import init
import cv2
repeat = [0]*7
diningroom = 0
bedroom = 1
livingroom = 2
undefined_room = 3
not_only = 0
last_class_names = 10
cap = 0
predictor = 0
m = 0
def main():
    global cap,predictor
    rospy.init_node('hdu_detect',anonymous=True)
    rospy.Subscriber('start_detect', Int8, detect_callback, queue_size=1, buff_size=52428800)
    cap,predictor = init()
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,800)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,600)
    data,res = detect(cap,predictor)
    rospy.spin()
def detect_callback(message): 
    global not_only,repeat,last_class_names,cap,predictor,m
    room_msg_pub = rospy.Publisher('room_msg',Float64,queue_size=20)
    if message.data:
        rospy.loginfo('detect begin')
        data,res = detect(cap,predictor)
#        cv2.imwrite('picture'+str(m)+'.jpg',frame)
#        print("success to save" + str(m) + ".jpg")
        m = m+1
        class_names = list(res.keys())
        i = 0
        for item in res.values():
            if len(item)!=0:
                score = round(item[0][4],2)
                if item[0][4]>0.75:#阈值设置
                    if class_names[i] != last_class_names:
                        print()
                        print(class_names[i])
                        print(score)
                        print(not_only)
                        if class_names[i] == 4:
                            repeat[i] = repeat[i] + 1
                            if repeat[i] == 1:
                                last_class_names = class_names[i]
                                not_only = not_only+1
                                repeat = [0]*7
                                print("person")
                                print(score)
                                print(repeat)
                                if not_only == 2:
                                    room_msg_pub.publish(bedroom)
                                    not_only = 0
                                    break
                                room_msg_pub.publish(undefined_room)
                            break
                        if class_names[i] == 1:
                            repeat[i] = repeat[i] + 1
                            if repeat[i] == 1:
                                last_class_names = class_names[i]
                                not_only = not_only+1
                                repeat = [0]*7
                                print("pet")
                                print(score)
                                print(repeat)
                                if not_only == 2:
                                    room_msg_pub.publish(bedroom)
                                    not_only = 0
                                    break
                                room_msg_pub.publish(undefined_room)
                            break
                        if class_names[i] == 0:
                            repeat[i] = repeat[i] + 1
                            if repeat[i] == 1:
                                last_class_names = class_names[i]
                                room_msg_pub.publish(diningroom)
                                repeat = [0]*7
                                print("tableware")
                                print(score)
                                print(repeat)
                            not_only = 0
                            break
                        if class_names[i] == 6:
                            repeat[i] = repeat[i] + 1
                            if repeat[i] == 1:
                                last_class_names = class_names[i]
                                room_msg_pub.publish(diningroom)
                                repeat = [0]*7
                                print("food")
                                print(score)
                                print(repeat)
                            not_only = 0
                            break
                        if class_names[i] == 2:
                            repeat[i] = repeat[i] + 1
                            if repeat[i] == 1:
                                last_class_names = class_names[i]
                                room_msg_pub.publish(bedroom)
                                repeat = [0]*7
                                print("bed")
                                print(score)
                                print(repeat)
                            not_only = 0
                            break
                        if class_names[i] == 3:
                            repeat[i] = repeat[i] + 1
                            if repeat[i] == 1:
                                last_class_names = class_names[i]
                                room_msg_pub.publish(livingroom)
                                repeat = [0]*7
                                print("television")
                                print(score)
                                print(repeat)
                            not_only = 0
                            break
                        if class_names[i] == 5:
                            repeat[i] = repeat[i] + 1
                            if repeat[i] == 1:
                                last_class_names = class_names[i]
                                room_msg_pub.publish(livingroom)
                                repeat = [0]*7
                                print("sofa")
                                print(score)
                                print(repeat)
                            break
                        if class_names[i] == 7:
                            repeat[i] = repeat[i] + 1
                            if repeat[i] == 1:
                                last_class_names = class_names[i]
                                room_msg_pub.publish(undefined_room)
                                repeat = [0]*7
                                print("desk")
                                print(score)
                                print(repeat)
                            break
                        print(not_only)
            i = i+1
if __name__ == '__main__':
    main()
