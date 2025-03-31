#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header, Float64,Int8
from std_msgs.msg import String
from demo.demo import detect
from demo.demo import init
import cv2.aruco as aruco
import cv2
from geometry_msgs.msg import TransformStamped, Twist
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
kp=0.0015
h_y=420
h_a=540
def main():
    global cap,predictor
#    rospy.init_node('hdu_detect',anonymous=True)
#    rospy.Subscriber('start_detect', Int8, detect_callback, queue_size=1, buff_size=52428800)
#    rospy.Subscriber("aruco_cmd",Int8,Stopcar_callback,queue_size=10)
#    rospy.Subscriber('room_index',Int8,leave_room_cb,queue_size=1)
    cap,predictor = init()
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,800)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,600)
    data,res = detect(cap,predictor)
    data,res = detect(cap,predictor)
    print(1)
    rospy.spin()
def leave_room_cb(msg):
    global not_only,last_class_names
    last_class_names = 10
    not_only = 0
    
def detect_callback(message): 
    global not_only,repeat,last_class_names,cap,predictor,m
    room_msg_pub = rospy.Publisher('room_msg',Float64,queue_size=20)
    if message.data:
        rospy.loginfo('capture begin')
        for m in range(8):#shezhi pai zhao shu liang
            ret_val,frame = cap.read()
            if ret_val:
                cv2.imwrite('picture'+str(m)+'.jpg',frame)
                print("success to save" + str(m) + ".jpg")
                m = m+1
#        room_msg_pub.publish(undefined_room)
        for x in range(2):#she zhi shi bie tu pian shu liang
            x = x + 4
            frame = cv2.imread('picture'+str(x)+'.jpg')
            rospy.loginfo('detect begin')
            meta, res = predictor.inference(frame)
    #       data,res = detect(cap,predictor)
            class_names = list(res.keys())
            i = 0
            for item in res.values():
                if len(item)!=0:
                    score = round(item[0][4],2)
                    if item[0][4]>0.40:#阈值设置
                        if class_names[i] == last_class_names:
                            i = i+1
                            continue
                        else:
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


def Stopcar_callback(data):
    pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1000)
    dis=0
    y=0
    ret_val, frame = cap.read()
    twist = Twist()
    while abs(h_a-dis) > 50 or abs(h_y-y) > 20:
        ret_val, frame = cap.read()

        frame = cv2.flip(frame,1)

    #将图像转换为灰度图
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	
    #加载aruco字典，本次比赛使用的是4x4的aruco码
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
	
    #建立aruco检测参数，默认即可
        parameters =  aruco.DetectorParameters_create()
    
    #检测aruco码的角点信息
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
	
    #如果检测到aruco码，输出其编号
        if ids is not None:
            dis = (corners[0][0][1][0]-corners[0][0][0][0])*(corners[0][0][2][1]-corners[0][0][0][1])*0.01   #面积的百分之一
            y = (corners[0][0][2][0]+corners[0][0][3][0])/2     #平行y轴的一条中线
            print("area is",dis)
            print("center is", y)
            #print()
            twist.linear.x =  0
            twist.linear.y =  0
            if kp*(h_a-dis)>0:
                twist.linear.x =  kp*(h_a-dis)
            if kp*(h_y-y)<0:
                twist.linear.y =  kp*(h_y-y)
            pub.publish(twist)
            
            #绘制出aruco码的外框    
            #aruco.drawDetectedMarkers(frame, corners, ids)
            #cv2.imshow("frame",frame)
    
            #按键1关闭程序
            #key = cv2.waitKey(1)
    twist.linear.x =  0
    twist.linear.y =  0
    pub.publish(twist)
    rospy.loginfo('reach final')
if __name__ == '__main__':
    main()
