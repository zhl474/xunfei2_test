#!/usr/bin/env python3
#coding=utf-8

import rospy
import cv2
import numpy 
from std_msgs.msg import Header, Float64, Int8
from std_msgs.msg import String
from demo.demo import detect
from demo.demo import init
from calibrate_helper import Calibrator
from geometry_msgs.msg import Twist
import time


# 初始化类别及对应序号, 序号与config保持一致
bulletproof_vest = 0
first_aid_kit = 1
spontoon = 2
teargas = 3
terrorist = 4  # 1个恐怖分子
terrorist2 = 5  # 2个恐怖分子
terrorist3 = 6  # 3个恐怖分子
goal = -1 #目标物

Kp = 0.00295  # 0.003125 kp=1/240，其中240是最大误差像素，1是希望最大误差时角速度是 ±1.0 弧度/秒
Kd = 0.00035
previous_error = 0
stop_flag = 0

# bridge = CvBridge()

predictor = init()
def main():
    rospy.init_node("hdu_detect", anonymous=True)
    print('start subscribe')
    message = Int8()
    message.data = 1
    detect_callback(message)
    # image_callback(1)

def detect_callback(message):
    pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1000)
    # 设置循环频率
    rate = rospy.Rate(2)
    global predictor
    msg = Twist()
    is_detected = 0
    cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
    rospy.loginfo('detect begin')
    while (is_detected == 0):
        rospy.loginfo('detect is_detected')
        rec, frame = cap.read()
        if rec:
            print("get imgs")
        else:
            print("no imgs")
        
        # frame = turn_to_yellow(frame)
        # cv2.imshow("frame", frame)
        # cv2.waitKey(2)
        res = detect(frame, predictor)
        # print(res)
        class_names = list(res.keys())
        # print(class_names)
        values = list(res.values())  # 置信度列表
        # print(values)
        i = 0
        # detect_flag=1
        for item in values:  # 对于每次识别结果
            if len(item) != 0:  # 当值不为0时
                score_thres = round(item[0][4], 3)  # 取置信度
                if score_thres > 0.8 and score_thres < 1.000:
                        rospy.loginfo(score_thres)
                        rospy.loginfo(class_names[i])
                        # detect_flag = 0 # 若此列表中有对应的类别，则再次给detect_flag保持为0,检测到了
                        # 判断识别到的类别
                        if class_names[i] == goal:  # 如果识别到的类别索引 class_names[i] 等于 0，即识别到了 "core_veg" 类别， 
                            # msg.angular.z = 0
                            # pub.publish(msg)
                            rospy.loginfo('find')  # 记录日志信息   
                            break
                        # else:
                        #     msg.angular.z = 0.5
                        #     pub.publish(msg)
                # else:
                #     msg.angular.z = 0.5
                #     pub.publish(msg)
            # else:
            #     msg.angular.z = 0.5
            #     pub.publish(msg)      
            i=i+1
        # rate.sleep()
            # if is_detected != 1:
            #     print('not find')

def turn_to_yellow(frame):
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # 转换BGR到RGB
    tint_color = (227, 207, 0)
    
    # 遍历图像的每个像素
    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            r, g, b = img[i, j]
            # 调整红色和绿色通道的值，增加黄色的成分
            r = numpy.clip(r + (tint_color[0] - r) * 0.5, 0, 255)
            g = numpy.clip(g + (tint_color[1] - g) * 0.5, 0, 255)
            # 保持蓝色通道不变
            img[i, j] = (r, g, b)
    yellow_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return yellow_img

# 中线计算
def mid(follow, mask):
    global stop_flag
    # print(follow.shape[0]) 图像高度
    # print(follow.shape[1]) 图像宽度
    
    # 获取图像宽度的一半，作为初始搜索中线的左侧边界
    halfWidth= follow.shape[1] // 2

    # 初始化为图像宽度的一半，后续可能会根据找到的中线进行调整
    half = halfWidth  
    # 从图像底部向上遍历每一行
    for y in range(follow.shape[0] - 1, -1, -1):
        # 检查左半部分是否全为0（即无道路标记） 
        if (mask[y][max(0,half-halfWidth):half] == numpy.zeros_like(mask[y][max(0,half-halfWidth):half])).all():
            left = max(0,half-halfWidth) # 如果是，则左侧边界为当前左边界    
        else:
            left = numpy.average(numpy.where(mask[y][0:half] == 255)) # 否则，计算左侧道路标记的平均位置
         # 检查右半部分是否全为0
        if (mask[y][half:min(follow.shape[1],half+halfWidth)] == numpy.zeros_like(mask[y][half:min(follow.shape[1],half+halfWidth)])).all():
            right = min(follow.shape[1],half+halfWidth) # 如果是，则右侧边界为当前右边界  
            
        else:
            right = numpy.average(numpy.where(mask[y][half:follow.shape[1]] == 255)) + half # 否则，计算右侧道路标记的平均位置，并加上中线位置（half）得到绝对位置  
        if y ==245:
            WhiteCount = numpy.sum(mask[y, :] == 255)  # 计算白色像素点数量
            if WhiteCount >= 180:
                stop_flag = 1
                print(f"stop_flag = {stop_flag}")

        mid = (left + right) // 2
        half = int(mid)

        # 绘制出中线
        follow[y, int(mid)] = 255

        if y == 235:
             mid_output = int(mid)
    cv2.circle(follow, (mid_output, 235), 5, 255, -1)
    # 相机y坐标中点与实际线的中点差
    error = follow.shape[1] // 2 - mid_output
    return follow,error
                                              
def image_callback(message):
    global stop_flag
    global previous_error
    # global integral
    # rate = rospy.Rate(80)
    twist = Twist()
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Frames per second using CAP_PROP_FPS : {fps}")
    # cap = cv2.VideoCapture('/dev/video0',cv2.CAP_DSHOW)
    # if cap.set(cv2.CAP_PROP_FPS, 30):
    #     rospy.logerr("set true")
    if not cap.isOpened():
        rospy.logerr("Cannot open camera")
        return
    # rospy.logerr("11")
    frame_count = 0
    start_time = time.time()
    while message == 1:
        # rospy.logerr("22")

        rec, frame = cap.read()
        frame_count += 1
        elapsed_time = time.time() - start_time
        fps = frame_count / elapsed_time
        print(f"FPS: {fps:.2f}")
        # if not rec:
        #     rospy.logerr("Failed to capture image")
        #     continue
        # img = frame
        # rospy.logerr("33")
        # img = cv.resize(img, (640, 480))
        # 裁剪像素y的大小，防止干扰
        y_start = 170
        y_end = y_start + 310
        cropped_img = frame[y_start:y_end,:]

        img_hsv = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
        # hsv阈值设置
        mask = cv2.inRange(img_hsv, numpy.array([0, 0, 210]), numpy.array([179, 30, 255]))

        follow = mask.copy()
        follow,error= mid(follow, mask)

        derivative = error - previous_error
        control_signal = Kp * error + Kd * derivative  #比例微分控制
        if (-10 <= error <= 10):
            error = 0

        # control_signal = Kp * error     # 比例控制
        # print("error=%d" % error)
        # print(control_signal)
        previous_error = error
        # 根据差值进行p调节或者pd调节
        if stop_flag:
            stop_flag = stop_flag + 1
            if stop_flag >= 8:
                twist.linear.x = 0
                twist.angular.z = 0
            else:
                twist.linear.x = 0.2
                twist.angular.z = -control_signal
            print(stop_flag)
        else:
            twist.linear.x = 0.2
            twist.angular.z = -control_signal
        rospy.loginfo(stop_flag)
        # rospy.info("Publishing Twist: linear.x=%f, angular.z=%f", twist.linear.x, twist.angular.z);
        cmd_pub.publish(twist)
        rospy.loginfo("Publishing Twist: linear.x=%f, angular.z=%f", twist.linear.x, twist.angular.z)
        # rate.sleep()
        # cv2.imshow("img", img)
        # cv2.imshow("mask", mask)
        # cv2.imshow("follow", follow)
        # cv2.waitKey(1)
if __name__ == '__main__':
    main()

