#!/usr/bin/env python3
# coding=utf-8

import rospy
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import time
from line_follow.srv import line_follow,line_followRequest,line_followResponse
from ztestnav2025.srv import lidar_process,lidar_processRequest,lidar_processResponse

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0
        self.vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.vel_msg = Twist()
        
        # 创建服务客户端并等待服务可用
        self.lidar_client = rospy.ServiceProxy("lidar_process/lidar_process", lidar_process)
        rospy.loginfo("等待雷达服务")
        rospy.wait_for_service("lidar_process/lidar_process")  # 确保服务可用[7,8](@ref)
        
        # 创建服务服务器self.lidar_client.call(self.lidar_Req)
        self.server = rospy.Service("line_server", line_follow, self.doReq)
        
        # 初始化服务请求
        self.lidar_Req = lidar_processRequest()
        self.lidar_Req.lidar_process_start = -1

        
    def compute(self, error):
        self.integral += error
        integral_limited = np.clip(self.integral, -1000, 1000)
        
        derivative = error - self.last_error
        output = (self.Kp * error) + (self.Ki * integral_limited) + (self.Kd * derivative)
        
        self.last_error = error
        return output
    
    def reset(self):
        self.integral = 0
        self.last_error = 0

    def fixed_threshold_binarization(self,image, threshold=210, max_value=255, threshold_type=cv2.THRESH_BINARY_INV):
        try:
            # 转换为灰度图像
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # 只保留图像下半部分
            height = gray.shape[0]
            cropped_gray = gray[height//2+110:, :]  # 裁剪下半部分
            
            # 高斯滤波降噪
            blur = cv2.GaussianBlur(cropped_gray, (5,5), 0)
            
            # 固定阈值二值化
            _, binary = cv2.threshold(blur, threshold, max_value, threshold_type)
            
            return binary  
            
        except Exception as e:
            rospy.logwarn(f"二值化异常: {str(e)}")
            return np.zeros((1, 1), dtype=np.uint8)  # 返回空图像

            
    def detect_center_line(self,binary_image):
        height, width = binary_image.shape
        
        # 获取所有黑色像素位置
        nonzero_y, nonzero_x = np.nonzero(binary_image == 0)
        
        # 分割左右半区
        mid_x = width // 2
        left_mask = (nonzero_x < mid_x)
        right_mask = (nonzero_x >= mid_x)
        
        # 分别提取左右侧点
        nonzero_y_left = nonzero_y[left_mask]
        nonzero_x_left = nonzero_x[left_mask]
        nonzero_y_right = nonzero_y[right_mask]
        nonzero_x_right = nonzero_x[right_mask]
        
        # 判断使用哪侧作为基准线
        use_right = len(nonzero_y_right) >= len(nonzero_y_left)
        
        k1 = 1.5
        k2 = 1.5
        # 初始化基准线数据
        if use_right and len(nonzero_y_right) > 0:
            # 右侧基准线处理
            max_x_right = np.zeros(height, dtype=np.int32)
            np.maximum.at(max_x_right, nonzero_y_right, nonzero_x_right)
            valid_rows = (max_x_right > mid_x) & (max_x_right < width)
            y_coords = np.where(valid_rows)[0]
            x_coords = max_x_right[valid_rows]
            
                
            # 计算偏移中线（右侧减偏移）
            centers = np.zeros(height, dtype=np.int32)
            centers[valid_rows] = max_x_right[valid_rows] - (300 - k1 * (130 - y_coords))
            
        elif len(nonzero_y_left) > 0:
            # 左侧基准线处理
            min_x_left = np.full(height, width, dtype=np.int32)
            np.minimum.at(min_x_left, nonzero_y_left, nonzero_x_left)
            valid_rows = (min_x_left > 0) & (min_x_left < mid_x)
            y_coords = np.where(valid_rows)[0]
            x_coords = min_x_left[valid_rows]
            
                
            # 计算偏移中线（左侧加偏移）
            centers = np.zeros(height, dtype=np.int32)
            centers[valid_rows] = min_x_left[valid_rows] + (320 - k2 * (130 - y_coords))
            
        else:
            # 默认中线
            center_line_x = mid_x
            visual_img = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
            cv2.line(visual_img, (center_line_x, 0), (center_line_x, height-1), (0, 255, 255), 2)
            return center_line_x, visual_img

        # 限制中线范围
        centers = np.clip(centers, 0, width-1)

        # 加权平均计算最终中线
        weights = np.arange(height, 0, -1) * 2
        
        if np.any(valid_rows):
            weighted_sum = np.sum(centers[valid_rows] * weights[valid_rows])
            total_weight = np.sum(weights[valid_rows])

            if total_weight != 0:
                center_line_x = int(weighted_sum / total_weight)
            else:
                center_line_x = mid_x
        else:
            center_line_x = mid_x

        
        # 可视化
        visual_img = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
        cv2.line(visual_img, (center_line_x, 0), (center_line_x, height-1), (0, 255, 255), 2)
        
        for y in range(height):
            if valid_rows[y]:
                cv2.circle(visual_img, (centers[y], y), 2, (0, 0, 255), -1)

        return center_line_x, visual_img

    def detect_stopping_line(self,binary_image):
        height, width = binary_image.shape
        # 检查图像底部10%的区域
        stop_section = binary_image[int(height * 0.99):, :]
        
        for row in stop_section:
            black_pixels = np.sum(row == 0)  # 统计黑色像素数量
            if black_pixels > width * 0.25:   # 黑色像素超过80%
                return True
        return False

    def doReq(self, req):
        cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        if not cap.isOpened():
            rospy.logerr("打开摄像头失败")
            return line_followResponse(0)
        while not rospy.is_shutdown():
            rospy.loginfo("开始巡线")
            lidar_resp = self.lidar_client.call(self.lidar_Req)
            if lidar_resp.lidar_results[0] != -1:
                self.avoid_move()
            ret, frame = cap.read()
            if not ret:
                rospy.logerr("获取图片失败")
                continue

            frame = cv2.flip(frame, 1)
            
            binary_image = self.fixed_threshold_binarization(frame)
            
            if self.detect_stopping_line(binary_image):
                rospy.loginfo("检测到停车线，停止小车!")
                self.vel_msg = Twist()
                self.vel_msg.linear.x = 0.0
                self.vel_msg.angular.z = 0.0
                self.vel_publisher.publish(vel_msg)
                break  # 退出循环


            height, width = binary_image.shape
            # cv2.imshow('frame',frame)
            # cv2.imshow('Binary', binary_image)
            # cv2.waitKey(1)
            
            # 检测中线并获取中心点位置
            center_line_x,visual_img = self.detect_center_line(binary_image)


            cv2.imshow('Center Line Detection', visual_img)
            cv2.waitKey(1)
            
            height, width = frame.shape[:2]
            error = center_line_x - width // 2
            angular_z = pid.compute(error)
            angular_z = np.clip(angular_z, -0.25, 0.25)  # 限制角速度范围
            
            # 动态调整线速度
            max_speed = 0.3
            max_error = width // 2  # 图像中心到边缘的最大误差
            decay_factor = max(0.0, 1 - abs(error) / max_error)
            linear_x = max_speed * decay_factor

            
            self.vel_msg.linear.x = linear_x
            self.vel_msg.angular.z = angular_z  # 方向修正
            
            self.vel_publisher.publish(self.vel_msg)
            self.rate.sleep()
        resp = line_followResponse(1)
        return resp
    
    def avoid_move(self):
        rospy.loginfo("雷达发现障碍物，开始避障")
        flag = 1
        start = time.time()
        while flag :
            lidar_resp = self.lidar_client.call(self.lidar_Req)
            if lidar_resp.lidar_results[0] == -1:
                flag = 0
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = 0
            self.vel_msg.linear.y = 0.3
            self.vel_publisher.publish(self.vel_msg)
        end = time.time()
        totol_time = end - start
        flag = 1
        while flag :
            if time.time()-end > 1.5:
                flag = 0
            self.vel_msg.linear.x = 0.4
            self.vel_msg.angular.z = 0
            self.vel_msg.linear.y = 0
            self.vel_publisher.publish(self.vel_msg)
        flag = 1
        start = time.time()
        while flag :
            if time.time()-start > totol_time:
                flag = 0
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = 0
            self.vel_msg.linear.y = -0.3
            self.vel_publisher.publish(self.vel_msg)
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.vel_msg.linear.y = 0
        self.vel_publisher.publish(self.vel_msg)


# 初始化节点
rospy.init_node('line')
pid = PIDController(Kp=-4, Ki=0, Kd=-0.1)
rospy.loginfo("视觉巡线节点已启动!")

rospy.spin()