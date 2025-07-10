#!/usr/bin/env python3
# coding=utf-8

import rospy
# 新增PoseStamped, Point, Quaternion
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
import cv2
import numpy as np
import time
from line_follow.srv import line_follow,line_followRequest,line_followResponse
from ztestnav2025.srv import lidar_process,lidar_processRequest,lidar_processResponse
# from ztestnav2025.srv import getpose_server,getpose_serverRequest,getpose_serverResponse
from nav_msgs.msg import Odometry  # 导入ROS的里程计消息类型
# 新增数学运算
from ztestnav2025.srv import getpose_server, getpose_serverRequest
import math
# 新增move_base和tf的库
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal






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


         # 里程计初始化 (原YourClass的内容)
        self.current_pose = None
        self.current_orientation = None # 新增：用于存储朝向
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        # 速度控制初始化
        self.vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.vel_msg = Twist()



        # move_base Action客户端初始化
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("正在等待move_base服务...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("move_base服务已连接。")


        
        # 创建服务客户端并等待服务可用
        self.lidar_client = rospy.ServiceProxy("lidar_process/lidar_process", lidar_process)
        rospy.loginfo("等待雷达服务")
        rospy.wait_for_service("lidar_process/lidar_process")  # 确保服务可用[7,8](@ref)

        # 【新增】位姿获取服务客户端
        rospy.loginfo("等待getpose_server服务...")
        rospy.wait_for_service("getpose_server")
        self.get_pose_client = rospy.ServiceProxy("getpose_server", getpose_server)
        rospy.loginfo("getpose_server服务已连接。")




        
        # 创建服务服务器self.lidar_client.call(self.lidar_Req)
        self.server = rospy.Service("line_server", line_follow, self.doReq)
        
        # 初始化服务请求
        self.lidar_Req = lidar_processRequest()
        self.lidar_Req.lidar_process_start = -1

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation #新增，获取小车朝向
        
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

    def fixed_threshold_binarization(self,image, threshold=180, max_value=255, threshold_type=cv2.THRESH_BINARY_INV):
        try:
            # 转换为灰度图像
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # 只保留图像下半部分
            height = gray.shape[0]
            cropped_gray = gray[height//2+30:, :]  # 裁剪下半部分
            
            # 高斯滤波降噪
            blur = cv2.GaussianBlur(cropped_gray, (5,5), 0)
            
            # 固定阈值二值化
            _, binary = cv2.threshold(blur, threshold, max_value, threshold_type)
            
            return binary  
            
        except Exception as e:
            rospy.logwarn(f"二值化异常: {str(e)}")
            return np.zeros((1, 1), dtype=np.uint8)  # 返回空图像

            # 新增的模式：为避障获取精确前方距离和角度
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
        
        k1 = 1.0
        k2 = 1.0
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
            centers[valid_rows] = max_x_right[valid_rows] - (420 - k1 * (210 - y_coords))
            
        elif len(nonzero_y_left) > 0:#新增的模式：为避障获取精确前方距离和角度
            # 左侧基准线处理
            min_x_left = np.full(height, width, dtype=np.int32)
            np.minimum.at(min_x_left, nonzero_y_left, nonzero_x_left)
            valid_rows = (min_x_left > 0) & (min_x_left < mid_x)
            y_coords = np.where(valid_rows)[0]
            x_coords = min_x_left[valid_rows]
            
                
            # 计算偏移中线（左侧加偏移）
            centers = np.zeros(height, dtype=np.int32)
            centers[valid_rows] = min_x_left[valid_rows] + (420 - k2 * (210 - y_coords))
            
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
            # print(lidar_resp)
            if lidar_resp.lidar_results[0] != -1:
                # cv2.waitKey(0)
                rospy.loginfo("准备开始避障")
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
                self.vel_publisher.publish(self.vel_msg)#这里原来的vel_msg应该是错的
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
            angular_z = np.clip(angular_z, -0.27, 0.27)  # 限制角速度范围
            
            # 动态调整线速度
            max_speed = 0.2
            max_error = width // 2  # 图像中心到边缘的最大误差
            decay_factor = max(0.0, 1 - abs(error) / max_error)
            linear_x = max_speed * decay_factor

            
            self.vel_msg.linear.x = linear_x
            self.vel_msg.angular.z = angular_z  # 方向修正
            
            self.vel_publisher.publish(self.vel_msg)
            self.rate.sleep()
        resp = line_followResponse(1)
        return resp
    #-----------------------------------原本的避障代码---------------------------------------------
    # def avoid_move(self):
    #     rospy.loginfo("雷达发现障碍物，开始避障")
    #     flag = 1
    #     safe_distance = 1.0 
    #     # integration_y = 0
    #     integration_z = 0
    #     while flag:
    #         lidar_resp = self.lidar_client.call(self.lidar_Req)
    #         if(lidar_resp.lidar_results[0]!=-1 and 
    #             lidar_resp.lidar_results[0] < safe_distance ):
    #             # integration_y = max(min(integration_y + lidar_resp.lidar_results[1],-0.1),0.1)
    #             integration_z = max(min(integration_z + lidar_resp.lidar_results[2]/lidar_resp.lidar_results[3],-0.1),0.1)
    #             # self.vel_msg.linear.y = max(min(lidar_resp.lidar_results[1]+integration_y, 0.1), -0.1)
    #             self.vel_msg.angular.z = max(min((lidar_resp.lidar_results[2]/lidar_resp.lidar_results[3]+integration_z) * -1, 0.1), -0.1)
    #             self.vel_publisher.publish(self.vel_msg) 
    #             # if abs(self.vel_msg.linear.y) < 0.03:
    #             #     integration_y = 0
    #             if abs(self.vel_msg.angular.z) > 0.1:
    #                 integration_z = 0
    #             if abs(self.vel_msg.linear.y) < 0.03 and abs(self.vel_msg.angular.z) < 0.05:
    #                 print("done")
    #                 break

    #     # 横向移动（仅替换计时逻辑）
    #     target_distance = 0.6  # 原代码对应0.3m/s * 2s
    #     start_pose = self.current_pose  # 记录起点
    #     while flag :
    #         moved_distance = abs(self.current_pose.y - start_pose.y)  # 计算已移动距离
    #         if moved_distance >= target_distance:
    #             break
    #         self.vel_msg.linear.x = 0
    #         self.vel_msg.angular.z = 0
    #         self.vel_msg.linear.y = 0.3
    #         self.vel_publisher.publish(self.vel_msg)
    #     flag = 1
    #     start = time.time()
    #     while flag :
    #         if time.time()-start > 1.4:
    #             flag = 0
    #         self.vel_msg.linear.x = 0.4
    #         self.vel_msg.angular.z = 0
    #         self.vel_msg.linear.y = 0
    #         self.vel_publisher.publish(self.vel_msg)
    #     flag = 1
    #     target_distance = 0.6
    #     start_pose = self.current_pose
    #     while flag and self.current_pose:
    #         moved_distance = abs(self.current_pose.y - start_pose.y)
    #         if moved_distance >= target_distance:
    #             break
    #         self.vel_msg.linear.x = 0ucar_car
    #         self.vel_msg.angular.z = 0
    #         self.vel_msg.linear.y = -0.3
    #         self.vel_publisher.publish(self.vel_msg)
    #     self.vel_msg.linear.x = 0
    #     self.vel_msg.angular.z = 0
    #     self.vel_msg.linear.y = 0
    #     self.vel_publisher.publish(self.vel_msg)
        #--------------------------------------分界线---------------------------------------

    #----------------------------------------------新的避障代码-------------------------
    def avoid_move(self):
        
        #新的避障逻辑: 使用move_base导航到障碍物后方30cm处。
        
        rospy.loginfo("检测到广域障碍物，切换至精确避障模式。")

        # 1. 使用新的服务模式(start=0)调用雷达服务，获取精确前方障碍物信息
        avoid_req = lidar_processRequest()
        avoid_req.lidar_process_start = 0  # 使用我们定义的新模式
        try:
            lidar_resp = self.lidar_client.call(avoid_req)
        except rospy.ServiceException as e:
            rospy.logerr(f"精确避障服务调用失败: {e}")
            return

        # 2. 检查服务返回结果，格式应为: [min_dist, angle] 或 [-1]
        if not lidar_resp.lidar_results or lidar_resp.lidar_results[0] == -1:
            rospy.logwarn("在正前方特定范围内未发现障碍物，取消本次避障。")
            return

        min_dist = lidar_resp.lidar_results[0]
        angle = lidar_resp.lidar_results[1]
        
        # 设定安全距离，如果障碍物太远则忽略
        safe_distance = 0.5 
        if min_dist > safe_distance:
            rospy.loginfo(f"障碍物在安全距离 {safe_distance}m 之外 ({min_dist:.2f}m)，继续巡线。")
            return


        #调用getpose_server服务获取机器人当前在map下的位姿
        try:
            pose_req = getpose_serverRequest()
            pose_resp = self.get_pose_client.call(pose_req)
            # 服务返回 [x, y, yaw]
            robot_x_map = pose_resp.pose_at[0]
            robot_y_map = pose_resp.pose_at[1]
            robot_yaw_map = pose_resp.pose_at[2]
        except rospy.ServiceException as e:
            rospy.logerr(f"调用 getpose_server 服务失败: {e}")
            return

        # 3. 计算目标点
        # 目标点位于障碍物后方25cm处，沿着雷达探测到的方向
        target_dist = min_dist + 0.25

        # 在机器人坐标系 (base_link) 中计算目标位置
        # ROS标准坐标系: X轴向前, Y轴向左, 角度逆时针为正
        x_robot_frame = target_dist * math.cos(angle)
        y_robot_frame = target_dist * math.sin(angle)


        #手动将目标点从机器人坐标系(base_link)转换到世界坐标系(map)
        cos_yaw = math.cos(robot_yaw_map)
        sin_yaw = math.sin(robot_yaw_map)
        
        # 2D 旋转+平移公式
        target_x_map = robot_x_map + (x_robot_frame * cos_yaw - y_robot_frame * sin_yaw)
        target_y_map = robot_y_map + (x_robot_frame * sin_yaw + y_robot_frame * cos_yaw)
        
        rospy.loginfo(f"计算出 map 坐标系目标点: x={target_x_map:.2f}, y={target_y_map:.2f}")
        #--------------------------------新加上避障后朝向与避障前朝向关于板的法线对称----------------------
        # 将雷达坐标系下的障碍物法线角度(angle)转换到map世界坐标系下
        obstacle_normal_map_frame = robot_yaw_map + angle

        # 根据反射定律 (新朝向 = 2*法线朝向 - 旧朝向) 计算最终朝向
        new_robot_yaw_map = 2 * obstacle_normal_map_frame - robot_yaw_map

        #  标准化最终朝向角到[-pi, pi]范围内
        new_robot_yaw_map = math.atan2(math.sin(new_robot_yaw_map), math.cos(new_robot_yaw_map))
        
        rospy.loginfo(f"原朝向: {math.degrees(robot_yaw_map):.2f}度 | "f"计算出的新朝向: {math.degrees(new_robot_yaw_map):.2f}度")






        
        #--------------------------------分割线-------------------------------------------------------
        # 6. 创建并发送move_base目标
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = target_x_map
        goal.target_pose.pose.position.y = target_y_map

        # half_yaw = robot_yaw_map * 0.5
        half_yaw = new_robot_yaw_map * 0.5
        # 计算sin和cos
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        # 创建四元数（x, y设置为0）
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw

        # # 【新增】将yaw朝向角转换为四元数
        # q = quaternion_from_euler(0, 0, robot_yaw_map)
        # goal.target_pose.pose.orientation = Quaternion(*q)
        
        rospy.loginfo("向move_base发送导航目标...")
        self.move_base_client.send_goal(goal)

        # 6. 等待move_base完成任务
        finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(60.0))

        if finished_within_time and self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("避障成功，已到达目标点。")
        else:
            rospy.logwarn("避障失败，move_base未能到达目标点或已超时。")
            self.move_base_client.cancel_goal()


        # 避障结束后，控制流将自动返回到doReq循环中，继续巡线













    # def avoid_move(self):
    #     rospy.loginfo("雷达发现障碍物，开始避障")
    #     flag = 1
    #     # integration_y = 0
    #     integration_z = 0
    #     self.vel_msg.linear.y = 0
    #     self.vel_msg.linear.x = 0
    #     self.vel_msg.angular.z = 0
    #     while flag:
    #         lidar_resp = self.lidar_client.call(self.lidar_Req)
    #         if(lidar_resp.lidar_results[0]!=-1):
    #             # integration_y = max(min(integration_y + lidar_resp.lidar_results[1],-0.1),0.1)
    #             integration_z = max(min(integration_z + lidar_resp.lidar_results[2]/lidar_resp.lidar_results[3],-0.2),0.2)
    #             # self.vel_msg.linear.y = max(min(lidar_resp.lidar_results[1]+integration_y, 0.1), -0.1)
    #             self.vel_msg.angular.z = max(min((lidar_resp.lidar_results[2]/lidar_resp.lidar_results[3]+integration_z) * -1, 0.2), -0.2)
    #             self.vel_publisher.publish(self.vel_msg) 
    #             # if abs(self.vel_msg.linear.y) < 0.03:
    #             #     integration_y = 0
    #             print(self.vel_msg.angular.z)
    #             if abs(self.vel_msg.angular.z) > 0.2:
    #                 integration_z = 0
    #             if abs(self.vel_msg.angular.z) < 0.05:
    #                 print("done")
    #                 break
    #     rospy.loginfo("平移")
    #     start = time.time()
    #     while flag :
    #         if time.time()-start > 2:
    #             flag = 0
    #         self.vel_msg.linear.x = 0
    #         self.vel_msg.angular.z = 0
    #         self.vel_msg.linear.y = 0.3
    #         self.vel_publisher.publish(self.vel_msg)
    #     flag = 1
    #     start = time.time()
    #     while flag :
    #         if time.time()-start > 1.6:
    #             flag = 0
    #         self.vel_msg.linear.x = 0.4
    #         self.vel_msg.angular.z = 0
    #         self.vel_msg.linear.y = 0
    #         self.vel_publisher.publish(self.vel_msg)
    #     flag = 1
    #     start = time.time()
    #     while flag :
    #         if time.time()-start > 2:
    #             flag = 0
    #         self.vel_msg.linear.x = 0
    #         self.vel_msg.angular.z = 0
    #         self.vel_msg.linear.y = -0.3
    #         self.vel_publisher.publish(self.vel_msg)
    #     self.vel_msg.linear.x = 0
    #     self.vel_msg.angular.z = 0
    #     self.vel_msg.linear.y = 0
    #     self.vel_publisher.publish(self.vel_msg)


# 初始化节点
rospy.init_node('line')
pid = PIDController(Kp=-0.3, Ki=0, Kd=-0.02)
rospy.loginfo("视觉巡线节点已启动!")

rospy.spin()