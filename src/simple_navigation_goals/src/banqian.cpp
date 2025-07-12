#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int8.h>
#include "std_msgs/Int32.h"  // 添加头文件
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include "actionlib/client/simple_goal_state.h"
#include "actionlib/client/simple_client_goal_state.h"
#include "actionlib/client/terminal_state.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <cstdlib>
#include <math.h>
#include "amcl/AMCLConfig.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// 全局变量
int g_target_detected = 0; //目标标志
int max_rotate_attempts = 4; //最大旋转次数
int current_point_index = 0; //当前点位
float g_target_x_error = 0; //x像素中心点误差
const float distance_threshold = 1.0;  // 设定前方距离阈值（单位：米）
bool target_found = false; 
int target_detected_fb = 0;
int g_target_detected_fb = 0;
int g_target_detected1 = 0;

ros::Publisher vel_pub;
ros::Publisher detect_pub;
std_msgs::Int8 start_detect;           //控制识别启动，1时启动
sensor_msgs::LaserScan::ConstPtr g_laser_scan; //雷达测距

move_base_msgs::MoveBaseGoal point[2];
move_base_msgs::MoveBaseGoal point_z[4];
move_base_msgs::MoveBaseGoal goal[3];


//相机目标图像像素x中点回调函数
void target_detected_x_cb(const std_msgs::Float64::ConstPtr& r){
    g_target_x_error = r->data;
    ROS_INFO("g_target_x_error=%f",g_target_x_error);
}
// 激光雷达回调函数
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    g_laser_scan = msg;
}
// 视觉进入识别判断回调函数
void target_detected_fb_cb(const std_msgs::Int8::ConstPtr& r1){
    target_detected_fb = r1->data;
}
//相机目标图像订阅回调函数
void target_detected_cb(const std_msgs::Int8::ConstPtr& r_p){
    g_target_detected = r_p->data;
    ROS_INFO("g_target_detected=%d",g_target_detected);
    if(g_target_detected == 2){
        g_target_detected = 0;
        g_target_detected1 = 1;
    }
}
void controlRobot(MoveBaseClient &ac){
    ROS_INFO("control");
    ros::Rate rate(10); // 控制频率 10 Hz
    geometry_msgs::Twist cmd_vel_msg;
    int rotate_attempts = 0;
    // 1. 旋转小车，直到目标物在相机视野内或旋转了一圈，g_target_detected是相机里面有目标物
    while (ros::ok() && rotate_attempts < max_rotate_attempts) {
        target_detected_fb = 0;
        g_target_detected == 0;
        // 到达识别点，开启识别；
        start_detect.data = 1;
        detect_pub.publish(start_detect);
        // 等待视觉反馈识别
        // while(target_detected_fb != 1){
        //     ROS_INFO("target_detected_fb");
        //     ros::Duration(0.1).sleep(); // 短暂休眠
        //     ros::spinOnce(); // 处理回调 
        // }
        while (g_target_detected == 0){ 
            ROS_INFO("g_target_detected");
            point_z[rotate_attempts].target_pose.header.frame_id = "map";
            point_z[rotate_attempts].target_pose.header.stamp = ros::Time::now();
            if(g_target_detected1 == 1){
                ROS_INFO("g_target_detected = 1");
                ac.sendGoal(point_z[rotate_attempts]);
                ac.waitForResult();
                while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
                { 

                } 
                rotate_attempts++;
            }
            ros::Duration(3).sleep(); // 短暂休眠
            ros::spinOnce(); // 处理回调 
        }
        
        if (g_target_detected == 1)
        { 
            ROS_INFO("find goal");
        }
    if (g_target_detected != 1) {
        // 全方位都没有检测到目标物，退出当前循环，选择另一个点进行操作
        ROS_WARN("No target found in the surrounding area, switching to the next point.");
        current_point_index++; //
        return;
    }
    // bool target_link_to=false;
    // 2. 调整小车，使目标物的中心点与相机中心点对齐 ,g_target_x_error是目标的x轴中心像素误差
    while (ros::ok()) {
        ROS_INFO("g_target_x_error");
        float k = 0.05; //根据角度与像素差调整 
        if(g_target_x_error != 0){
            cmd_vel_msg.angular.z = k*g_target_x_error; // 根据目标物角度调整方向
            vel_pub.publish(cmd_vel_msg);   
        }
        else{
            break;
        }
        start_detect.data = 1;
        detect_pub.publish(start_detect);
        ros::spinOnce();
        rate.sleep();
        
    }
    //标志位 
    target_found = true;
    //正对后，根据雷达测距
    // 3. 检测前方距离，确定是否前进或停止
    if (g_laser_scan) {
        int front_index = g_laser_scan->ranges.size() / 2;
        float front_distance = g_laser_scan->ranges[front_index];

        if (front_distance > distance_threshold) {
            // 前方距离大于阈值，继续前进
            cmd_vel_msg.linear.x = 0.5; // 设置前进速度
        } else {
            // 前方距离小于等于阈值，停止并播报
            cmd_vel_msg.linear.x = 0.0;
            return;

        }
        // 发布速度控制指令
        vel_pub.publish(cmd_vel_msg);
        rate.sleep();
    }   
    
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "visual_lidar_controller");
    ros::Time::init();
    ros::Rate r(1); 
	setlocale(LC_ALL,"");
    ros::NodeHandle nh;

    detect_pub = nh.advertise<std_msgs::Int8>("start_detect1",1);  //识别控制发布者
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10); // 速度控制发布者

    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 10, laserCallback); // 激光雷达订阅者
    ros::Subscriber target_detected_fb_sub = nh.subscribe<std_msgs::Int8>("target_detected_fb",1,target_detected_fb_cb);//相机目标图像订阅者
    ros::Subscriber target_detected_sub = nh.subscribe<std_msgs::Int8>("target_detected",1,target_detected_cb);//相机目标图像订阅者
    ros::Subscriber target_detected_x_sub = nh.subscribe<std_msgs::Float64>("target_detected_x",1,target_detected_x_cb);//相机目标图像像素中点误差订阅者

    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(2.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    start_detect.data = 0;      //预初始化识别标志位为0

    ros::Time start_time = ros::Time::now();   


    goal[0].target_pose.pose.position.x = 0.975;
    goal[0].target_pose.pose.position.y = 1.335;
    goal[0].target_pose.pose.position.z = 0.000;
    goal[0].target_pose.pose.orientation.x = 0.000;
    goal[0].target_pose.pose.orientation.y = 0.000;
    goal[0].target_pose.pose.orientation.z = 0.001;
    goal[0].target_pose.pose.orientation.w = 1.000;  //读取恐怖分子数量

    goal[1].target_pose.pose.position.x = 0.196;
    goal[1].target_pose.pose.position.y = 0.019;
    goal[1].target_pose.pose.position.z = 0.000;
    goal[1].target_pose.pose.orientation.x = 0.000;
    goal[1].target_pose.pose.orientation.y = 0.000;
    goal[1].target_pose.pose.orientation.z = 1.000;
    goal[1].target_pose.pose.orientation.w = 0.005; //过坡前的点

    goal[2].target_pose.pose.position.x = -1.866;
    goal[2].target_pose.pose.position.y = 0.039;
    goal[2].target_pose.pose.position.z = 0.000;
    goal[2].target_pose.pose.orientation.x = 0.000;
    goal[2].target_pose.pose.orientation.y = 0.000;
    goal[2].target_pose.pose.orientation.z = 1.000;
    goal[2].target_pose.pose.orientation.w = 0.012;  //过坡之后到的点

    point[0].target_pose.pose.position.x = 1.0;
    point[0].target_pose.pose.position.y = 0.0;
    point[0].target_pose.pose.orientation.z = -0.016;
    point[0].target_pose.pose.orientation.w = 1.000; 

    point[1].target_pose.pose.position.x = 0.0;
    point[1].target_pose.pose.position.y = 1.0;
    point[1].target_pose.pose.orientation.z = -0.016;
    point[1].target_pose.pose.orientation.w = 1.000; 

    point_z[0].target_pose.pose.position.x = -1.855;
    point_z[0].target_pose.pose.position.y = -0.001;
    point_z[0].target_pose.pose.orientation.z = 0.9655;
    point_z[0].target_pose.pose.orientation.w = 0.2605; 

    point_z[1].target_pose.pose.position.x = -1.855;
    point_z[1].target_pose.pose.position.y = -0.001;
    point_z[1].target_pose.pose.orientation.z = -0.9661;
    point_z[1].target_pose.pose.orientation.w = -0.2579;

    point_z[2].target_pose.pose.position.x = -1.855;
    point_z[2].target_pose.pose.position.y = -0.001;
    point_z[2].target_pose.pose.orientation.z = 0.4577;
    point_z[2].target_pose.pose.orientation.w = -0.8890;

    point_z[3].target_pose.pose.position.x = -1.855;
    point_z[3].target_pose.pose.position.y = -0.001;
    point_z[3].target_pose.pose.orientation.z = -0.4577;
    point_z[3].target_pose.pose.orientation.w = -0.8891;
    int i;
    for ( i = 0; i < 3;i++) {
        goal[i].target_pose.header.frame_id = "map";
        goal[i].target_pose.header.stamp = ros::Time::now();

        point[i].target_pose.header.frame_id = "map";
        point[i].target_pose.header.stamp = ros::Time::now();

        point_z[i].target_pose.header.frame_id = "map";
        point_z[i].target_pose.header.stamp = ros::Time::now();
    
        ROS_INFO("Sending goal");
        ac.sendGoal(goal[i]);

        ac.waitForResult();

        while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {

        }
        if(i == 2){
            while (ros::ok() && current_point_index < 2) {
                ros::spinOnce(); // 处理回调函数
                while(target_found == false){
                    // if(current_point_index == 1){
                    //     goal[i].target_pose.header.frame_id = "map";
                    //     goal[i].target_pose.header.stamp = ros::Time::now();
                    //     ROS_INFO("Sending goal");
                    //     ac.sendGoal(point[1]);
                    //     ac.waitForResult();

                    //     while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
                    //     {

                    //     }
                    // }
                    // if(current_point_index == 2){
                    //     ac.sendGoal(point[2]);
                    //     ac.waitForResult();

                    //     while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
                    //     {

                    //     }
                    // }
                    controlRobot(ac);
            }
            break;
        }
        }
        }
        return 0;
}

   
