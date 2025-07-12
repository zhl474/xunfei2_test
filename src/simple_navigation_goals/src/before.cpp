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

std_msgs::Int8 start_detect;//恐怖分子识别启动，为1时启动

void doMsg(const std_msgs::Int8::ConstPtr& msg_p){
    int terrorist_count = msg_p->data;   //改成整型的形式  
    if (terrorist_count != -1) {   
            ROS_INFO("%d",terrorist_count);
        }
    }

int main(int argc, char** argv) {

    ros::Time::init();
    ros::Rate r(10); 
	setlocale(LC_ALL,"");
    ros::init(argc, argv, "send_goal_circle_final");
    
    
    ros::NodeHandle n;	

    ROS_INFO("GOOD");
    ROS_INFO("okkk!");
    ros::Publisher terrorist_pub = n.advertise<std_msgs::Int8>("start_detect",10);//开始识别信息发布
    ros::Subscriber terrorist_sub = n.subscribe<std_msgs::Int8>("terrorist_data",10,doMsg);   //恐怖分子数量订阅
    while(true){
        start_detect.data = 0;      //初始化恐怖分子识别标志位为0
        start_detect.data = 1;                  //识别标志位设为1，则开始识别
        terrorist_pub.publish(start_detect);
        ROS_INFO("GOOD1");
        r.sleep();
    }
    ros::spinOnce();

    
    return 0;
    }
