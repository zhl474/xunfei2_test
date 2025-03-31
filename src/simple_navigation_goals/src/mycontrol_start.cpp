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

int goal_num;
int awake_flag=0;      //语音唤醒标志位

double goal0_x;
double goal0_y;
double goal0_oz;
double goal0_ow;

double goal1_x;
double goal1_y;
double goal1_oz;
double goal1_ow;

double goal2_x;
double goal2_y;
double goal2_oz;
double goal2_ow;

double goal3_x;
double goal3_y;
double goal3_oz;
double goal3_ow;

double goal4_x;
double goal4_y;
double goal4_oz;
double goal4_ow;

double goal5_x;
double goal5_y;
double goal5_oz;
double goal5_ow;

double goal6_x;
double goal6_y;
double goal6_oz;
double goal6_ow;

double goal7_x;
double goal7_y;
double goal7_oz;
double goal7_ow;

double goal8_x;
double goal8_y;
double goal8_oz;
double goal8_ow;

double goal_list[9][4];

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


move_base_msgs::MoveBaseGoal goal;

void get_param()
{
    ros::param::get("mycontrol_start/goal_num",goal_num);

    ros::param::get("mycontrol_start/goal0_x",goal0_x);
    ros::param::get("mycontrol_start/goal0_y",goal0_y);
    ros::param::get("mycontrol_start/goal0_oz",goal0_oz);
    ros::param::get("mycontrol_start/goal0_ow",goal0_ow);

    ros::param::get("mycontrol_start/goal1_x",goal1_x);
    ros::param::get("mycontrol_start/goal1_y",goal1_y);
    ros::param::get("mycontrol_start/goal1_oz",goal1_oz);
    ros::param::get("mycontrol_start/goal1_ow",goal1_ow);

    ros::param::get("mycontrol_start/goal2_x",goal2_x);
    ros::param::get("mycontrol_start/goal2_y",goal2_y);
    ros::param::get("mycontrol_start/goal2_oz",goal2_oz);
    ros::param::get("mycontrol_start/goal2_ow",goal2_ow);

    ros::param::get("mycontrol_start/goal3_x",goal3_x);
    ros::param::get("mycontrol_start/goal3_y",goal3_y);
    ros::param::get("mycontrol_start/goal3_oz",goal3_oz);
    ros::param::get("mycontrol_start/goal3_ow",goal3_ow);

    ros::param::get("mycontrol_start/goal4_x",goal4_x);
    ros::param::get("mycontrol_start/goal4_y",goal4_y);
    ros::param::get("mycontrol_start/goal4_oz",goal4_oz);
    ros::param::get("mycontrol_start/goal4_ow",goal4_ow);

    ros::param::get("mycontrol_start/goal5_x",goal5_x);
    ros::param::get("mycontrol_start/goal5_y",goal5_y);
    ros::param::get("mycontrol_start/goal5_oz",goal5_oz);
    ros::param::get("mycontrol_start/goal5_ow",goal5_ow);

    ros::param::get("mycontrol_start/goal6_x",goal6_x);
    ros::param::get("mycontrol_start/goal6_y",goal6_y);
    ros::param::get("mycontrol_start/goal6_oz",goal6_oz);
    ros::param::get("mycontrol_start/goal6_ow",goal6_ow);

    ros::param::get("mycontrol_start/goal7_x",goal7_x);
    ros::param::get("mycontrol_start/goal7_y",goal7_y);
    ros::param::get("mycontrol_start/goal7_oz",goal7_oz);
    ros::param::get("mycontrol_start/goal7_ow",goal7_ow);

    ros::param::get("mycontrol_start/goal8_x",goal8_x);
    ros::param::get("mycontrol_start/goal8_y",goal8_y);
    ros::param::get("mycontrol_start/goal8_oz",goal8_oz);
    ros::param::get("mycontrol_start/goal8_ow",goal8_ow);
    
    ROS_INFO("get goal param succeefully!");    //获取目标点参数
    ROS_INFO("goal_num is %d ",goal_num);    //获取目标点参数
   
}


void init_goallist()        //将目标点存入二维数组goal_list
{
    goal_list[0][0]=goal0_x;
    goal_list[0][1]=goal0_y;
    goal_list[0][2]=goal0_oz;
    goal_list[0][3]=goal0_ow;

    goal_list[1][0]=goal1_x;
    goal_list[1][1]=goal1_y;
    goal_list[1][2]=goal1_oz;
    goal_list[1][3]=goal1_ow;

    goal_list[2][0]=goal2_x;
    goal_list[2][1]=goal2_y;
    goal_list[2][2]=goal2_oz;
    goal_list[2][3]=goal2_ow;

    goal_list[3][0]=goal3_x;
    goal_list[3][1]=goal3_y;
    goal_list[3][2]=goal3_oz;
    goal_list[3][3]=goal3_ow;

    goal_list[4][0]=goal4_x;
    goal_list[4][1]=goal4_y;
    goal_list[4][2]=goal4_oz;
    goal_list[4][3]=goal4_ow;

    goal_list[5][0]=goal5_x;
    goal_list[5][1]=goal5_y;
    goal_list[5][2]=goal5_oz;
    goal_list[5][3]=goal5_ow;

    goal_list[6][0]=goal6_x;
    goal_list[6][1]=goal6_y;
    goal_list[6][2]=goal6_oz;
    goal_list[6][3]=goal6_ow;

    goal_list[7][0]=goal7_x;
    goal_list[7][1]=goal7_y;
    goal_list[7][2]=goal7_oz;
    goal_list[7][3]=goal7_ow;

    goal_list[8][0]=goal8_x;
    goal_list[8][1]=goal8_y;
    goal_list[8][2]=goal8_oz;
    goal_list[8][3]=goal8_ow;

    ROS_INFO("goal_list init succeefully");    //获取目标点参数

}



void mic_awake_cb(const std_msgs::Int8::ConstPtr& a_p)
{
    awake_flag = a_p->data;             //传递a_p值
    ROS_INFO("awake flag is set");
}



int main(int argc, char** argv)
{
    ros::Time::init();
    ros::Rate r(0.25);
	ros::init(argc, argv, "mycontrol_start");
	ros::NodeHandle n;


    ros::Subscriber awake_sub = n.subscribe<std_msgs::Int8>("awake_flag",1,mic_awake_cb);

    
    while(!awake_flag)
    {
	//ROS_INFO("%d ",awake_flag);
        ros::spinOnce();
    }

    ROS_INFO("control_start_node start running!");

    get_param();                    //获取目标点参数
    init_goallist();                //初始化目标点列表

    MoveBaseClient ac("move_base", true); 
    
    //等待action回应
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    goal.target_pose.header.frame_id = "map";
    
    for(int i=0;i<goal_num;i++)        //循环次数为目标点个数，发布目标点
    {
        
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = goal_list[i][0];
        goal.target_pose.pose.position.y = goal_list[i][1];
        goal.target_pose.pose.orientation.z = goal_list[i][2];
        goal.target_pose.pose.orientation.w = goal_list[i][3];

	    ROS_INFO("Sending goal -- %d",i);
        ac.sendGoal(goal);
        

        ac.waitForResult();
        while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
            // ROS_INFO("running!");
        }
        if(i==0){
            system("aplay ~/ucar_car/src/simple_navigation_goals/voice/c1.wav");
            r.sleep();
        }else if(i==3){
            system("aplay ~/ucar_car/src/simple_navigation_goals/voice/j0.wav");
            r.sleep();
        }else if(i==4){
            system("aplay ~/ucar_car/src/simple_navigation_goals/voice/j1.wav");
            r.sleep();
        }else if(i==8){
            system("aplay ~/ucar_car/src/simple_navigation_goals/voice/f.wav");
            r.sleep();
        }else{

        }  
        ROS_INFO("Arrive at!");
    }

    
    ros::spin();

    return 0;
}


