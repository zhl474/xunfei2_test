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


int awake_flag = 0;
int terrorist_count = -1; // 全局变量，用于存储恐怖分子的数量
std_msgs::Int8 start_detect;//恐怖分子识别启动，为1时启动
std::string sound_file;  //音频文件的路径
bool terrorist_data_received = false; // 全局标志，表示是否已接收到恐怖分子数据

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void mic_awake_cb(const std_msgs::Int8::ConstPtr& a_p)
{
    awake_flag = a_p->data;             //传递a_p值
    ROS_INFO("awake flag is set");
}

void doMsg(const std_msgs::Int8::ConstPtr& msg_p){
    int terrorist_count = msg_p->data;   //改成整型的形式  
    if (terrorist_count != -1) {   
        switch (terrorist_count) {  
            case 1: sound_file = "~/ucar_car/src/simple_navigation_goals/voice/c1.wav";
                    terrorist_data_received = true;
                    break;  
            case 2: sound_file = "~/ucar_car/src/simple_navigation_goals/voice/c2.wav";
                    terrorist_data_received = true;
                    break;  
            case 3: sound_file = "~/ucar_car/src/simple_navigation_goals/voice/c3.wav";
                    terrorist_data_received = true; 
                    break;  
            default: ROS_WARN("Unknown terrorist count: %d", terrorist_count); break;  
        }
          
        // 注意：这里不再调用 system，因为播放将在主循环中控制  
        // system(("aplay " + sound_file).c_str());  
    }  
}//处理恐怖分子识别时获取的数据

void playAudio(const std::string& sound_file) {  
    system("aplay " + sound_file);    
}  //处理音频

int main(int argc, char** argv) {

    ros::Time::init();
    ros::Rate r(0.25); 
	setlocale(LC_ALL,"");
    ros::init(argc, argv, "send_goal_circle_final");
    
    
    ros::NodeHandle n;	

    ROS_INFO("GOOD");
    ROS_INFO("okkk!");
    ros::Publisher terrorist_pub = n.advertise<std_msgs::Int8>("began_terrorist",10);//开始识别信息发布
    ros::Subscriber awake_sub = n.subscribe<std_msgs::Int8>("awake_flag", 1, mic_awake_cb);  //语音唤醒订阅
    ros::Subscriber terrorist_sub = n.subscribe<std_msgs::Int8>("terrorist_data",10,doMsg);   //恐怖分子数量订阅
    

    start_detect.data = 0;      //初始化恐怖分子识别标志位为0
    while (!awake_flag)                  //等待语音唤醒标志位
    {
        ros::spinOnce();
    }
    ROS_INFO("RUNNING!");
    
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    ros::Time start_time = ros::Time::now();   
    move_base_msgs::MoveBaseGoal goal[9];

    //we'll send a goal to the robot to move 1 meter forward


    goal[0].target_pose.pose.position.x = 1.168;
    goal[0].target_pose.pose.position.y = 1.245;
    goal[0].target_pose.pose.position.z = 0.000;
    goal[0].target_pose.pose.orientation.x = 0.000;
    goal[0].target_pose.pose.orientation.y = 0.000;
    goal[0].target_pose.pose.orientation.z = -0.016;
    goal[0].target_pose.pose.orientation.w = 1.000;  //读取恐怖分子数量

    goal[1].target_pose.pose.position.x = 0.196;
    goal[1].target_pose.pose.position.y = 0.019;
    goal[1].target_pose.pose.position.z = 0.000;
    goal[1].target_pose.pose.orientation.x = 0.000;
    goal[1].target_pose.pose.orientation.y = 0.000;
    goal[1].target_pose.pose.orientation.z = 1.000;
    goal[1].target_pose.pose.orientation.w = 0.005; //过坡前的点

    goal[2].target_pose.pose.position.x = -1.914;
    goal[2].target_pose.pose.position.y = -0.043;
    goal[2].target_pose.pose.position.z = 0.000;
    goal[2].target_pose.pose.orientation.x = 0.000;
    goal[2].target_pose.pose.orientation.y = 0.000;
    goal[2].target_pose.pose.orientation.z = 1.000;
    goal[2].target_pose.pose.orientation.w = 0.005;  //过坡之后到的点

    goal[3].target_pose.pose.position.x = -2.353;
    goal[3].target_pose.pose.position.y = -1.670;
    goal[3].target_pose.pose.position.z = 0.000;
    goal[3].target_pose.pose.orientation.x = 0.000;
    goal[3].target_pose.pose.orientation.y = 0.000;
    goal[3].target_pose.pose.orientation.z = -0.715;
    goal[3].target_pose.pose.orientation.w = 0.699; //读取急救包位置

    goal[4].target_pose.pose.position.x = -2.211;
    goal[4].target_pose.pose.position.y = 0.648;
    goal[4].target_pose.pose.position.z = 0.000;
    goal[4].target_pose.pose.orientation.x = 0.000;
    goal[4].target_pose.pose.orientation.y = 0.000;
    goal[4].target_pose.pose.orientation.z = 0.932; 
    goal[4].target_pose.pose.orientation.w = 0.362; //读取急救物品区域

    goal[5].target_pose.pose.position.x = -1.628;
    goal[5].target_pose.pose.position.y = 0.052;
    goal[5].target_pose.pose.position.z = 0.000;
    goal[5].target_pose.pose.orientation.x = 0.000;
    goal[5].target_pose.pose.orientation.y = 0.000;
    goal[5].target_pose.pose.orientation.z = 0.001;
    goal[5].target_pose.pose.orientation.w = 1.000; //坡道前停止区域

    goal[6].target_pose.pose.position.x = 1.023;
    goal[6].target_pose.pose.position.y = -0.500;
    goal[6].target_pose.pose.position.z = 0.000;
    goal[6].target_pose.pose.orientation.x = 0.000;
    goal[6].target_pose.pose.orientation.y = 0.000;
    goal[6].target_pose.pose.orientation.z = 0.000;
    goal[6].target_pose.pose.orientation.w = 1.000; //坡道后停止区域

    goal[7].target_pose.pose.position.x = 0.994;
    goal[7].target_pose.pose.position.y = -0.454;
    goal[7].target_pose.pose.position.z = 0.000;
    goal[7].target_pose.pose.orientation.x = 0.000;
    goal[7].target_pose.pose.orientation.y = 0.000;
    goal[7].target_pose.pose.orientation.z = 0.018;
    goal[7].target_pose.pose.orientation.w = 1.000; //终点前停止区域


    goal[8].target_pose.pose.position.x = 2.062;
    goal[8].target_pose.pose.position.y = 0.066;
    goal[8].target_pose.pose.position.z = 0.000;
    goal[8].target_pose.pose.orientation.x = 0.000;
    goal[8].target_pose.pose.orientation.y = 0.000;
    goal[8].target_pose.pose.orientation.z = 0.704;
    goal[8].target_pose.pose.orientation.w = 0.711; //到达终点

    int i;
    
    for (i = 0; i < 9;i++) {
        goal[i].target_pose.header.frame_id = "map";
        goal[i].target_pose.header.stamp = ros::Time::now();
        ROS_INFO("Sending goal");
        ac.sendGoal(goal[i]);

        ac.waitForResult();

        while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {

        }
       

	    if(i==0){
            start_detect.data = 1;                  //识别标志位设为1，则开始识别
            terrorist_pub.publish(start_detect);
            while (!terrorist_data_received) {  
                ros::Duration(0.1).sleep(); // 短暂休眠
                ros::spinOnce(); // 处理回调  
            }
            playAudio(sound_file);                                                     
        }
        else if (i == 3 || i == 4 || i == 8) {  

            switch (i) {  
                case 3: sound_file= "~/ucar_car/src/simple_navigation_goals/voice/j0.wav"; break;  
                case 4: sound_file = "~/ucar_car/src/simple_navigation_goals/voice/j1.wav"; break;  
                case 8: sound_file = "~/ucar_car/src/simple_navigation_goals/voice/f.wav"; break;  
            }  
            playAudio(sound_file);
        }
        else{
            
        }
        ROS_INFO("Arrive at!");

    }
    return 0;
}
