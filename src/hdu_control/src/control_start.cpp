#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <ros/time.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include "actionlib/client/simple_goal_state.h"
#include "actionlib/client/simple_client_goal_state.h"
#include "actionlib/client/terminal_state.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>

int stop_point;
int goal_num;
int awake_flag=0;      //语音唤醒标志位

double goal0_x;
double goal0_y;
double goal0_z;

double goal1_x;
double goal1_y;
double goal1_z;

double goal2_x;
double goal2_y;
double goal2_z;

double goal3_x;
double goal3_y;
double goal3_z;

double goal4_x;
double goal4_y;
double goal4_z;

double goal5_x;
double goal5_y;
double goal5_z;

double goal6_x;
double goal6_y;
double goal6_z;

double goal_list[7][3];

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


move_base_msgs::MoveBaseGoal goal;

void get_param()
{
    ros::param::get("control_start/goal_num",goal_num);

    ros::param::get("control_start/goal0_x",goal0_x);
    ros::param::get("control_start/goal0_y",goal0_y);
    ros::param::get("control_start/goal0_z",goal0_z);

    ros::param::get("control_start/goal1_x",goal1_x);
    ros::param::get("control_start/goal1_y",goal1_y);
    ros::param::get("control_start/goal1_z",goal1_z);

    ros::param::get("control_start/goal2_x",goal2_x);
    ros::param::get("control_start/goal2_y",goal2_y);
    ros::param::get("control_start/goal2_z",goal2_z);

    ros::param::get("control_start/goal3_x",goal3_x);
    ros::param::get("control_start/goal3_y",goal3_y);
    ros::param::get("control_start/goal3_z",goal3_z);

    ros::param::get("control_start/goal4_x",goal4_x);
    ros::param::get("control_start/goal4_y",goal4_y);
    ros::param::get("control_start/goal4_z",goal4_z);

    ros::param::get("control_start/goal5_x",goal5_x);
    ros::param::get("control_start/goal5_y",goal5_y);
    ros::param::get("control_start/goal5_z",goal5_z);

    ros::param::get("control_start/goal6_x",goal6_x);
    ros::param::get("control_start/goal6_y",goal6_y);
    ros::param::get("control_start/goal6_z",goal6_z);
    
    ROS_INFO("get goal param succeefully!");    //获取目标点参数
    ROS_INFO("goal_num is %d ",goal_num);    //获取目标点参数
   

}


void init_goallist()        //将目标点存入二维数组goal_list
{
    goal_list[0][0]=goal0_x;
    goal_list[0][1]=goal0_y;
    goal_list[0][2]=goal0_z;

    goal_list[1][0]=goal1_x;
    goal_list[1][1]=goal1_y;
    goal_list[1][2]=goal1_z;

    goal_list[2][0]=goal2_x;
    goal_list[2][1]=goal2_y;
    goal_list[2][2]=goal2_z;

    goal_list[3][0]=goal3_x;
    goal_list[3][1]=goal3_y;
    goal_list[3][2]=goal3_z;

    goal_list[4][0]=goal4_x;
    goal_list[4][1]=goal4_y;
    goal_list[4][2]=goal4_z;

    goal_list[5][0]=goal5_x;
    goal_list[5][1]=goal5_y;
    goal_list[5][2]=goal5_z;

    goal_list[6][0]=goal6_x;
    goal_list[6][1]=goal6_y;
    goal_list[6][2]=goal6_z;

    ROS_INFO("goal_list init succeefully");    //获取目标点参数

}





void mic_awake_cb(const std_msgs::Int8::ConstPtr& a_p)
{
    awake_flag = a_p->data;             //传递a_p值
    ROS_INFO("awake flag is set");
}













int main(int argc, char** argv)
{
	ros::init(argc, argv, "control_start");
	ros::NodeHandle n;


    ros::Subscriber awake_sub = n.subscribe<std_msgs::Int8>("awake_flag",10,mic_awake_cb);

    
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

	ROS_INFO("Sending goal -- %d",i);
        ac.sendGoal(goal);
        

        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The base reach goal -- %d",i);
        else
            ROS_INFO("The base failed to move for some reason");
    }
    ROS_INFO("Navgation done..");
    
    ros::spin();

    return 0;
}


