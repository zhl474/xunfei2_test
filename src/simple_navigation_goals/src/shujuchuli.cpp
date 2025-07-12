#include <ros/ros.h>
#include <ros/master.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <std_msgs/Int8.h>
//#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Config.h>
#include <tf/transform_listener.h>
using namespace std;



char *voice[4][4]={{"aplay ~/ucar_car/src/simple_navigation_goals/voice/B_corn.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/B_cucumber.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/B_rice.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/B_wheat.wav"},
                    {"aplay ~/ucar_car/src/simple_navigation_goals/voice/C_corn.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/C_cucumber.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/C_rice.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/C_wheat.wav"},
                    {"aplay ~/ucar_car/src/simple_navigation_goals/voice/D_corn.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/D_cucumber.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/D_rice.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/D_wheat.wav"},
                    {"aplay ~/ucar_car/src/simple_navigation_goals/voice/E_corn.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/E_cucumber.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/E_rice.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/E_wheat.wav"}};
char *num[7] = {"aplay ~/ucar_car/src/simple_navigation_goals/voice/1.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/2.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/3.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/4.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/5.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/6.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/7.wav"};
int count[4]={0};
int area_sort[4] = {4,4,4,4};   //各房间种物类型BCDE 4 is invalid data
int room_index=0;
int room_change_flag = 0;
int past_room_index = 0;
int detect_fail = 0;
void area_msg_cb(const std_msgs::Int8::ConstPtr& r_p )
{
    
    int result = r_p->data;
    //detect_flag = 1;
    if (result == 10){//detect fail
        //success_detect = 0;
        ROS_INFO("detect result  : %d !",result); 
        room_change_flag = 0;
        detect_fail++;
        if(detect_fail==2)
        {
            room_index++;
            detect_fail = 0;
        }
    }
    else//success detect
    {
        detect_fail = 0;
        ROS_INFO("detect result : %d !",result);
        //success_detect = 1;
        area_sort[room_index]=result;
        past_room_index = room_index;
        room_index++;
        if(room_index==3)
        {
            room_change_flag = 1;
            if(area_sort[0]!=4 && area_sort[1]!=4 && area_sort[2]!=4)
            {
                room_index = 4;
                room_change_flag = 2;
            }
        }
        if(room_index == 4)
        {
            int sum = 0;
            if(area_sort[0]==4)
            {
                sum = area_sort[1]+area_sort[3]+area_sort[2];            //三个房间总共有三种累加和，分别为3，4,5，6
                if(sum == 3){
                    area_sort[0]=3;
                } 
                else if(sum == 4){
                    area_sort[0]=2;
                }
                else if(sum == 5){
                    area_sort[0]=1;
                }
                else {
                    area_sort[0]=0;
                }
            }
            else if(area_sort[1]==4)
            {
                sum = area_sort[0]+area_sort[3]+area_sort[2];            //三个房间总共有三种累加和，分别为3，4,5，6
                if(sum == 3){
                    area_sort[1]=3;
                } 
                else if(sum == 4){
                    area_sort[1]=2;
                }
                else if(sum == 5){
                    area_sort[1]=1;
                }
                else {
                    area_sort[1]=0;
                }
            }
            else if(area_sort[2]==4)
            {
                sum = area_sort[0]+area_sort[1]+area_sort[3];            //三个房间总共有三种累加和，分别为3，4,5，6
                if(sum == 3){
                    area_sort[2]=3;
                } 
                else if(sum == 4){
                    area_sort[2]=2;
                }
                else if(sum == 5){
                    area_sort[2]=1;
                }
                else {
                    area_sort[2]=0;
                }
            }
            else if(area_sort[3]==4)
            {
                sum = area_sort[0]+area_sort[1]+area_sort[2];            //三个房间总共有三种累加和，分别为3，4,5，6
                if(sum == 3){
                    area_sort[3]=3;
                } 
                else if(sum == 4){
                    area_sort[3]=2;
                }
                else if(sum == 5){
                    area_sort[3]=1;
                }
                else {
                    area_sort[3]=0;
                }
            }
            ROS_INFO("%d %d %d %d",area_sort[0],area_sort[1],area_sort[2],area_sort[3]);
            if((room_index - past_room_index)!=1)
            {
                room_change_flag = 2;
                return ;
            }
        }
        // std_msgs::Int8 changeroom;
        // changeroom.data=1;
        // room_change.publish(changeroom);
        room_change_flag = 1;
    }
    ROS_INFO("shujuchuli room_index : %d !",room_index);
    ROS_INFO("shujuchuli detect_fail : %d !",detect_fail);
}

void data_msg_cb(const std_msgs::Int8::ConstPtr& r_p)
{
    if(int result = r_p->data)
    {
        room_change_flag = 100; 
    }

}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "shujuchuli");
    ros::NodeHandle n;
    ros::Subscriber room_sub = n.subscribe<std_msgs::Int8>("/area_msg",1,area_msg_cb); //房间类型订阅
    ros::Publisher room_change = n.advertise<std_msgs::Int8>("/room_change",1);//publisher
    ros::Subscriber data_request_sub = n.subscribe<std_msgs::Int8>("/data_request",1,data_msg_cb); 
    //ros::Subscriber room_sub_f = n.subscribe<std_msgs::String>("/area_msgf",1,area_msg_cb_f);
    // ros::spin();
    while(1)
    {
        if(room_change_flag)//change room
        {
            std_msgs::Int8 changeroom;
            if(room_change_flag == 2)
            {
                changeroom.data=2;
                room_change.publish(changeroom);
                room_change_flag = 0;
            } 
            else if(room_change_flag == 1)
            {
                changeroom.data=1;
                room_change.publish(changeroom);
                room_change_flag = 0;
            }
            else//通过room_change_flag = 100 去给导航结点发送BCDE区域信息
            {
                // changeroom.data = area_sort[0]*32+area_sort[1]*16+area_sort[2]*4+area_sort[3];
                // room_change.publish(changeroom);
                // room_change_flag = 0;
                ros::Publisher data_send = n.advertise<std_msgs::String>("/data_send",1);//publisher
                std_msgs::String msg;
                msg.data = std::to_string(area_sort[0])+std::to_string(area_sort[1])+std::to_string(area_sort[2])+std::to_string(area_sort[3]);
                data_send.publish(msg);
                room_change_flag = 0;
            }
            ros::spinOnce();
        }
        ros::spinOnce();
    }
}