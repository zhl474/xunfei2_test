/*
 * client.cpp
 */
#include "user_interface.h"
#include <string>
#include <locale>
#include <codecvt>
#include <ctime>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <xf_mic_asr_offline/Get_Offline_Result_srv.h>
#include <xf_mic_asr_offline/Set_Major_Mic_srv.h>
#include <xf_mic_asr_offline/Set_Led_On_srv.h>
#include <xf_mic_asr_offline/Get_Major_Mic_srv.h>
#include <xf_mic_asr_offline/Pcm_Msg.h>
#include <xf_mic_asr_offline/Start_Record_srv.h>
#include <xf_mic_asr_offline/Set_Awake_Word_srv.h>
#include <xf_mic_asr_offline/Get_Awake_Angle_srv.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <sys/stat.h>
#include <iostream>

using namespace std;

int awake_angle = -1;


void awake_angle_Callback(std_msgs::Int32 msg)
{
    awake_angle = msg.data;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "client_node");
    ros::NodeHandle nh;

    /*离线命令词识别*/
    ros::ServiceClient get_offline_recognise_result_client = 
    nh.serviceClient<xf_mic_asr_offline::Get_Offline_Result_srv>("xf_asr_offline_node/get_offline_recognise_result_srv");
    /*设置主麦克风*/
    ros::ServiceClient Set_Major_Mic_client =
    nh.serviceClient<xf_mic_asr_offline::Set_Major_Mic_srv>("xf_asr_offline_node/set_major_mic_srv");
    /*获取主麦克风*/
    ros::ServiceClient Get_Major_Mic_client =
    nh.serviceClient<xf_mic_asr_offline::Get_Major_Mic_srv>("xf_asr_offline_node/get_major_mic_srv");
    /*修改唤醒词*/
    ros::ServiceClient Set_Awake_Word_client =
    nh.serviceClient<xf_mic_asr_offline::Set_Awake_Word_srv>("xf_asr_offline_node/set_awake_word_srv");

    xf_mic_asr_offline::Set_Awake_Word_srv SetAwakeWord_srv;
    SetAwakeWord_srv.request.awake_word = "小飞小飞快速出发执行任务"; // 设置唤醒词
    if(Set_Awake_Word_client.call(SetAwakeWord_srv))
    {
        ROS_INFO("succeed to call service \"set_awake_word_srv\"!");
        std::cout << "result: " << SetAwakeWord_srv.response.result << endl;
        std::cout << "fail reason: " << SetAwakeWord_srv.response.fail_reason << endl;
    }
    else
    {
        ROS_INFO("failed to call service \"set_awake_word_srv\"!");
    }
    /*订阅唤醒角度*/
    ros::Subscriber awake_angle_sub = nh.subscribe("/mic/awake/angle", 1, awake_angle_Callback);
    
    xf_mic_asr_offline::Get_Offline_Result_srv GetOfflineResult_srv;
    xf_mic_asr_offline::Set_Major_Mic_srv SetMajorMic_srv;
    xf_mic_asr_offline::Get_Major_Mic_srv GetMajorMic_srv;
 
    std::string word = "";
    int recognize_fail_count;
    int recognize_fail_count_threshold;
    int confidence_threshold;
    int time_per_order;

    //SetMajorMic_srv.request.mic_id = 2;
    //call set_major_mic_srv
    //if(Set_Major_Mic_client.call(SetMajorMic_srv))
    //{
        //ROS_INFO("succeed to call service \"set_major_mic_srv\"!");
        //std::cout << "result: " << SetMajorMic_srv.response.result << endl;
        //std::cout << "fail reason: " << SetMajorMic_srv.response.fail_reason << endl;
    //}
   // else
    //{
        //ROS_INFO("failed to call service \"set_major_mic_srv\"!");     
    //}

    sleep(2);

    GetOfflineResult_srv.request.offline_recognise_start = 1;
    GetOfflineResult_srv.request.confidence_threshold = 20;
    GetOfflineResult_srv.request.time_per_order = 3;
    //call get_offline_recognise_result_srv
    if(get_offline_recognise_result_client.call(GetOfflineResult_srv))
    {
        ROS_INFO("succeed to call service \"get_offline_recognise_result_srv\"!");
        std::cout << "result: " << GetOfflineResult_srv.response.result << endl;
        std::cout << "fail reason: " << GetOfflineResult_srv.response.fail_reason << endl;
        std::cout << "text: " << GetOfflineResult_srv.response.text << endl;
        system("play -t raw -r 16k -e signed -b 16 -c 1 '/home/nie/mmic_ws/src/SR2.1.2-HR1.1.3/vvui_ros-master/xf_mic_asr_offline/audio/vvui_deno.pcm' -q --no-show-progress");
    }
    else
    {
        ROS_INFO("failed to call service \"get_offline_recognise_result_srv\"!");
    }

    return 0;
}
