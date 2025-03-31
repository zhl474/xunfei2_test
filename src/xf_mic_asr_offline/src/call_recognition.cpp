/**************************************************************************
作者：caidx1
功能：录音调用控制器，包含休眠功能
**************************************************************************/
#include <user_interface.h>
#include <string>
#include <locale>
#include <codecvt>
#include <ctime>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <xf_mic_asr_offline/Get_Offline_Result_srv.h>
#include <xf_mic_asr_offline/Pcm_Msg.h>
#include <xf_mic_asr_offline/Start_Record_srv.h>
#include <xf_mic_asr_offline/Set_Awake_Word_srv.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <sys/stat.h>
#include <iostream>
#include <geometry_msgs/Twist.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


using namespace std;
int awake_flag = 0;    //唤醒标志位
int confidence_threshold ;
int seconds_per_order ;

/**************************************************************************
函数功能：唤醒标志sub回调函数
入口参数：唤醒标志位awake_flag_msg  voice_control.cpp
返回  值：无
**************************************************************************/
void awake_flag_Callback(std_msgs::Int8 msg)
{
	awake_flag = msg.data;
	//printf("awake_flag=%d\n",awake_flag);

	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

	struct goal_pose
	{
    		double pose[3];
    		double orientation[4];
	};

	struct goal_pose set_point[]=
	{
	 //这里可以添加自己的多个导航目标点
     		{2.181, -0.116, 0.000,0.000, 0.000, 0.009, 1.000},
                {2.170, -0.159, 0.000,0.000, 0.000, -0.178, 0.984},
                {2.322, -0.257, 0.000,0.000, 0.000, -0.584, 0.812},
                {2.450, -1.031, 0.000,0.000, 0.000, 0.000, 1.000},
   //  {2.263, -0.095, 0.000,0.000, 0.000, -0.019, 1.000},
   //  {2.343, -0.130, 0.000,0.000, 0.000, -0.485, 0.875},
   //  {2.455, -0.950, 0.000,0.000, 0.000, 0.014, 1.000},
   //  {4.603, -0.457, 0.000,0.000, 0.000, -0.000, 1.000},
    




                {5.031, -3.107, 0.000 ,  0.000, 0.000, 0.360, 0.933}, 
                {5.122, -4.002, 0.000,0.000, 0.000, -0.004, 1.000},   		
                {5.052, -5.058, 0.000,0.000, 0.000, -0.361, 0.933},


                {3.087, -4.590, 0.000,0.000, 0.000, -0.001, 1.000}, 
                {3.033, -5.124, 0.000,0.000, 0.000, -0.692, 0.722},  
                {2.885, -4.544, 0.000,0.000, 0.000, 1.000, 0.002},
    // {2.397, -1.967, 0.000,0.000, 0.000, 0.000, 1.000}, 	
                {1.141, -5.105, 0.000,0.000, 0.000, -0.757, 0.653}, 
    // {1.230, -1.793, 0.000,0.000, 0.000, -0.126, 0.992},

                {0.13, -1.125, 0.000,0.000, 0.000, 0.000, 1.000},
    // {-0.103, -1.172, 0.000,0.000, 0.000, 0.007, 1.000 },  
	};

	int size =sizeof(set_point)/sizeof(goal_pose);
	
    	MoveBaseClient ac("move_base",true);
 	 if(msg.data)
	{
    		

    		while (!ac.waitForServer(ros::Duration(1.0)))
    		{
       			ROS_INFO("waiting for the move_base action server to come up");
    		}
    
    		for(int i=0;i<size;i++)  
    		{
        		struct goal_pose point=set_point[i];
        		move_base_msgs::MoveBaseGoal goal;

        //goal.target_pose.header.frame_id="base_link"; //局部定位
	    		goal.target_pose.header.frame_id="map";			//全局定位
        		goal.target_pose.header.stamp=ros::Time::now();

        		goal.target_pose.pose.position.x=point.pose[0];
        		goal.target_pose.pose.position.y=point.pose[1];
        		goal.target_pose.pose.position.z=point.pose[2];
        		goal.target_pose.pose.orientation.x=point.orientation[0];
        		goal.target_pose.pose.orientation.y=point.orientation[1];
        		goal.target_pose.pose.orientation.z=point.orientation[2];        
        		goal.target_pose.pose.orientation.w=point.orientation[3];
 		ROS_INFO("sending goal ...");
        		ac.sendGoal(goal);
        		ROS_INFO("goal executing ...");
        		ac.waitForResult();
        		if(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
        		{
            			("the base move goal execute succsess ...");
           			 continue;   //如果到达目标点想继续循环运动，则设为continue
           // break;        //如果到达目标点之后，想停止，则break     
        		}	 
        		else{
            			ROS_INFO("the base move goal execute failed ...");
            			break;
        			}
    		}

	}

	
}


/**************************************************************************
函数功能：主函数
入口参数：无
返回  值：无
**************************************************************************/
int main(int argc, char *argv[])
{
	//int temp=0;
	int recognize_fail_count = 0;    //被动休眠相关变量
	int recognize_fail_count_threshold = 5;    //识别失败次数阈值
	
	string str1 = "ok";				//语音识别相关字符串
	string str2 = "fail";				//语音识别相关字符串
	string str3 = "小车休眠";				//语音识别相关字符串
	string str4 = "失败5次";				//语音识别相关字符串
	string str5 = "失败10次";				//语音识别相关字符串

	ros::init(argc, argv, "call_rec");    //初始化ROS节点
	ros::init(argc, argv, "simple_navigation_goals");
	ros::NodeHandle nh;    //创建句柄
 
	/***离线命令词识别服务客服端创建***/
	ros::ServiceClient get_offline_recognise_result_client = 
	nh.serviceClient<xf_mic_asr_offline::Get_Offline_Result_srv>("xf_asr_offline_node/get_offline_recognise_result_srv");
	
	/***修改唤醒词服务客户端创建***/
	//ros::ServiceClient Set_Awake_Word_client =
	//nh.serviceClient<xf_mic_asr_offline::Set_Awake_Word_srv>("xf_asr_offline_node/set_awake_word_srv");

	/***唤醒标志位话题订阅者创建***/
	ros::Subscriber awake_flag_sub = nh.subscribe("awake_flag", 1, awake_flag_Callback);

	/***唤醒标志位话题发布者创建***/
	ros::Publisher awake_flag_pub = nh.advertise<std_msgs::Int8>("awake_flag", 1);

	/***离线命令词识别结果话题发布者创建***/
	ros::Publisher control = nh.advertise<std_msgs::String>("voice_words", 10);

	ros::Rate loop_rate(10);    //循环频率10Hz


	nh.param("/confidence", confidence_threshold, 18);
	nh.param("/seconds_per_order", seconds_per_order, 3);

  /***请求修改唤醒词服务***/
	//xf_mic_asr_offline::Set_Awake_Word_srv SetAwakeWord_srv;
	//SetAwakeWord_srv.request.awake_word="小车小车";
	//Set_Awake_Word_client.call(SetAwakeWord_srv);
  /***等待服务应答***/
	//std::cout << "Set_Awake_Word: " << SetAwakeWord_srv.response.result << endl;

  /***离线命令词识别服务参数设置***/
	xf_mic_asr_offline::Get_Offline_Result_srv GetOfflineResult_srv;
	GetOfflineResult_srv.request.offline_recognise_start = 1;
	GetOfflineResult_srv.request.confidence_threshold = confidence_threshold;
	GetOfflineResult_srv.request.time_per_order = seconds_per_order;


	while(ros::ok())
	{
		if(awake_flag)    //判断休眠状态还是唤醒状态
		{
			if(get_offline_recognise_result_client.call(GetOfflineResult_srv))    //请求离线命令词识别服务并返回应答为调用成功
			{
				//ROS_INFO("succeed to call service \"get_offline_recognise_result_srv\"!");    //打印识别结果、置信度、识别命令词等信息
				//std::cout << "result: " << GetOfflineResult_srv.response.result << endl;
				//std::cout << "fail reason: " << GetOfflineResult_srv.response.fail_reason << endl;
				//std::cout << "text: " << GetOfflineResult_srv.response.text << endl;

				if(str3 == GetOfflineResult_srv.response.text)    //主动休眠
					awake_flag=0;
				else if(str1 == GetOfflineResult_srv.response.result)    //清零被动休眠相关变量
					recognize_fail_count = 0;
				else if(str2 == GetOfflineResult_srv.response.result)    //记录识别失败次数
				{
					recognize_fail_count++;
					printf("recognize_fail_count=%d\n",recognize_fail_count);
					if(recognize_fail_count==5)    //连续识别失败5次，用户界面显示提醒信息
					{
						std_msgs::String count_msg;
						count_msg.data = str4;
						control.publish(count_msg);
						
					}
					else if(recognize_fail_count==10)    //连续识别失败10次，用户界面显示提醒信息
					{
						std_msgs::String count_msg;
						count_msg.data = str5;
						control.publish(count_msg);
						
					}
				}
			}else        //请求离线命令词识别服务并返回应答为调用失败
			{
				ROS_INFO("failed to call service \"get_offline_recognise_result_srv\"!");
				//awake_flag=0;
				break;
			}

			if(recognize_fail_count >= recognize_fail_count_threshold)    //被动休眠
			{
				awake_flag=0;
				std_msgs::String controloff_msg;
				controloff_msg.data = str3;
				control.publish(controloff_msg);
				recognize_fail_count = 0;

        std_msgs::Int8 awake_flag_msg;
        awake_flag_msg.data = 0;
        awake_flag_pub.publish(awake_flag_msg);
			}
			
		}		
		//printf("awake_flag=%d\n",awake_flag);
		//printf("-----confidence_threshold =%d\n",confidence_threshold);
		//printf("seconds_per_order =%d\n",seconds_per_order); 
		ros::spinOnce();    
		loop_rate.sleep();    //10Hz循环
		
	}

}
