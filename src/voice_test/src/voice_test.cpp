#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <iostream>

using namespace std;

ros::Publisher voice_mode_pub;
std_msgs::Int8 voice_mode;


/**************************************************************************
函数功能：离线命令词识别结果sub回调函数
入口参数：命令词字符串  voice_control.cpp等
返回  值：无
**************************************************************************/
void voice_words_callback(const std_msgs::String& msg)
{
	/***语音指令***/
	string str = msg.data.c_str();    //取传入数据
	string str0 = "小车休眠";
	string str1 = "小车出发";
	string str2 = "小车前进";
	string str3 = "小车后退"; 
	string str4 = "小车左转";
	string str5 = "小车右转";
	string str6 = "小车左移";
	string str7 = "小车右移";
	/***其它指令***/
	string str10 = "小车唤醒";
	string str11 = "失败5次";
	string str12 = "失败10次";
	
	
	
	/***********************************/
	/***********************************/
	/***********************************/
	string str13 = "任务领取";
	/***********************************
	指令：大车启动
	动作：迅飞比赛要求
	***********************************/
	if(str == str13){
		voice_mode.data = 20;
		voice_mode_pub.publish(voice_mode);
		system("play ~/ucar_ws/src/mp3/voice_1.mp3");
	}
	/***********************************/
	/***********************************/
	/***********************************/
	

	/***********************************
	指令：小车休眠
	动作：底盘运动控制器失能，发布速度空指令，唤醒标志位置零
	***********************************/
	if(str == str0){
		voice_mode.data = 0;
		voice_mode_pub.publish(voice_mode);
		cout<<"小车休眠，等待下一次唤醒"<<endl;
	}
	/***********************************
	指令：小车出发
	动作：底盘运动控制器失能，发布速度空指令
	***********************************/
	else if(str == str1){
		voice_mode.data = 1;
		voice_mode_pub.publish(voice_mode);
		system("play ~/ucar_ws/src/mp3/voice_1.mp3");
		//cout<<"好的：小车出发"<<endl;
	}
	/***********************************
	指令：小车前进
	动作：底盘运动控制器使能，发布速度指令
	***********************************/
	else if(str == str2){
		voice_mode.data = 2;
		voice_mode_pub.publish(voice_mode);
		system("play ~/ucar_ws/src/mp3/voice_2.mp3");
		//cout<<"好的：小车前进"<<endl;
	}
	/***********************************
	指令：小车后退
	动作：底盘运动控制器使能，发布速度指令
	***********************************/
	else if(str == str3){
		voice_mode.data = 3;
		voice_mode_pub.publish(voice_mode);
		system("play ~/ucar_ws/src/mp3/voice_3.mp3");
		//cout<<"好的：小车后退"<<endl;
	}
	/***********************************
	指令：小车左转
	动作：底盘运动控制器使能，发布速度指令
	***********************************/
	else if(str == str4){
		voice_mode.data = 4;
		voice_mode_pub.publish(voice_mode);
		system("play ~/ucar_ws/src/mp3/voice_4.mp3");
		//cout<<"好的：小车左转"<<endl;
	}
	/***********************************
	指令：小车右转
	动作：底盘运动控制器使能，发布速度指令
	***********************************/
	else if(str == str5){
		voice_mode.data = 5;
		voice_mode_pub.publish(voice_mode);
		system("play ~/ucar_ws/src/mp3/voice_5.mp3");
		//cout<<"好的：小车右转"<<endl;
	}
	/***********************************
	指令：小车左移
	动作：底盘运动控制器使能，发布速度指令
	***********************************/
	else if(str == str6){
		voice_mode.data = 6;
		voice_mode_pub.publish(voice_mode);
		system("play ~/ucar_ws/src/mp3/voice_6.mp3");
		//cout<<"好的：小车左移"<<endl;
	}
	/***********************************
	指令：小车右移
	动作：底盘运动控制器使能，发布速度指令
	***********************************/
	else if(str == str7){
		voice_mode.data = 7;
		voice_mode_pub.publish(voice_mode);
		system("play ~/ucar_ws/src/mp3/voice_7.mp3");
		//cout<<"好的：小车右移"<<endl;
	}


	/***********************************
	辅助指令：失败5次
	动作：用户界面打印提醒
	***********************************/
	else if(str == str11){
		cout<<"您已经连续【输入空指令or识别失败】5次，累计达15次自动进入休眠，输入有效指令后计数清零"<<endl;
	}
	/***********************************
	辅助指令：失败10次
	动作：用户界面打印提醒
	***********************************/
	else if(str == str12){
		cout<<"您已经连续【输入空指令or识别失败】10次，累计达15次自动进入休眠，输入有效指令后计数清零"<<endl;
	}
	/***********************************
	辅助指令：小车唤醒
	动作：用户界面打印提醒
	***********************************/
	else if(str == str10){
		//cout<<"小车已被唤醒，请说语音指令"<<endl;
		voice_mode.data = 10;
		voice_mode_pub.publish(voice_mode);
		system("play ~/ucar_ws/src/mp3/voice_10.mp3");
	}
	
}






int main(int argc, char** argv)
{
	ros::init(argc, argv, "voic_test");
	ros::NodeHandle n;

	voice_mode_pub = n.advertise<std_msgs::Int8>("/voice_mode", 1000);
	/***创建离线命令词识别结果话题订阅者***/
	ros::Subscriber voice_words_sub = n.subscribe("voice_words",10,voice_words_callback);

	ros::Rate loop_rate(100);



	while(ros::ok())
	{
	  
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
