#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include "actionlib/client/simple_goal_state.h"
#include "actionlib/client/simple_client_goal_state.h"
#include "actionlib/client/terminal_state.h"
#include <ros/time.h>
#include <serial_port/rcmsg.h>
#include <geometry_msgs/Twist.h>
#include <ucar_controller/SetLEDMode.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/BatteryState.h> 
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <stdio.h>

using namespace std;
serial_port::rcmsg RC_Info;
geometry_msgs::Twist vel_cmd;
geometry_msgs::PoseWithCovarianceStamped init_pose;
std_msgs::Int8 radio_mode;
ucar_controller::SetLEDMode rgb_set;
std_srvs::Empty srv;
sensor_msgs::BatteryState bat;
float bat_limit=20;    //
char voiceVel_flag = 0; 
char arry_flag=0;
char arry_time=61;



/* 重命名类定义，可通过Client来定义这个类（原型）actionlib::SimpleActionClient< ActionSpec > Class Template 
   < ActionSpec >是通过.action文件产生的msgs文件包含的，这里<  >表示C艹的继承
   move_base_msgs::MoveBaseAction对应的是move_base的.action文件产生的msg文件，roscd move_base_msgs可查看 */
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;
typedef actionlib::SimpleClientGoalState State;
int second = 0;
State::StateEnum state;

 

/* https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1SimpleActionClient.html */
/* http://wiki.ros.org/actionlib */
/* https://www.guyuehome.com/908 */
/* 同时佩服我的才华 */
geometry_msgs::Pose pose_list[6];
geometry_msgs::Point current_point;

/* 关于geometry_msgs::PoseStamped的一些定义，作用就是填充geometry_msgs::PoseStamped */
geometry_msgs::Point setPoint(double _x, double _y, double _z);
geometry_msgs::Quaternion setQuaternion(double _angleRan);
geometry_msgs::Point setPoint(double _x, double _y, double _z)
{
  geometry_msgs::Point m_point;
  m_point.x = _x;
  m_point.y = _y;
  m_point.z = _z;
  return m_point;
}

geometry_msgs::Quaternion setQuaternion(double _angleRan)
{
  geometry_msgs::Quaternion m_quaternion;
  m_quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, _angleRan);//这是一个欧拉角转四元素，车子只有Yaw这个自由度
  return m_quaternion;
}



void init_goalList()
{
  pose_list[0].position = setPoint(-3.24275684357, 0.142583549023, 0);
  pose_list[0].orientation = setQuaternion( 3.1415 );
  
  pose_list[1].position = setPoint(-0.2, -5, 0);
  pose_list[1].orientation = setQuaternion( 3.1415 );

  pose_list[2].position = setPoint(-0.2, -5, 0);
  pose_list[2].orientation = setQuaternion( 3.1415 );

  pose_list[3].position = setPoint(-0.2, -5, 0);
  pose_list[3].orientation = setQuaternion( 3.1415 );

  pose_list[4].position = setPoint(-0.2, -5, 0);
  pose_list[4].orientation = setQuaternion( 3.1415 );

  pose_list[5].position = setPoint(-0.2, -5, 0);
  pose_list[5].orientation = setQuaternion( 3.1415 );
}

/* sendGoal回调函数 */
void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResult::ConstPtr& result)

{
  if(state==State::PREEMPTED){//表示取消了目标
    ROS_INFO("goal cancelled");
  }else{
    ROS_INFO("Goal done");
  }
}

void activeCb()
{
  ROS_INFO("Goal Received");
}

void feedbackCb(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
{
  ROS_INFO("Got base_position of Feedback");
  current_point.x = feedback->base_position.pose.position.x;//movebase通过action传递回来的参数，这个没看到官网说明
  current_point.y = feedback->base_position.pose.position.y;//大佬看源码晓得的https://blog.csdn.net/tiancailx/article/details/86513304
  current_point.z = feedback->base_position.pose.position.z;//嗯，roscd move_base_msgs，可以通过.action和.msg文件来观察的
}





void radioCallback(const serial_port::rcmsg::ConstPtr& msg)
{
    //ROS_INFO("RC recived");
    //将遥控器值转入需要变量
    RC_Info.rssi = msg->rssi;
    RC_Info.ch1 = msg->ch1;
    RC_Info.ch2 = msg->ch2;
    RC_Info.ch3 = msg->ch3;
    RC_Info.ch4 = msg->ch4;
                
    RC_Info.swa = msg->swa;
    RC_Info.swb = msg->swb;
    RC_Info.swc = msg->swc;//三段开关
    RC_Info.swd = msg->swd;
				
    RC_Info.vra = msg->vra;
    RC_Info.vrb = msg->vrb;
}



void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
    bat.percentage=msg->percentage;
    //ROS_INFO("BatteryState recived");
    //ROS_INFO("Battery%f",bat.percentage);
}



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
	string str1 = "小车停";
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

	/***********************************
	指令：小车休眠
	动作：底盘运动控制器失能，发布速度空指令，唤醒标志位置零
	***********************************/
	if(str == str0){
		voiceVel_flag = 0;
		//cout<<"小车休眠，等待下一次唤醒"<<endl;
	}
	/***********************************
	指令：小车停
	动作：底盘运动控制器失能，发布速度空指令
	***********************************/
	else if(str == str1){
		voiceVel_flag = 1;
		system("play ~/ucar_ws/src/mp3/voice_1.mp3");
		//cout<<"好的：小车出发"<<endl;
	}
	/***********************************
	指令：小车前进
	动作：底盘运动控制器使能，发布速度指令
	***********************************/
	else if(str == str2){
		voiceVel_flag = 2;
		system("play ~/ucar_ws/src/mp3/voice_2.mp3");
		//cout<<"好的：小车前进"<<endl;
	}
	/***********************************
	指令：小车后退
	动作：底盘运动控制器使能，发布速度指令
	***********************************/
	else if(str == str3){
		voiceVel_flag = 3;
		system("play ~/ucar_ws/src/mp3/voice_3.mp3");
		//cout<<"好的：小车后退"<<endl;
	}
	/***********************************
	指令：小车左转
	动作：底盘运动控制器使能，发布速度指令
	***********************************/
	else if(str == str4){
	  voiceVel_flag = 4;
		system("play ~/ucar_ws/src/mp3/voice_4.mp3");
		//cout<<"好的：小车左转"<<endl;
	}
	/***********************************
	指令：小车右转
	动作：底盘运动控制器使能，发布速度指令
	***********************************/
	else if(str == str5){
		voiceVel_flag = 5;
		system("play ~/ucar_ws/src/mp3/voice_5.mp3");
		//cout<<"好的：小车右转"<<endl;
	}
	/***********************************
	指令：小车左移
	动作：底盘运动控制器使能，发布速度指令
	***********************************/
	else if(str == str6){
		voiceVel_flag = 6;
		system("play ~/ucar_ws/src/mp3/voice_6.mp3");
		//cout<<"好的：小车左移"<<endl;
	}
	/***********************************
	指令：小车右移
	动作：底盘运动控制器使能，发布速度指令
	***********************************/
	else if(str == str7){
		voiceVel_flag = 7;
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
		system("play ~/ucar_ws/src/mp3/voice_10.mp3");
	}
}








int main(int argc, char** argv)
{
	uint16_t RC_Data[10] = {0};
	init_goalList();

	ros::init(argc, argv, "RC");
	ros::NodeHandle n;

	ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Publisher radio_mode_pub = n.advertise<std_msgs::Int8>("/radio_mode", 1000);
	ros::Publisher init_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000);

	ros::ServiceClient led_client = n.serviceClient<ucar_controller::SetLEDMode>("/set_led_light");
	ros::ServiceClient array_client = n.serviceClient<std_srvs::Empty>("/global_localization");
	ros::ServiceClient mapclean_client = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

	ros::Subscriber radio_sub = n.subscribe("/RC",10,radioCallback);
	ros::Subscriber bat_sub = n.subscribe("/battery_state",10,batteryCallback);
	/***创建离线命令词识别结果话题订阅者***/
	ros::Subscriber voice_words_sub = n.subscribe("voice_words",10,voice_words_callback);

	ros::Rate loop_rate(1000);

	/* SimpleActionClient (ros::NodeHandle &n, const std::string &name, bool spin_thread=true)为构造函数，在创建对象时会自动调用
			move_base表示action服务器的名字，服务器是在movebase源码里定义好了的，roscd move_base_msgs
			true表示调用ros::spin()，true -> don't need ros::spin()，如果为true，则启动一个线程来运行ros::spin()，ros::spin()用来开启订阅 */
	Client ac("move_base", true);
	/* waitForServer等待ActionServer连接到此客户端，这里设置为60s 
	if (!ac.waitForServer(ros::Duration(60)))
	{
			ROS_INFO("Can't connected to move base server");
			return 1;
	}*/
	/* 第一个参数代表目标动作，
			第二个参数指定目标goal被目标server成功完成/取消的回调函数，默认 SimpleDoneCallback(), 
			第三个参数指定目标goal被目标server成功接收的回调函数，默认 SimpleActiveCallback(), 
			第四个参数表示目标Client接收到目标server返回数据回调函数，默认 SimpleFeedbackCallback() 
	*/
	//ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);



	while(ros::ok())
	{   
		/* 模式0，红色，停止模式
		 * 模式1，黄色，遥控模式
		 * 模式2，蓝色，语音模式
		 * 模式3，绿色，初始模式
		 * 模式4，紫色，导航模式
		 * 故障模式，遥控器失联红闪,电池没电黄闪
		*/
		if(RC_Info.rssi>0 && bat.percentage>bat_limit){
			if(RC_Info.swd>0 && RC_Info.swc>0){//模式0，红色
				radio_mode.data=0;

				rgb_set.request.mode_type=0;	//0常亮；1呼吸；2闪烁
				rgb_set.request.frequency=0;	//模式频率（常亮模式自动忽略）
				rgb_set.request.red_value=255;	//红色光亮度范围（0-255），在线颜色选择器http://tools.jb51.net/static/colorpicker/
				rgb_set.request.green_value=0;	//绿色光亮度范围（0-255）
				rgb_set.request.blue_value=0;	//蓝色光亮度范围（0-255）

				vel_cmd.linear.x = 0;//纵向运动
				vel_cmd.linear.y = 0;//横向运动，右手坐标系
				vel_cmd.angular.z = 0;//自旋运动，右手坐标系
				vel_pub.publish(vel_cmd);
			}else if(RC_Info.swd>0 && RC_Info.swc==0){//模式1，黄色
				radio_mode.data=1;

				rgb_set.request.mode_type=0;	//0常亮；1呼吸；2闪烁
				rgb_set.request.frequency=0;	//模式频率（常亮模式自动忽略）
				rgb_set.request.red_value=255;	//红色光亮度范围（0-255），在线颜色选择器http://tools.jb51.net/static/colorpicker/
				rgb_set.request.green_value=255;	//绿色光亮度范围（0-255）
				rgb_set.request.blue_value=0;	//蓝色光亮度范围（0-255）

				vel_cmd.linear.x = RC_Info.ch2*0.005;//纵向运动
				vel_cmd.linear.y = -RC_Info.ch1*0.005;//横向运动，右手坐标系
				vel_cmd.angular.z = -RC_Info.ch4*0.01;//自旋运动，右手坐标系
				vel_pub.publish(vel_cmd);
			}else if(RC_Info.swd>0 && RC_Info.swc<0){//模式2，蓝色
				if(radio_mode.data!=2){
					voiceVel_flag=0;
				}
				radio_mode.data=2;
				
				rgb_set.request.mode_type=0;	//0常亮；1呼吸；2闪烁
				rgb_set.request.frequency=0;	//模式频率（常亮模式自动忽略）
				rgb_set.request.red_value=0;	//红色光亮度范围（0-255），在线颜色选择器http://tools.jb51.net/static/colorpicker/
				rgb_set.request.green_value=0;	//绿色光亮度范围（0-255）
				rgb_set.request.blue_value=255;	//蓝色光亮度范围（0-255）
				switch (voiceVel_flag)
				{
					/* 0：休眠
				 	 * 1：停止
				 	 * 2：前进
				 	 * 3：后退
				 	 * 4：左转
				 	 * 5：右转
				 	 * 6：左移
				 	 * 7：右移
					*/
					case 0://休眠,此部分操作在语音包的call_recognition.cpp
						break;
					case 1://停止
						vel_cmd.linear.x = 0;//纵向运动
						vel_cmd.linear.y = 0;//横向运动，右手坐标系
						vel_cmd.angular.z = 0;//自旋运动，右手坐标系
						vel_pub.publish(vel_cmd);
						break;
					case 2://前进
		        vel_cmd.linear.x = RC_Info.vra*0.003;//纵向运动
		        vel_cmd.linear.y = 0;//横向运动，右手坐标系
		        vel_cmd.angular.z = 0;//自旋运动，右手坐标系
		        vel_pub.publish(vel_cmd);
						break;
					case 3://后退
						vel_cmd.linear.x = -RC_Info.vra*0.003;//纵向运动
						vel_cmd.linear.y = 0;//横向运动，右手坐标系
						vel_cmd.angular.z = 0;//自旋运动，右手坐标系
						vel_pub.publish(vel_cmd);
						break;
					case 4://左转
						vel_cmd.linear.x = 0;//纵向运动
						vel_cmd.linear.y = 0;//横向运动，右手坐标系
						vel_cmd.angular.z = RC_Info.vra*0.003;//自旋运动，右手坐标系
						vel_pub.publish(vel_cmd);
						break;
					case 5://右转
						vel_cmd.linear.x = 0;//纵向运动
						vel_cmd.linear.y = 0;//横向运动，右手坐标系
						vel_cmd.angular.z = -RC_Info.vra*0.003;//自旋运动，右手坐标系
						vel_pub.publish(vel_cmd);
						break;
					case 6://左移
						vel_cmd.linear.x = 0;//纵向运动
						vel_cmd.linear.y = RC_Info.vra*0.003;//横向运动，右手坐标系
						vel_cmd.angular.z = 0;//自旋运动，右手坐标系
						vel_pub.publish(vel_cmd);
						break;
					case 7://右移
						vel_cmd.linear.x = 0;//纵向运动
						vel_cmd.linear.y = -RC_Info.vra*0.003;//横向运动，右手坐标系
						vel_cmd.angular.z = 0;//自旋运动，右手坐标系
						vel_pub.publish(vel_cmd);
						break;
					default: 
						break;
				}
			}else if(RC_Info.swd<0 && RC_Info.swc>0){//模式3，绿色
				radio_mode.data=3;
	        	  
		    rgb_set.request.mode_type=0;	//0常亮；1呼吸；2闪烁
        rgb_set.request.frequency=0;	//模式频率（常亮模式自动忽略）
        rgb_set.request.red_value=0;	//红色光亮度范围（0-255），在线颜色选择器http://tools.jb51.net/static/colorpicker/
        rgb_set.request.green_value=255;	//绿色光亮度范围（0-255）
        rgb_set.request.blue_value=0;	//蓝色光亮度范围（0-255）
        double secs;
        double secs1;


				if(arry_flag==1){
					if(RC_Info.swa<0){
						vel_cmd.linear.x = 0;//纵向运动
						vel_cmd.linear.y = 0;//横向运动，右手坐标系
						vel_cmd.angular.z = 1;//自旋运动，右手坐标系
						vel_pub.publish(vel_cmd);
						secs1 =ros::Time::now().toSec();
						if((secs1-secs)>arry_time){
							mapclean_client.call(srv);
							printf("%f\n",secs);
							arry_flag=0;
						}
					}else{
						vel_cmd.linear.x = 0;//纵向运动
						vel_cmd.linear.y = 0;//横向运动，右手坐标系
						vel_cmd.angular.z = 0;//自旋运动，右手坐标系
						vel_pub.publish(vel_cmd);
						init_pose.header.frame_id = "map";
						init_pose.header.stamp = ros::Time::now();
						init_pose.pose.pose.orientation.z=-0.85;
						init_pose.pose.pose.orientation.w=0.37;
						init_pub.publish(init_pose);
						mapclean_client.call(srv);
						arry_flag=0;
					}
				}else{
					if(RC_Info.swb<0){
						if(RC_Info.swa<0){array_client.call(srv);}
						arry_flag=1;
						vel_cmd.linear.x = 0;//纵向运动
						vel_cmd.linear.y = 0;//横向运动，右手坐标系
						vel_cmd.angular.z = 0;//自旋运动，右手坐标系
						vel_pub.publish(vel_cmd);
						secs =ros::Time::now().toSec();
					}else{
						vel_cmd.linear.x = 0;//纵向运动
						vel_cmd.linear.y = 0;//横向运动，右手坐标系
						vel_cmd.angular.z = 0;//自旋运动，右手坐标系
						vel_pub.publish(vel_cmd);
					}
				}
			}else if(RC_Info.swd<0 && RC_Info.swc==0){//模式4，紫色
        radio_mode.data=4;
    	  
        rgb_set.request.mode_type=0;	//0常亮；1呼吸；2闪烁
        rgb_set.request.frequency=0;	//模式频率（常亮模式自动忽略）
        rgb_set.request.red_value=255;	//红色光亮度范围（0-255），在线颜色选择器http://tools.jb51.net/static/colorpicker/
        rgb_set.request.green_value=0;	//绿色光亮度范围（0-255）
        rgb_set.request.blue_value=255;	//蓝色光亮度范围（0-255）
        if(RC_Info.swb<0){
          /* 定义航点的消息结构体，并通过sendGoal发送出去 */
          move_base_msgs::MoveBaseGoal goal;
          goal.target_pose.header.frame_id = "map";
          goal.target_pose.header.stamp = ros::Time::now();
          goal.target_pose.pose = pose_list[0];
          ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
          //ac.cancelGoal();
          mapclean_client.call(srv);
        }
			}else if(RC_Info.swd<0 && RC_Info.swc<0){//模式5，白色
				char radio5_flag=0;
				if(radio_mode.data!=5){
					radio5_flag=1;
				}
        radio_mode.data=5;
        if(radio5_flag==1){
        	system("play ~/ucar_ws/src/mp3/voice_10.mp3");
        	radio5_flag=0;
        }
        rgb_set.request.mode_type=0;	//0常亮；1呼吸；2闪烁
        rgb_set.request.frequency=0;	//模式频率（常亮模式自动忽略）
        rgb_set.request.red_value=255;	//红色光亮度范围（0-255），在线颜色选择器http://tools.jb51.net/static/colorpicker/
        rgb_set.request.green_value=255;	//绿色光亮度范围（0-255）
        rgb_set.request.blue_value=255;	//蓝色光亮度范围（0-255）
			}
		}else{//故障模式，遥控器失联红闪,电池没电黄闪
    	  radio_mode.data=6;
    	  if(RC_Info.rssi==0){
  	      rgb_set.request.mode_type=2;	//0常亮；1呼吸；2闪烁
  	      rgb_set.request.frequency=5;	//模式频率（常亮模式自动忽略）
  	      rgb_set.request.red_value=255;	//红色光亮度范围（0-255），在线颜色选择器http://tools.jb51.net/static/colorpicker/
  	      rgb_set.request.green_value=0;	//绿色光亮度范围（0-255）
  	      rgb_set.request.blue_value=0;	//蓝色光亮度范围（0-255）
    	  }
    	  if(bat.percentage<bat_limit){
  	      rgb_set.request.mode_type=2;	//0常亮；1呼吸；2闪烁
  	      rgb_set.request.frequency=5;	//模式频率（常亮模式自动忽略）
  	      rgb_set.request.red_value=255;	//红色光亮度范围（0-255），在线颜色选择器http://tools.jb51.net/static/colorpicker/
  	      rgb_set.request.green_value=255;	//绿色光亮度范围（0-255）
  	      rgb_set.request.blue_value=0;	//蓝色光亮度范围（0-255）
    	  }
    	  vel_cmd.linear.x = 0;//纵向运动
    	  vel_cmd.linear.y = 0;//横向运动，右手坐标系
    	  vel_cmd.angular.z = 0;//自旋运动，右手坐标系
    	  vel_pub.publish(vel_cmd);
		}
		radio_mode_pub.publish(radio_mode);
		led_client.call(rgb_set);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
