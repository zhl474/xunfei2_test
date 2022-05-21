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

/* 重命名类定义，可通过Client来定义这个类（原型）actionlib::SimpleActionClient< ActionSpec > Class Template 
   < ActionSpec >是通过.action文件产生的msgs文件包含的
   move_base_msgs::MoveBaseAction对应的是move_base的.action文件产生的msg文件，roscd move_base_msgs可查看 */
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;
typedef actionlib::SimpleClientGoalState State;
State::StateEnum state;



move_base_msgs::MoveBaseGoal goal;
ros::Publisher control_mode_pub;
ros::Publisher last_mode_pub;
ros::Publisher init_pub;
ros::Publisher run_mode_pub;
ros::Publisher vel_pub;
ros::ServiceClient mapclean_client;
std_msgs::Int8 control_mode;
std_msgs::Int8 last_mode;
std_srvs::Empty srv;
char voice_mode=0;
char run_mode=0;
char cv_mode=0;
char aruco_mode=10;
char yolo =0 ;
char longhair = 0;
char glasses = 0;
char control_flag=0;
char goal_flag=0;
char guosai_flag=0;

/* https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1SimpleActionClient.html */
/* http://wiki.ros.org/actionlib */
/* https://www.guyuehome.com/908 */
/* ... */
geometry_msgs::Pose pose_list[7];
geometry_msgs::Pose pose_init;
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
  pose_list[0].position = setPoint(goal0_x, goal0_y, 0);
  pose_list[0].orientation = setQuaternion( goal0_z );
  
  pose_list[1].position = setPoint(goal1_x, goal1_y, 0);
  pose_list[1].orientation = setQuaternion( goal1_z );

  pose_list[2].position = setPoint(goal2_x, goal2_y, 0);
  pose_list[2].orientation = setQuaternion( goal2_z );

  pose_list[3].position = setPoint(goal3_x, goal3_y, 0);
  pose_list[3].orientation = setQuaternion( goal3_z );

  pose_list[4].position = setPoint(goal4_x, goal4_y, 0);
  pose_list[4].orientation = setQuaternion( goal4_z );

  pose_list[5].position = setPoint(goal5_x, goal5_y, 0);
  pose_list[5].orientation = setQuaternion( goal5_z );
  
  pose_list[6].position = setPoint(goal6_x, goal6_y, 0);
  pose_list[6].orientation = setQuaternion( goal6_z );
}




/* sendGoal回调函数 */
void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResult::ConstPtr& result)

{
  if(state==State::PREEMPTED){//表示取消了目标
    ROS_INFO("goal cancelled");
  }else{
		/*while(!mapclean_client.call(srv)){
			ROS_WARN("MAPCLEAN FAIL");
		}*/
		if(control_mode.data == 0){
		  
      if(guosai_flag == 1){

        last_mode.data = 2;
        last_mode_pub.publish(last_mode);

      }
      if(guosai_flag == 3){

        last_mode.data = 3;
        last_mode_pub.publish(last_mode);
        /*while(!mapclean_client.call(srv)){
			    ROS_WARN("MAPCLEAN FAIL");
		    }*/
      }
		}
		if(control_mode.data == 1){
		  
      system("play ~/ucar_ws/src/mp3/B.mp3");
      ROS_INFO("Saying B done ");
			last_mode.data = 20;
			last_mode_pub.publish(last_mode);
      control_mode.data = 2;
      control_mode_pub.publish(control_mode);
			/*while(!mapclean_client.call(srv)){
      	ROS_WARN("MAPCLEAN FAIL");
    	}*/
      
			ROS_INFO("Goal B done ");
		}else if(control_mode.data == 3){
			control_mode.data = 4;
      control_mode_pub.publish(control_mode);
			while(!mapclean_client.call(srv)){
      	ROS_WARN("MAPCLEAN FAIL");
    	}

			ROS_INFO("Goal C done ");
		}else if(control_mode.data == 5){
			control_mode.data = 6;
      control_mode_pub.publish(control_mode);
			ROS_INFO("Goal D done ");
			//system("play ~/ucar_ws/src/mp3/over.mp3");
		}
    
  }
}

void activeCb()
{
	goal_flag=1;
  ROS_INFO("Goal Received");
}

void feedbackCb(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
{
  //ROS_INFO("Got base_position of Feedback");
	
}


/* 激光雷达回调函数 */
double rx = 0 ;
double ry = 0 ;
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  if(control_flag == 0){
    /*std::vector<float> ranges=msg->ranges;
    std::cout<< ranges.size()<<" ";

    std::cout<< msg->header.stamp<<" ";
    std::cout<< msg->header.frame_id<<" ";
    std::cout<< msg->angle_min<<" ";
    std::cout<< msg->angle_max<<" ";
		std::cout<< msg->angle_increment<<" ";
		std::cout<< msg->time_increment<<" ";
		std::cout<< "dis_ranges:"<<  msg->range_min<<" ";
		std::cout<< msg->range_max<<std::endl;*/
		double ranges[3] = {};
    int rang_p[3] = {};
		int i = 0;
		for(i = 0; i < 10; i++){
		  if(msg->ranges[i] > 0.01 && msg->ranges[i] < 1.0){
		    ranges[0] = msg->ranges[i];
		    rang_p[0] = i;
		    break;
		  }
		}
		for(i = 0; i < 10; i++){
		  if(msg->ranges[i+90] > 0.01 && msg->ranges[i+90] < 1.0){
		    ranges[1] = msg->ranges[i + 90];
		    rang_p[1] = i + 90;
		    break;
		  }
		}
		for(i = 0; i < 10; i++){
		  if(msg->ranges[i+270] > 0.01 && msg->ranges[i+270] < 1.0){
		    ranges[2] = msg->ranges[i+270];
		    rang_p[2] = i + 270;
		    break;
		  }
		}
		std::cout<< " 1_ranges:"  <<  rang_p[0]  << " 1_dis:" << ranges[0]<<" ";
		std::cout<< " 2_ranges:"  <<  rang_p[1]  << " 1_dis:" << ranges[1]<<" ";
		std::cout<< " 3_ranges:"  <<  rang_p[2]  << " 1_dis:" << ranges[2]<<std::endl;
		
		double x = 0;
		x = ranges[1] + ranges[2];
		if(x > 0.95 && x < 1.05){
		  rx = ranges[2];
		  ry = ranges[0] + 0.11;
		  std::cout<< " x:"  <<  rx  << std::endl;
		  std::cout<< " y:"  <<  ry  << std::endl;
		  pose_init.position = setPoint(rx, ry, 0);
		  pose_init.orientation = setQuaternion( 1.57 );
		}else{
		  system("play ~/ucar_ws/src/mp3/voice_11.mp3");
		  ROS_INFO("INITPOS FAIL");
		  assert(0);
		}
  }
  
}

void cv_mode_callback(const std_msgs::Int8& msg){
  cv_mode = msg.data;
}

/*void aruco_mode_callback(const std_msgs::Int8& msg){
  aruco_mode = msg.data;
}*/

void voice_mode_callback(const std_msgs::Int8& msg){
  voice_mode = msg.data;
}

void run_mode_callback(const std_msgs::Int8& msg){
	run_mode=msg.data;
}

void yolo_callback(const std_msgs::Int8& msg){
	yolo=msg.data;
}

void longhair_callback(const std_msgs::Int8& msg){
	longhair=msg.data;
}

void glasses_callback(const std_msgs::Int8& msg){
	glasses=msg.data;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "control_test");
	ros::NodeHandle n("~");
	ros::Rate loop_rate(100);
	

  n.getParam("stop_point",stop_point);
	n.getParam("goal0_x",goal0_x);
	n.getParam("goal0_y",goal0_y);
	n.getParam("goal0_z",goal0_z);

	n.getParam("goal1_x",goal1_x);
	n.getParam("goal1_y",goal1_y);
	n.getParam("goal1_z",goal1_z);

	n.getParam("goal2_x",goal2_x);
	n.getParam("goal2_y",goal2_y);
	n.getParam("goal2_z",goal2_z);

	n.getParam("goal3_x",goal3_x);
	n.getParam("goal3_y",goal3_y);
	n.getParam("goal3_z",goal3_z);

	n.getParam("goal4_x",goal4_x);
	n.getParam("goal4_y",goal4_y);
	n.getParam("goal4_z",goal4_z);

	n.getParam("goal5_x",goal5_x);
	n.getParam("goal5_y",goal5_y);
	n.getParam("goal5_z",goal5_z);
	
	n.getParam("goal6_x",goal6_x);
	n.getParam("goal6_y",goal6_y);
	n.getParam("goal6_z",goal6_z);

	init_goalList();
	
  geometry_msgs::PoseWithCovarianceStamped init_pose;
  geometry_msgs::Twist vel_cmd;
  
  
  vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  init_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000);
  mapclean_client = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  
  last_mode_pub = n.advertise<std_msgs::Int8>("/last_mode", 1000);
	control_mode_pub = n.advertise<std_msgs::Int8>("/control_mode", 1000);
	ros::Subscriber voice_mode_sub = n.subscribe("/voice_mode",10,voice_mode_callback);
	ros::Subscriber run_mode_sub = n.subscribe("/run_mode",10,run_mode_callback);
	ros::Subscriber yolo_sub = n.subscribe("/yolo",10,yolo_callback);
	ros::Subscriber longhair_sub = n.subscribe("/longhair",10,longhair_callback);
	ros::Subscriber glasses_sub = n.subscribe("/glasses",10,glasses_callback);
	ros::Subscriber cv_mode_Subscribe = n.subscribe("/cv_mode",10,cv_mode_callback);
	//ros::Subscriber aruco_Subscribe = n.subscribe("/aruco_mode",1000,aruco_mode_callback);
	ros::Subscriber laser_sub = n.subscribe("/scan", 100, laserCallback);
  
  
	control_mode.data = 0;
	control_mode_pub.publish(control_mode);

	/* SimpleActionClient (ros::NodeHandle &n, const std::string &name, bool spin_thread=true)为构造函数，在创建对象时会自动调用
		move_base表示action服务器的名字，服务器是在movebase源码里定义好了的，roscd move_base_msgs
		true表示调用ros::spin()，true -> don't need ros::spin()，如果为true，则启动一个线程来运行ros::spin()，ros::spin()用来开启订阅 */
	Client ac("move_base", true);
	/* waitForServer等待ActionServer连接到此客户端，这里设置为60s */
	if (!ac.waitForServer(ros::Duration(6)))
	{
			ROS_ERROR("Can't connected to move base server");
			assert(0);
	}
	/* 第一个参数代表目标动作，
			第二个参数指定目标goal被目标server成功完成/取消的回调函数，默认 SimpleDoneCallback(), 
			第三个参数指定目标goal被目标server成功接收的回调函数，默认 SimpleActiveCallback(), 
			第四个参数表示目标Client接收到目标server返回数据回调函数，默认 SimpleFeedbackCallback() 
	*/
	//ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
	char i=0;
	
	while(ros::ok())
	{
	  if(!control_flag && rx != 0){//初始状态重新定位,必须要一直刷，不能设置为只运行一次
	    for(i=0;i<10;i++){
				init_pose.header.frame_id = "map";
				init_pose.header.stamp = ros::Time::now();
				init_pose.pose.pose = pose_init;
				init_pub.publish(init_pose);
				while(!mapclean_client.call(srv)){
        	ROS_WARN("MAPCLEAN FAIL");
      	}
			}
			ROS_INFO("INITPOS OK");
			control_flag=1;
	  }
    if(run_mode == 0){//语音控制模式
      if(goal_flag){
        ac.cancelGoal();//先取消导航目标点
        goal_flag = 0;
      }
			switch(voice_mode)
			{
				case 0://休眠,在刚启动的时候各个mode都默认为0也会进入这个
					vel_cmd.linear.x = 0;//纵向运动
					vel_cmd.linear.y = 0;//横向运动，右手坐标系
					vel_cmd.angular.z = 0;//自旋运动，右手坐标系
					vel_pub.publish(vel_cmd);
					break;
				case 2://前进
					vel_cmd.linear.x = 0.1;//纵向运动
					vel_cmd.linear.y = 0;//横向运动，右手坐标系
					vel_cmd.angular.z = 0;//自旋运动，右手坐标系
					vel_pub.publish(vel_cmd);
					break;
				case 3://后退
					vel_cmd.linear.x = -0.1;//纵向运动
					vel_cmd.linear.y = 0;//横向运动，右手坐标系
					vel_cmd.angular.z = 0;//自旋运动，右手坐标系
					vel_pub.publish(vel_cmd);
					break;
				case 4://左转
					vel_cmd.linear.x = 0;//纵向运动
					vel_cmd.linear.y = 0;//横向运动，右手坐标系
					vel_cmd.angular.z = 0.3;//自旋运动，右手坐标系
					vel_pub.publish(vel_cmd);
					break;
				case 5://右转
					vel_cmd.linear.x = 0;//纵向运动
					vel_cmd.linear.y = 0;//横向运动，右手坐标系
					vel_cmd.angular.z = -0.3;//自旋运动，右手坐标系
					vel_pub.publish(vel_cmd);
					break;
				case 6://左移
					vel_cmd.linear.x = 0;//纵向运动
					vel_cmd.linear.y = 0.1;//横向运动，右手坐标系
					vel_cmd.angular.z = 0;//自旋运动，右手坐标系
					vel_pub.publish(vel_cmd);
					break;
				case 7://右移
					vel_cmd.linear.x = 0;//纵向运动
					vel_cmd.linear.y = -0.1;//横向运动，右手坐标系
					vel_cmd.angular.z = 0;//自旋运动，右手坐标系
					vel_pub.publish(vel_cmd);
					break;
				case 10://唤醒
					vel_cmd.linear.x = 0;//纵向运动
					vel_cmd.linear.y = 0;//横向运动，右手坐标系
					vel_cmd.angular.z = 0;//自旋运动，右手坐标系
					vel_pub.publish(vel_cmd);
					break;
				default: 
					break;
			}
    }else if(run_mode == 1){//图像处理控制模式
			if(goal_flag){
        ac.cancelGoal();//先取消导航目标点
        goal_flag = 0;
      }
			if(cv_mode == 0){
				vel_cmd.linear.x = 0;//纵向运动
				vel_cmd.linear.y = 0;//横向运动，右手坐标系
				vel_cmd.angular.z = 0;//自旋运动，右手坐标系
				vel_pub.publish(vel_cmd);
			}
    	
			
    }else{//导航控制模式
			/*首先确保连接到了movebase_actionlib服务*/
			if(!ac.isServerConnected()){
				ROS_WARN("Move_Base Actionlib Server Disonnected");
				if(!ac.waitForServer(ros::Duration(5)))
				{
					ROS_ERROR("Can't connected to move base server");
					assert(0);
				}
			}
			switch(control_mode.data)
			{
				case 0://此时为第一次唤醒状态
          if(voice_mode == 20){
            /*if(!goal_flag){
								goal.target_pose.header.frame_id = "map";
								goal.target_pose.header.stamp = ros::Time::now();
								goal.target_pose.pose = pose_list[0];
								ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
								ROS_INFO("FIRST AWAKE AND SEND GOAL(TO B)");
							}else{//目标发送成功
								control_mode.data = 1;
								control_mode_pub.publish(control_mode);
							}*/
						if(goal5_x < 10){
						  if(guosai_flag == 0){
								if(!goal_flag){
									goal.target_pose.header.frame_id = "map";
									goal.target_pose.header.stamp = ros::Time::now();
									goal.target_pose.pose = pose_list[5];
									ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
									ROS_INFO("FIRST AWAKE AND SEND GOAL(TO detect1)");
								}else{//目标发送成功
								  if(goal6_x < 10){guosai_flag = 1;}
									if(goal6_x > 10){guosai_flag = 3;}
								}
							}

						  if(guosai_flag == 2){
								if(!goal_flag){
									goal.target_pose.header.frame_id = "map";
									goal.target_pose.header.stamp = ros::Time::now();
									goal.target_pose.pose = pose_list[6];
									ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
									ROS_INFO("FIRST AWAKE AND SEND GOAL(TO detect2)");
								}else{//目标发送成功
									guosai_flag = 3;
								}
							}
						  if(guosai_flag == 4){
								if(!goal_flag){
								  goal.target_pose.header.frame_id = "map";
								  goal.target_pose.header.stamp = ros::Time::now();
								  goal.target_pose.pose = pose_list[0];
								  ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
								  ROS_INFO("FIRST AWAKE AND SEND GOAL(TO B)");
								  ROS_INFO("guosai_flag:%d",guosai_flag);
							  }else{//目标发送成功
							  	control_mode.data = 1;
								  control_mode_pub.publish(control_mode);
							  }
							}
							if(guosai_flag == 1){
								if(cv_mode == 10){
								  goal_flag = 0;
								  guosai_flag = 2;
								}
							}
							if(guosai_flag == 3){
								if(cv_mode == 11){
								  goal_flag = 0;
								  guosai_flag = 4;
								}
							}
						}
						if(goal5_x > 10){
							if(guosai_flag == 0){
							  if(goal6_x < 10){
									if(!goal_flag){
										goal.target_pose.header.frame_id = "map";
										goal.target_pose.header.stamp = ros::Time::now();
										goal.target_pose.pose = pose_list[6];
										ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
										ROS_INFO("FIRST AWAKE AND SEND GOAL(TO detect1)");
									}else{//目标发送成功
										guosai_flag = 3;
									}
								}else{
								  guosai_flag = 3;
								}
							}
						  if(guosai_flag == 4){
								if(!goal_flag){
								  goal.target_pose.header.frame_id = "map";
								  goal.target_pose.header.stamp = ros::Time::now();
								  goal.target_pose.pose = pose_list[0];
								  ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
								  ROS_INFO("FIRST AWAKE AND SEND GOAL(TO B)");
							  }else{//目标发送成功
							  	control_mode.data = 1;
								  control_mode_pub.publish(control_mode);
							  }
							}
							if(guosai_flag == 3){
								if(cv_mode == 11){
								  goal_flag = 0;
								  guosai_flag = 4;
								}
							}
						}
					}
					break;
				case 1://此时为去第一个目标点（B区）状态
					if(!goal_flag){
						goal.target_pose.header.frame_id = "map";
						goal.target_pose.header.stamp = ros::Time::now();
						goal.target_pose.pose = pose_list[0];
						ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
						ROS_INFO("SEND GOAL(TO B)");
					}else{//目标发送成功
						control_mode.data = 1;
						control_mode_pub.publish(control_mode);
					}
					break;
				case 2://此时为到达第一个目标点（B区）状态
				  if(cv_mode == 1){//
						if(!goal_flag){
							goal.target_pose.header.frame_id = "map";
							goal.target_pose.header.stamp = ros::Time::now();
							goal.target_pose.pose = pose_list[4];
							ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
							ROS_INFO("SEND GOAL(TO C)");
						}else{//目标发送成功
							control_mode.data = 3;
							control_mode_pub.publish(control_mode);
						}
				  }else{
				  	ROS_INFO("WAIT FOR OPENCV ARUCO");
				  	goal_flag = 0;
				  }
          
					break;
				case 3://此时为去第二个目标点（C区）状态
					if(!goal_flag){
						goal.target_pose.header.frame_id = "map";
						goal.target_pose.header.stamp = ros::Time::now();
						goal.target_pose.pose = pose_list[4];
						ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
						ROS_INFO("SEND GOAL(TO C)");
					}else{//目标发送成功
						control_mode.data = 3;
						control_mode_pub.publish(control_mode);
					}
					break;
				case 4://到达第二个目标点（C区）状态
					if(cv_mode == 3){//
						if(stop_point == 0){
							if(!goal_flag){
								goal.target_pose.header.frame_id = "map";
								goal.target_pose.header.stamp = ros::Time::now();
								goal.target_pose.pose = pose_list[1];
								ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
								ROS_INFO("SEND GOAL(TO D1)");
							}else{//目标发送成功
								control_mode.data = 5;
								control_mode_pub.publish(control_mode);
							}
						}else if(stop_point == 1){
							if(!goal_flag){
								goal.target_pose.header.frame_id = "map";
								goal.target_pose.header.stamp = ros::Time::now();
								goal.target_pose.pose = pose_list[2];
								ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
								ROS_INFO("SEND GOAL(TO D2)");
							}else{//目标发送成功
								control_mode.data = 5;
								control_mode_pub.publish(control_mode);
							}
						}else if(stop_point == 2){
							if(!goal_flag){
								goal.target_pose.header.frame_id = "map";
								goal.target_pose.header.stamp = ros::Time::now();
								goal.target_pose.pose = pose_list[3];
								ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
								ROS_INFO("SEND GOAL(TO D3)");
							}else{//目标发送成功
								control_mode.data = 5;
								control_mode_pub.publish(control_mode);
							}
						}
					}else{
				  	ROS_INFO("WAIT FOR OPENCV SWITCHING");
				  	goal_flag = 0;
				  }
					break;
				case 5://此时为去第三个目标点（D区）状态
						if(stop_point == 0){
							if(!goal_flag){
								goal.target_pose.header.frame_id = "map";
								goal.target_pose.header.stamp = ros::Time::now();
								goal.target_pose.pose = pose_list[1];
								ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
								ROS_INFO("SEND GOAL(TO D1)");
							}else{//目标发送成功
								control_mode.data = 5;
								control_mode_pub.publish(control_mode);
							}
						}else if(stop_point == 1){
							if(!goal_flag){
								goal.target_pose.header.frame_id = "map";
								goal.target_pose.header.stamp = ros::Time::now();
								goal.target_pose.pose = pose_list[2];
								ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
								ROS_INFO("SEND GOAL(TO D2)");
							}else{//目标发送成功
								control_mode.data = 5;
								control_mode_pub.publish(control_mode);
							}
						}else if(stop_point == 2){
							if(!goal_flag){
								goal.target_pose.header.frame_id = "map";
								goal.target_pose.header.stamp = ros::Time::now();
								goal.target_pose.pose = pose_list[3];
								ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
								ROS_INFO("SEND GOAL(TO D3)");
							}else{//目标发送成功
								control_mode.data = 5;
								control_mode_pub.publish(control_mode);
							}
						}
					break;
				case 6://
          exit(EXIT_SUCCESS);
				  if(yolo == 1){
				    ROS_INFO("longhair: %d",longhair);
				    ROS_INFO("glasses: %d",glasses);
				    if(longhair == 0){
				      system("play ~/ucar_ws/src/mp3/0longhair.mp3");
				    }else if(longhair == 1){
				      system("play ~/ucar_ws/src/mp3/1longhair.mp3");
				    }else if(longhair == 2){
				      system("play ~/ucar_ws/src/mp3/2longhair.mp3");
				    }
				    if(glasses == 0){
				      system("play ~/ucar_ws/src/mp3/0glasses.mp3");
				    }else if(glasses == 1){
				      system("play ~/ucar_ws/src/mp3/1glasses.mp3");
				    }else if(glasses == 2){
				      system("play ~/ucar_ws/src/mp3/2glasses.mp3");
				    }
				    yolo = 0;
				  }
					break;
				default: 
					break;
			}
    }
	    
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}



















