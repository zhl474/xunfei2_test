#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <iostream>
#include <std_srvs/Empty.h>

ros::Publisher run_mode_pub;
std_msgs::Int8 run_mode;
char contorl_mode=0;
char cv_mode=0;
ros::ServiceClient mapclean_client;
std_srvs::Empty srv;

void control_mode_callback(const std_msgs::Int8& msg){
  contorl_mode = msg.data;
  if(contorl_mode == 6){
    exit(EXIT_SUCCESS);
  }
}



void cv_mode_callback(const std_msgs::Int8& msg){
  if(msg.data == 0){//进入B区，二维码识别开始
		run_mode.data=1;
    run_mode_pub.publish(run_mode);
	}else if(msg.data == 1){//B区二维码识别完成
		run_mode.data=2;
    run_mode_pub.publish(run_mode);
	}else if(msg.data == 2){//进入C区
		run_mode.data=1;
    run_mode_pub.publish(run_mode);
	}else if(msg.data == 3){//进入C区识别完成
		run_mode.data=2;
    run_mode_pub.publish(run_mode);
	}
}


void voice_mode_callback(const std_msgs::Int8& msg){
	if(msg.data == 0||msg.data == 1||msg.data == 20){//接受到休眠指令或者发车指令
    run_mode.data=2;
    run_mode_pub.publish(run_mode);
  }else if(msg.data == 10){//接受到唤醒指令
	  /*if(contorl_mode == 0){//判断是否为第一次唤醒
			run_mode.data=2;
    	run_mode_pub.publish(run_mode);
		}else{
			run_mode.data=0;
    	run_mode_pub.publish(run_mode);
		}*/
		run_mode.data=2;
    run_mode_pub.publish(run_mode);
		while(!mapclean_client.call(srv)){
    	ROS_WARN("MAPCLEAN FAIL");
  	}
  }else{//接受到其它指令
		run_mode.data = 0;
    run_mode_pub.publish(run_mode);
	}
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "run_test");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
	

  
	run_mode_pub = n.advertise<std_msgs::Int8>("/run_mode", 1000);
	ros::Subscriber voice_mode_sub = n.subscribe("/voice_mode",10,voice_mode_callback);
	ros::Subscriber control_mode_sub = n.subscribe("/control_mode",10,control_mode_callback);
  ros::Subscriber odomSubscribe = n.subscribe("/cv_mode",1000,cv_mode_callback);
  mapclean_client = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  
	while(ros::ok())
	{
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
