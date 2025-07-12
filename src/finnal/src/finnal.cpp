#include <ros/ros.h>
#include <stdio.h>
#include <std_msgs/Int8.h>
#include <iostream>
#include <unistd.h> 

char contorl_mode=0;
char yolo=0;
char longhair = 0;
char glasses = 0;
ros::Publisher finnal_pub;
std_msgs::Int8 finnal_mode;
char flag = 0;

void control_mode_callback(const std_msgs::Int8& msg){
  contorl_mode = msg.data;
  

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "finnal");
  ros::NodeHandle nh_("~");
  ros::Rate loop_rate(10);
  
	finnal_pub = nh_.advertise<std_msgs::Int8>("/finnal", 1000);
	ros::Subscriber control_mode_sub = nh_.subscribe("/control_mode",10,control_mode_callback);
	
	int i = 0;
  while(ros::ok()){
    if(flag == 0){
      if(contorl_mode == 6){
        i++;
		    //ROS_INFO("iiiiii:  %d ",i);
        //if(i == 50){
          //system("play ~/ucar_ws/src/mp3/over1.mp3");

       // }
        if(i == 50){
          system("play ~/ucar_ws/src/mp3/newover.mp3");
          flag = 1;
        }
      }
    }
    if(flag == 1){
    	finnal_mode.data = 1;
	    finnal_pub.publish(finnal_mode);
	  }
		ros::spinOnce();

		loop_rate.sleep();
  }
  return 0;
}
