#include "ros/ros.h"
#include <std_msgs/Int8.h>
#include <iostream>
#include <fstream>   
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

using namespace std;

geometry_msgs::Pose now_pose;
double yaw;

std_msgs::Int8 laser_mode;
ros::Publisher laser_mode_pub;

std::vector<float> keypoint;
std::vector<float> keypoint1;
std::vector<float> keypoint_angle;
std::vector<float> keypoint1_angle;

std::vector<float> ranges;
std::vector<float> range_angle;

std::vector<float> franges;
std::vector<float> frange_angle;

std::vector<float> ffrange_angle;

std::vector<std::vector<float>> fffrange_angle;

char flag = 0; 
char contorl_mode = 0;
void control_mode_callback(const std_msgs::Int8& msg){
  contorl_mode = msg.data;
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

  if(contorl_mode == 4 && flag < 5){
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
		int i, j;
		float dis;
		float angle;

    /* 遍历所有点，去掉不可用的点，获得能够代表位置信息的点  */
		for(i = 0; i < msg->ranges.size(); i++){
		  if(msg->ranges[i] > 0.01 && msg->ranges[i] < 1.5){
		    ranges.push_back(msg->ranges[i]);
		    angle = i * 0.0187;
		    range_angle.push_back(angle);
		  }
		}
		
		/* 为每个点检测距离是否与地图贴和，如果不帖和，则认为是一个可能存在障碍物/人物的板子上的点 */
		for(i = 0; i < ranges.size(); i++){
		  if(range_angle[i] > 5.495 || range_angle[i] < 0.785){
		    dis = 0.7 / abs(cos(range_angle[i]));
		  }else if(range_angle[i] > 0.785 && range_angle[i] < 2.355){
		    dis = 0.7 / abs(sin(range_angle[i]));
		  }else if(range_angle[i] > 2.355 && range_angle[i] <  3.925){
		    dis = 0.7 / abs(cos(range_angle[i]));
		  }else if(range_angle[i] > 3.925 && range_angle[i] < 5.495){
		    dis = 0.7 / abs(sin(range_angle[i]));
		  }else{
		    dis = 100;
		  }
		  //printf("dist: %f\r\n",dis);
		  if(ranges[i] - dis < -0.1 && dis < 5){
		    printf("dis:%f ranges:%f angle:%f\r\n",dis,ranges[i],range_angle[i]);
		    franges.push_back(ranges[i]);
		    frange_angle.push_back(range_angle[i]);
		  }
		}
		
		
		i = msg->ranges.size();
		printf("msg->ranges: %d\r\n",i);
		i = ranges.size();
		printf("ranges counts: %d\r\n",i);
		i = franges.size();
		printf("franges counts: %d\r\n",i);
		/*
		printf("ranges[0]: %f  range_angle[0]: %f\r\n",ranges[0],range_angle[0]);
		printf("ranges[30]: %f  range_angle[30]: %f\r\n",ranges[30],range_angle[30]);
		printf("ranges[0]: %f  range_angle[0]: %f\r\n",franges[0],frange_angle[0]);
		printf("ranges[30]: %f  range_angle[30]: %f\r\n",franges[30],frange_angle[30]);
		*/
		/* 将点根据密集程度分类，将比较分散的点视为杂点去掉 */
		for(i = 1; i < franges.size(); i++){
		  printf("franges: %3f  frange_angle: %3f\r\n",franges[i],frange_angle[i]);
		  if(frange_angle[i] - frange_angle[i-1] < 0.1){
		    ffrange_angle.push_back(frange_angle[i]);
		  }else{
		    if(ffrange_angle.size() > 5){
		      fffrange_angle.push_back(ffrange_angle);
		    }
		    ffrange_angle.clear();
		  }
		  if(i == franges.size() - 1){
		    if(ffrange_angle.size() > 5){
		      fffrange_angle.push_back(ffrange_angle);
		    }
		    ffrange_angle.clear();
		  }
		}

	  
		/*  */
		for(i = 0; i < fffrange_angle.size(); i++){
		  printf("-----variable: %3d  \r\n",i);
		  for(j = 0; j < fffrange_angle[i].size(); j++){
		    printf("fffrange_angle: %3f  \r\n",fffrange_angle[i][j]);
		    
		  }
		}
		
		ffrange_angle.clear();
		fffrange_angle.clear();
		ranges.clear();
		range_angle.clear();
		franges.clear();
		frange_angle.clear();
		
	}
}




int main(int argc, char **argv)
{

    ros::init(argc, argv, "laser_receive");
    ros::NodeHandle nh;
    ros::Subscriber control_mode_sub = nh.subscribe("/control_mode",10,control_mode_callback);
    ros::Subscriber laser_sub = nh.subscribe("/scan", 1, laserCallback);
    laser_mode_pub = nh.advertise<std_msgs::Int8>("/laser_mode", 1000);
    ros::spin();

    return 0;
}
