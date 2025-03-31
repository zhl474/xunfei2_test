#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
double dis_long;
double ix,px,py;
double robot_v;
double distance_17;
int i = 0;
ros::Time current_time;

int main(int argc,char ** argv){
    ros::init(argc,argv,"distance");
    ros::NodeHandle n;
    ros::Publisher command_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);

    std::cout << "please input  theta(rad)" <<std::endl;
    std::cin >>dis_long;
    std::cout << "please input w(rad/s)" << std::endl;
    std::cin >> robot_v;
// please input distance(m)
// 0.45
// please input velocity(m/s)
// 0.8
    geometry_msgs::Twist com_msg;
    ros::Time before = ros::Time::now();
    ros::Time after = ros::Time::now();
    while(ros::ok()){
        distance_17 = 0;
        before = ros::Time::now();
        after = ros::Time::now();
        while(distance_17< dis_long){
            com_msg.linear.y = robot_v;
            command_pub.publish(com_msg);
            after = ros::Time::now();
            distance_17 = robot_v *((after - before).toSec());
        }  
        ros::Duration(0.5).sleep();
        ROS_INFO("DISTANCE = %f     %f        %f",distance_17,dis_long,robot_v);
        break; 
    }
}