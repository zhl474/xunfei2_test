#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int goal_num;
void get_param()
{
    ros::param::get("control_start/goal_num",goal_num);

}


void send_goal(MoveBaseClient& ac, double x, double y, double z, double w) {
    move_base_msgs::MoveBaseGoal goal;

    // 设置目标点
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.z = z;
    goal.target_pose.pose.orientation.w = w;

    // 发送目标点
    ac.sendGoal(goal);

    // 等待结果
    ac.waitForResult();

    // 检查是否成功到达
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("success");
    else
        ROS_INFO("fail");
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "simple_navigation_goals");
    ros::NodeHandle nh;

    MoveBaseClient ac("move_base", true);

    ROS_INFO("Waiting for the move_base action server to come up");
    ac.waitForServer();

    // 移动到A点
    send_goal(ac, 1.0, 1.5, 0.0, 1.0);

    // 在A点停留3秒
    ros::Duration(3.0).sleep();

    // 移动到B点
    send_goal(ac, 0.0, 0.0, 0.0, 1.0);

    // 移动到C点
    send_goal(ac, 2.0, 0.0, 0.0, 1.0);

    return 0;
}

