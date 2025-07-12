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

    // ����Ŀ���
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.z = z;
    goal.target_pose.pose.orientation.w = w;

    // ����Ŀ���
    ac.sendGoal(goal);

    // �ȴ����
    ac.waitForResult();

    // ����Ƿ�ɹ�����
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

    // �ƶ���A��
    send_goal(ac, 1.0, 1.5, 0.0, 1.0);

    // ��A��ͣ��3��
    ros::Duration(3.0).sleep();

    // �ƶ���B��
    send_goal(ac, 0.0, 0.0, 0.0, 1.0);

    // �ƶ���C��
    send_goal(ac, 2.0, 0.0, 0.0, 1.0);

    return 0;
}

