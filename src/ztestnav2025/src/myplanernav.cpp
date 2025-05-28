#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void init_goals( std::vector<move_base_msgs::MoveBaseGoal> &goals)
{
    tf2::Quaternion q;
    
    q.setRPY(0, 0, 0);
    goals[0].target_pose.pose.position.x = 1.75;
    goals[0].target_pose.pose.position.y = 0.27;
    goals[0].target_pose.pose.position.z = 0.0;
    goals[0].target_pose.pose.orientation.x = q.x();
    goals[0].target_pose.pose.orientation.y = q.y();
    goals[0].target_pose.pose.orientation.z = q.z();    
    goals[0].target_pose.pose.orientation.w = q.w();

    q.setRPY(0, 0, 1.57);
    goals[1].target_pose.pose.position.x = 1.75;
    goals[1].target_pose.pose.position.y = 0.27;
    goals[1].target_pose.pose.position.z = 0.0;
    goals[1].target_pose.pose.orientation.x = q.x();
    goals[1].target_pose.pose.orientation.y = q.y();
    goals[1].target_pose.pose.orientation.z = q.z();    
    goals[1].target_pose.pose.orientation.w = q.w();

}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    //-------------初始化movebase------------------//
    ros::init(argc,argv,"zhltest");
    ros::NodeHandle nh;
    MoveBaseClient ac("move_base", true);     
    //等待action回应
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    } 
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    

    //--------------寻找板子，发布目标点-------------//
    for(int i=0;i<2;i++)        //循环次数为目标点个数，发布目标点
    {
        
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = goals[i].target_pose.pose.position.x;
        goal.target_pose.pose.position.y = goals[i].target_pose.pose.position.y;
        goal.target_pose.pose.position.z = goals[i].target_pose.pose.position.z;
        goal.target_pose.pose.orientation.x = goals[i].target_pose.pose.orientation.x;
        goal.target_pose.pose.orientation.y = goals[i].target_pose.pose.orientation.y;
        goal.target_pose.pose.orientation.z = goals[i].target_pose.pose.orientation.z;
        goal.target_pose.pose.orientation.w = goals[i].target_pose.pose.orientation.w;

	    ROS_INFO("Sending goal -- %d",i);
        ac.sendGoal(goal);

        ac.waitForResult();
        ROS_INFO("测试");
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The base reach goal -- %d",i);
        else
            ROS_INFO("The base failed to move for some reason");
    }
    ROS_INFO("Navgation done..");
    
    ros::spin();

    return 0;
}

