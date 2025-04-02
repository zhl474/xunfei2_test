#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void init_goals( std::vector<move_base_msgs::MoveBaseGoal> &goals)
{
    goals[0].target_pose.pose.position.x = 2.85;
    goals[0].target_pose.pose.position.y = 1.05;
    goals[0].target_pose.pose.position.z = 0.0;
    goals[0].target_pose.pose.orientation.x = 0;
    goals[0].target_pose.pose.orientation.y = 0;
    goals[0].target_pose.pose.orientation.z = 0;    
    goals[0].target_pose.pose.orientation.w = 1;

    goals[1].target_pose.pose.position.x = 2.85;
    goals[1].target_pose.pose.position.y = 1.55;
    goals[1].target_pose.pose.position.z = 0.0;
    goals[1].target_pose.pose.orientation.x = 0;
    goals[1].target_pose.pose.orientation.y = 0;
    goals[1].target_pose.pose.orientation.z = 0.707;    
    goals[1].target_pose.pose.orientation.w = 0.707;

    goals[2].target_pose.pose.position.x = 1.14;
    goals[2].target_pose.pose.position.y = 1.55;
    goals[2].target_pose.pose.position.z = 0.0;
    goals[2].target_pose.pose.orientation.x = 0;
    goals[2].target_pose.pose.orientation.y = 0;
    goals[2].target_pose.pose.orientation.z = 0;    
    goals[2].target_pose.pose.orientation.w = 1;

    goals[3].target_pose.pose.position.x = 1.14;
    goals[3].target_pose.pose.position.y = 2.04;
    goals[3].target_pose.pose.position.z = 0.0;
    goals[3].target_pose.pose.orientation.x = 0;
    goals[3].target_pose.pose.orientation.y = 0;
    goals[3].target_pose.pose.orientation.z = -0.707;    
    goals[3].target_pose.pose.orientation.w = 0.707;

    goals[4].target_pose.pose.position.x = 3.12;
    goals[4].target_pose.pose.position.y = 2.04;
    goals[4].target_pose.pose.position.z = 0.0;
    goals[4].target_pose.pose.orientation.x = 0;
    goals[4].target_pose.pose.orientation.y = 0;
    goals[4].target_pose.pose.orientation.z = 0;    
    goals[4].target_pose.pose.orientation.w = 1;

    goals[5].target_pose.pose.position.x = 3.12;
    goals[5].target_pose.pose.position.y = 2.55;
    goals[5].target_pose.pose.position.z = 0.0;
    goals[5].target_pose.pose.orientation.x = 0;
    goals[5].target_pose.pose.orientation.y = 0;
    goals[5].target_pose.pose.orientation.z = 0.707;    
    goals[5].target_pose.pose.orientation.w = 0.707;

    goals[6].target_pose.pose.position.x = 1.12;
    goals[6].target_pose.pose.position.y = 2.55;
    goals[6].target_pose.pose.position.z = 0.0;
    goals[6].target_pose.pose.orientation.x = 0;
    goals[6].target_pose.pose.orientation.y = 0;
    goals[6].target_pose.pose.orientation.z = 0;    
    goals[6].target_pose.pose.orientation.w = 1;

    goals[7].target_pose.pose.position.x = 1.12;
    goals[7].target_pose.pose.position.y = 3.85;
    goals[7].target_pose.pose.position.z = 0.0;
    goals[7].target_pose.pose.orientation.x = 0;
    goals[7].target_pose.pose.orientation.y = 0;
    goals[7].target_pose.pose.orientation.z = -0.38;    
    goals[7].target_pose.pose.orientation.w = 0.92;

    goals[8].target_pose.pose.position.x = 3.05;
    goals[8].target_pose.pose.position.y = 5.06;
    goals[8].target_pose.pose.position.z = 0.0;
    goals[8].target_pose.pose.orientation.x = 0;
    goals[8].target_pose.pose.orientation.y = 0;
    goals[8].target_pose.pose.orientation.z = -0.707;    
    goals[8].target_pose.pose.orientation.w = 0.707;

    goals[9].target_pose.pose.position.x = 3.63;
    goals[9].target_pose.pose.position.y = 5.06;
    goals[9].target_pose.pose.position.z = 0.0;
    goals[9].target_pose.pose.orientation.x = 0;
    goals[9].target_pose.pose.orientation.y = 0;
    goals[9].target_pose.pose.orientation.z = 0;    
    goals[9].target_pose.pose.orientation.w = -1;

    goals[10].target_pose.pose.position.x = 3.63;
    goals[10].target_pose.pose.position.y = 2.95;
    goals[10].target_pose.pose.position.z = 0.0;
    goals[10].target_pose.pose.orientation.x = 0;
    goals[10].target_pose.pose.orientation.y = 0;
    goals[10].target_pose.pose.orientation.z = -0.55;    
    goals[10].target_pose.pose.orientation.w = 0.83;

    goals[11].target_pose.pose.position.x = 4.62;
    goals[11].target_pose.pose.position.y = 1.37;
    goals[11].target_pose.pose.position.z = 0.0;
    goals[11].target_pose.pose.orientation.x = 0;
    goals[11].target_pose.pose.orientation.y = 0;
    goals[11].target_pose.pose.orientation.z = 0;    
    goals[11].target_pose.pose.orientation.w = 1;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"zhltest");
    ros::NodeHandle nh;

    MoveBaseClient ac("move_base", true); 
    
    //等待action回应
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
    ROS_INFO("Waiting for the move_base action server to come up");
    }

    std::vector<move_base_msgs::MoveBaseGoal> goals(12);
    init_goals(goals);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    
    for(int i=0;i<12;i++)        //循环次数为目标点个数，发布目标点
    {
        
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = goals[i].target_pose.pose.position.x;
        goal.target_pose.pose.position.y = goals[i].target_pose.pose.position.y;
        goal.target_pose.pose.orientation.z = goals[i].target_pose.pose.orientation.z;

	    ROS_INFO("Sending goal -- %d",i);
        ac.sendGoal(goal);
        

        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The base reach goal -- %d",i);
        else
            ROS_INFO("The base failed to move for some reason");
    }
    ROS_INFO("Navgation done..");
    
    ros::spin();

    return 0;
}

