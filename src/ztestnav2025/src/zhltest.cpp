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

    q.setRPY(0, 0, 1.57);
    goals[2].target_pose.pose.position.x = 1.75;
    goals[2].target_pose.pose.position.y = 0.77;
    goals[2].target_pose.pose.position.z = 0.0;
    goals[2].target_pose.pose.orientation.x = q.x();
    goals[2].target_pose.pose.orientation.y = q.y();
    goals[2].target_pose.pose.orientation.z = q.z();    
    goals[2].target_pose.pose.orientation.w = q.w();

    q.setRPY(0, 0, 3.14);
    goals[3].target_pose.pose.position.x = 1.75;
    goals[3].target_pose.pose.position.y = 0.77;
    goals[3].target_pose.pose.position.z = 0.0;
    goals[3].target_pose.pose.orientation.x = q.x();
    goals[3].target_pose.pose.orientation.y = q.y();
    goals[3].target_pose.pose.orientation.z = q.z();    
    goals[3].target_pose.pose.orientation.w = q.w();

    q.setRPY(0, 0, 3.14);
    goals[4].target_pose.pose.position.x = 0.29;
    goals[4].target_pose.pose.position.y = 0.77;
    goals[4].target_pose.pose.position.z = 0.0;
    goals[4].target_pose.pose.orientation.x = q.x();
    goals[4].target_pose.pose.orientation.y = q.y();
    goals[4].target_pose.pose.orientation.z = q.z();    
    goals[4].target_pose.pose.orientation.w = q.w();
    
    q.setRPY(0, 0, 1.57);
    goals[5].target_pose.pose.position.x = 0.29;
    goals[5].target_pose.pose.position.y = 0.77;
    goals[5].target_pose.pose.position.z = 0.0;
    goals[5].target_pose.pose.orientation.x = q.x();
    goals[5].target_pose.pose.orientation.y = q.y();
    goals[5].target_pose.pose.orientation.z = q.z();    
    goals[5].target_pose.pose.orientation.w = q.w();

    q.setRPY(0, 0, 1.57);
    goals[6].target_pose.pose.position.x = 0.29;
    goals[6].target_pose.pose.position.y = 0.77;
    goals[6].target_pose.pose.position.z = 0.0;
    goals[6].target_pose.pose.orientation.x = q.x();
    goals[6].target_pose.pose.orientation.y = q.y();
    goals[6].target_pose.pose.orientation.z = q.z();    
    goals[6].target_pose.pose.orientation.w = q.w();

    q.setRPY(0, 0, 0);
    goals[7].target_pose.pose.position.x = 0.75;
    goals[7].target_pose.pose.position.y = 1.26;
    goals[7].target_pose.pose.position.z = 0.0;
    goals[7].target_pose.pose.orientation.x = q.x();
    goals[7].target_pose.pose.orientation.y = q.y();
    goals[7].target_pose.pose.orientation.z = q.z();    
    goals[7].target_pose.pose.orientation.w = q.w();

    q.setRPY(0, 0, 0);
    goals[8].target_pose.pose.position.x = 1.75;
    goals[8].target_pose.pose.position.y = 1.26;
    goals[8].target_pose.pose.position.z = 0.0;
    goals[8].target_pose.pose.orientation.x = q.x();
    goals[8].target_pose.pose.orientation.y = q.y();
    goals[8].target_pose.pose.orientation.z =  q.z();    
    goals[8].target_pose.pose.orientation.w =  q.w();

    q.setRPY(0, 0, 1.57);
    goals[9].target_pose.pose.position.x = 1.75;
    goals[9].target_pose.pose.position.y = 1.26;
    goals[9].target_pose.pose.position.z = 0.0;
    goals[9].target_pose.pose.orientation.x = q.x();
    goals[9].target_pose.pose.orientation.y = q.y();
    goals[9].target_pose.pose.orientation.z =  q.z();    
    goals[9].target_pose.pose.orientation.w =  q.w();

    q.setRPY(0, 0, 1.57);
    goals[10].target_pose.pose.position.x = 1.75;
    goals[10].target_pose.pose.position.y = 1.77;
    goals[10].target_pose.pose.position.z = 0.0;
    goals[10].target_pose.pose.orientation.x = q.x();
    goals[10].target_pose.pose.orientation.y =  q.y();
    goals[10].target_pose.pose.orientation.z = q.z();    
    goals[10].target_pose.pose.orientation.w = q.w();

    q.setRPY(0, 0, 3.14);
    goals[11].target_pose.pose.position.x = 1.75;
    goals[11].target_pose.pose.position.y = 1.77;
    goals[11].target_pose.pose.position.z = 0.0;
    goals[11].target_pose.pose.orientation.x = q.x();
    goals[11].target_pose.pose.orientation.y =  q.y();
    goals[11].target_pose.pose.orientation.z = q.z();    
    goals[11].target_pose.pose.orientation.w = q.w();

    q.setRPY(0, 0, 3.14);
    goals[12].target_pose.pose.position.x = 0.5;
    goals[12].target_pose.pose.position.y = 1.77;
    goals[12].target_pose.pose.position.z = 0.0;
    goals[12].target_pose.pose.orientation.x = q.x();
    goals[12].target_pose.pose.orientation.y = q.y();
    goals[12].target_pose.pose.orientation.z = q.z();    
    goals[12].target_pose.pose.orientation.w = q.w();

    q.setRPY(0, 0, 1.57);
    goals[13].target_pose.pose.position.x = 0.5;
    goals[13].target_pose.pose.position.y = 1.77;
    goals[13].target_pose.pose.position.z = 0.0;
    goals[13].target_pose.pose.orientation.x = q.x();
    goals[13].target_pose.pose.orientation.y = q.y();
    goals[13].target_pose.pose.orientation.z = q.z();    
    goals[13].target_pose.pose.orientation.w = q.w();

    q.setRPY(0, 0, 1.57);
    goals[14].target_pose.pose.position.x = 0.5;
    goals[14].target_pose.pose.position.y = 3.07;
    goals[14].target_pose.pose.position.z = 0.0;
    goals[14].target_pose.pose.orientation.x = q.x();
    goals[14].target_pose.pose.orientation.y = q.y();
    goals[14].target_pose.pose.orientation.z = q.z();    
    goals[14].target_pose.pose.orientation.w = q.w();

    q.setRPY(0, 0,1.57);
    goals[15].target_pose.pose.position.x = 0.5;
    goals[15].target_pose.pose.position.y = 3.07;
    goals[15].target_pose.pose.position.z = 0.0;
    goals[15].target_pose.pose.orientation.x = q.x();
    goals[15].target_pose.pose.orientation.y = q.y();
    goals[15].target_pose.pose.orientation.z = q.z();    
    goals[15].target_pose.pose.orientation.w = q.w();

    q.setRPY(0, 0, 0);
    goals[16].target_pose.pose.position.x = 2.25;
    goals[16].target_pose.pose.position.y = 4.25;
    goals[16].target_pose.pose.position.z = 0.0;
    goals[16].target_pose.pose.orientation.x = q.x();
    goals[16].target_pose.pose.orientation.y = q.y();
    goals[16].target_pose.pose.orientation.z = q.z();    
    goals[16].target_pose.pose.orientation.w = q.w();

    q.setRPY(0, 0, 0);
    goals[17].target_pose.pose.position.x = 2.25;
    goals[17].target_pose.pose.position.y = 4.25;
    goals[17].target_pose.pose.position.z = 0.0;
    goals[17].target_pose.pose.orientation.x = q.x();
    goals[17].target_pose.pose.orientation.y = q.y();
    goals[17].target_pose.pose.orientation.z = q.z();    
    goals[17].target_pose.pose.orientation.w = q.w();

    q.setRPY(0, 0, 0);
    goals[18].target_pose.pose.position.x = 2.75;
    goals[18].target_pose.pose.position.y = 4.25;
    goals[18].target_pose.pose.position.z = 0.0;
    goals[18].target_pose.pose.orientation.x = q.x();
    goals[18].target_pose.pose.orientation.y =  q.y();
    goals[18].target_pose.pose.orientation.z = q.z();    
    goals[18].target_pose.pose.orientation.w = q.w();

    q.setRPY(0, 0, -1.57);
    goals[19].target_pose.pose.position.x = 2.75;
    goals[19].target_pose.pose.position.y = 4.25;
    goals[19].target_pose.pose.position.z = 0.0;
    goals[19].target_pose.pose.orientation.x = q.x();
    goals[19].target_pose.pose.orientation.y =  q.y();
    goals[19].target_pose.pose.orientation.z = q.z();    
    goals[19].target_pose.pose.orientation.w = q.w();

    q.setRPY(0, 0, -1.57);
    goals[20].target_pose.pose.position.x = 2.75;
    goals[20].target_pose.pose.position.y = 2.17;
    goals[20].target_pose.pose.position.z = 0.0;
    goals[20].target_pose.pose.orientation.x = q.x();
    goals[20].target_pose.pose.orientation.y = q.y();
    goals[20].target_pose.pose.orientation.z = q.z();    
    goals[20].target_pose.pose.orientation.w = q.w();

    q.setRPY(0, 0, -0.75);
    goals[21].target_pose.pose.position.x = 2.78;
    goals[21].target_pose.pose.position.y = 2.17;
    goals[21].target_pose.pose.position.z = 0.0;
    goals[21].target_pose.pose.orientation.x = q.x();
    goals[21].target_pose.pose.orientation.y = q.y();
    goals[21].target_pose.pose.orientation.z = q.z();    
    goals[21].target_pose.pose.orientation.w = q.w();

    q.setRPY(0, 0, -0.75);
    goals[22].target_pose.pose.position.x = 3.77;
    goals[22].target_pose.pose.position.y = 0.59;
    goals[22].target_pose.pose.position.z = 0.0;
    goals[22].target_pose.pose.orientation.x = q.x();
    goals[22].target_pose.pose.orientation.y = q.y();
    goals[22].target_pose.pose.orientation.z = q.z();    
    goals[22].target_pose.pose.orientation.w = q.w();
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"zhltest");
    ros::NodeHandle nh;

    std::vector<move_base_msgs::MoveBaseGoal> goals(12);
    init_goals(goals);

    MoveBaseClient ac("move_base", true); 
    
    //等待action回应
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
    ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    
    for(int i=0;i<23;i++)        //循环次数为目标点个数，发布目标点
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

