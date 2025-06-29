#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "nav_client");
    MoveBaseClient ac("move_base", true);
    int i = 0;
    // 等待导航启动
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    //坐标系为map
    goal.target_pose.header.frame_id = "map";
    // 时间戳
    goal.target_pose.header.stamp = ros::Time::now();
    // 目标位置
    goal.target_pose.pose.position.x = 4.03508996963501;
    goal.target_pose.pose.position.y = 1.2556221961975098;
    goal.target_pose.pose.position.z = 0.0; 
    // 目标方向
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.7071;
    goal.target_pose.pose.orientation.w = 0.7071;

    ROS_INFO("Sending goal_1");
    // 发送目标位置
    ac.sendGoal(goal);
    // 等待结果
    ac.waitForResult();
    // 判断结果
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Goal_1 reached");
        i = 2;
    }
    else
    {
        ROS_INFO("The base failed to move to goal_1");
    }
    
    if(i == 2)
    {
        // 目标位置
        goal.target_pose.pose.position.x = 2.727267026901245;
        goal.target_pose.pose.position.y = 1.514411392211914;
        goal.target_pose.pose.position.z = 0.0; 
        // 目标方向
        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = 0.7071;
        goal.target_pose.pose.orientation.w = 0.7071;

        ROS_INFO("Sending goal_3");
        // 发送目标位置
        ac.sendGoal(goal);
        // 等待结果
        ac.waitForResult();
        // 判断结果
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Goal_3 reached");
            i = 4;
        }
        else
        {
            ROS_INFO("The base failed to move to goal_3");
        }
    }
    
    if(i == 4)
    {
        // 目标位置
        goal.target_pose.pose.position.x = 0.9367432832717896;
        goal.target_pose.pose.position.y = 1.6391351699829102;
        goal.target_pose.pose.position.z = 0.0; 
        // 目标方向
        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = 0.7071;
        goal.target_pose.pose.orientation.w = 0.7071;

        ROS_INFO("Sending goal_5");
        // 发送目标位置
        ac.sendGoal(goal);
        // 等待结果
        ac.waitForResult();
        // 判断结果
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Goal_5 reached");
            i = 8;
        }
        else
        {
            ROS_INFO("The base failed to move to goal_5");
        }
    }
    
    
    if(i == 8)
    {
        // 目标位置
        goal.target_pose.pose.position.x = 0.2;
        goal.target_pose.pose.position.y = 0;
        goal.target_pose.pose.position.z = 0.0; 
            // 目标方向
        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = 0;
        goal.target_pose.pose.orientation.w = 1;

        ROS_INFO("Sending goal_9");
            // 发送目标位置
        ac.sendGoal(goal);
            // 等待结果
        ac.waitForResult();
            // 判断结果
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Goal_9 reached");
            i = 10;
        }
        else
        {
            ROS_INFO("The base failed to move to goal_9");
        }
    }
        
    

    return 0;
}
 