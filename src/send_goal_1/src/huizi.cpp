 /*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-03-29 10:19:39
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-03-29 12:19:55
 * @FilePath: /dhlearn_ws/src/send_goal/src/huizi.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
 
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
 
int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
 
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
 
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
 
  move_base_msgs::MoveBaseGoal goal[3];
 
  //we'll send a goal to the robot to move 1 meter forward
  
goal[0].target_pose.pose.position.x = 1.859;
  goal[0].target_pose.pose.position.y = 0.086;
  goal[0].target_pose.pose.position.z = 0.000;
  goal[0].target_pose.pose.orientation.x = 0.000;
  goal[0].target_pose.pose.orientation.y = 0.000;
  goal[0].target_pose.pose.orientation.z = 0.714;
  goal[0].target_pose.pose.orientation.w = 0.700;
 
goal[1].target_pose.pose.position.x = 1.048;
  goal[1].target_pose.pose.position.y = 1.019;
  goal[1].target_pose.pose.position.z = 0.000;
  goal[1].target_pose.pose.orientation.x = 0.000;
  goal[1].target_pose.pose.orientation.y = 0.000;
  goal[1].target_pose.pose.orientation.z = -1.000;
  goal[1].target_pose.pose.orientation.w = 0.018;

goal[2].target_pose.pose.position.x = -0.053;
  goal[2].target_pose.pose.position.y = -0.039;
  goal[2].target_pose.pose.position.z = 0.000;
  goal[2].target_pose.pose.orientation.x = 0.000;
  goal[2].target_pose.pose.orientation.y = 0.000;
  goal[2].target_pose.pose.orientation.z = 0.029;
  goal[2].target_pose.pose.orientation.w = 1.000;
int i;
for(i=0;i<3;i++){
  goal[i].target_pose.header.frame_id = "base_link";
  goal[i].target_pose.header.stamp = ros::Time::now();
  ROS_INFO("Sending goal");
  ac.sendGoal(goal[i]);
 
  ac.waitForResult();
 
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

} 
  return 0;
}
