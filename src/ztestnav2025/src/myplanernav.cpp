#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include "ztestnav2025/getpose_server.h"
#include "ztestnav2025/turn_detect.h"
#include "ztestnav2025/lidar_process.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void init_goals( std::vector<move_base_msgs::MoveBaseGoal> &goals)
{
    tf2::Quaternion q;
    
    q.setRPY(0, 0, 3.14);
    goals[0].target_pose.pose.position.x = 1.25;
    goals[0].target_pose.pose.position.y = 0.75;
    goals[0].target_pose.pose.position.z = 0.0;
    goals[0].target_pose.pose.orientation.x = q.x();
    goals[0].target_pose.pose.orientation.y = q.y();
    goals[0].target_pose.pose.orientation.z = q.z();    
    goals[0].target_pose.pose.orientation.w = q.w();

    q.setRPY(0, 0, 1.57);
    goals[1].target_pose.pose.position.x = 0.50;
    goals[1].target_pose.pose.position.y = 2.25;
    goals[1].target_pose.pose.position.z = 0.0;
    goals[1].target_pose.pose.orientation.x = q.x();
    goals[1].target_pose.pose.orientation.y = q.y();
    goals[1].target_pose.pose.orientation.z = q.z();    
    goals[1].target_pose.pose.orientation.w = q.w();

}

int main(int argc, char *argv[])
{
    ROS_INFO("主干代码开始，初始化对象，等待服务中");
    setlocale(LC_ALL,"");
    //-------------初始化movebase，实例对象------------------//
    ros::init(argc,argv,"zhltest");
    ros::NodeHandle nh;
    MoveBaseClient ac("move_base", true); 
    std::vector<move_base_msgs::MoveBaseGoal> goals(12);
    init_goals(goals);    
    //等待action回应
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("等待movebase服务中---");
    } 
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    //实例对象初始化
    MecanumController mecanumController(nh);
    //客户端初始化
    ROS_INFO("等待lidar_process服务中---");
    ros::ServiceClient client_find_board = nh.serviceClient<ztestnav2025::lidar_process>("lidar_process");
    ztestnav2025::lidar_process where_board;
    client_find_board.waitForExistence();
    

    //--------------------------------------走廊环境导航，发布目标点--------------------------------//
    ROS_INFO("走廊环境导航开始");
    for(int i=0;i<2;i++){        //循环次数为目标点个数，发布目标点
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = goals[i].target_pose.pose.position.x;
        goal.target_pose.pose.position.y = goals[i].target_pose.pose.position.y;
        goal.target_pose.pose.position.z = goals[i].target_pose.pose.position.z;
        goal.target_pose.pose.orientation.x = goals[i].target_pose.pose.orientation.x;
        goal.target_pose.pose.orientation.y = goals[i].target_pose.pose.orientation.y;
        goal.target_pose.pose.orientation.z = goals[i].target_pose.pose.orientation.z;
        goal.target_pose.pose.orientation.w = goals[i].target_pose.pose.orientation.w;

	    ROS_INFO("走廊环境导航，前往第%d个目标",i);
        ac.sendGoal(goal);

        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("到达第%d个目标",i);
        else
            ROS_INFO("无法到达%d目标",i);
        if(i==0){

        }
    }
    ROS_INFO("走廊环境导航完成");


    //--------------------------------------目标检测区域开始--------------------------------//
    ROS_INFO("拣货区域任务开始");
    where_board.request.lidar_process_start = 1;
    if (client_find_board.call(where_board)){
        size_t len_of_where_board = where_board.response.lidar_results.size();
        size_t board_count = len_of_where_board / 6;
        ROS_INFO("找到%zu个板子",board_count);
        for(size_t i=0;i<board_count;i++){
            ROS_INFO("第%zu个板子位于%.2f,%.2f",i,where_board.response.lidar_results[i*6],where_board.response.lidar_results[i*6+1]);
        }
    }
    else{
        ROS_ERROR("找板服务请求失败");
        return 1;
    }
    mecanumController.turn_and_find(1,1,0);









    ros::spin();

    return 0;
}

