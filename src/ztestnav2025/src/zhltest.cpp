#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include "ztestnav2025/getpose_server.h"
#include "ztestnav2025/turn_detect.h"
#include "ztestnav2025/lidar_process.h"
#include "line_follow/line_follow.h"
#include "ztestnav2025/traffic_light.h"
#include "qr_01/qr_srv.h"
#include "communication/msg_1.h"
#include "communication/msg_2.h"

#include <cmath>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
void goal_set(move_base_msgs::MoveBaseGoal &goal,double x,double y,double yaw,tf2::Quaternion q){
    q.setRPY(0, 0, yaw);
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = q.x();
    goal.target_pose.pose.orientation.y = q.y();
    goal.target_pose.pose.orientation.z = q.z();
    goal.target_pose.pose.orientation.w = q.w();
}

void go_destination(move_base_msgs::MoveBaseGoal &goal,double x,double y,double yaw,tf2::Quaternion &q,MoveBaseClient &ac){
    goal.target_pose.header.stamp = ros::Time::now();
    goal_set(goal,x,y,yaw,q);
    ac.sendGoal(goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("到达目标");
    else
        ROS_INFO("无法到达目标");
}


int main(int argc, char *argv[]){
    setlocale(LC_ALL,"");
    std::map<int, std::string> name = {
        {0, "辣椒"},
        {1, "番茄"},
        {2, "土豆"},
        {3, "香蕉"},
        {4, "苹果"},
        {5, "西瓜"},
        {6, "可乐"},
        {7, "蛋糕"},
        {8, "牛奶"}
    };
    ROS_INFO("主干代码开始，初始化对象，等待服务中"); 
    //-----------------------------------初始化movebase，实例对象---------------------------//
    ros::init(argc,argv,"zhltest");
    ros::NodeHandle nh;
    MoveBaseClient ac("move_base", true); 
    tf2::Quaternion q;  
    //等待action回应
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("等待movebase服务中---");
    } 
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    //实例对象初始化
    MecanumController mecanumController(nh);
    //客户端初始化
    ROS_INFO("等待lidar_process服务中---");
    ros::ServiceClient client_find_board = nh.serviceClient<ztestnav2025::lidar_process>("/lidar_process/lidar_process");
    ztestnav2025::lidar_process where_board;
    client_find_board.waitForExistence();
    ROS_INFO("等待坐标获取服务中---");
    ros::ServiceClient poseget_client = nh.serviceClient<ztestnav2025::getpose_server>("getpose_server");
    ztestnav2025::getpose_server pose_result;
    pose_result.request.getpose_start = 1;
    poseget_client.waitForExistence();

    ROS_INFO("走廊环境导航开始");
    // mecanumController.rotateCircle(3.14,1,0.6);
    // mecanumController.rotateCircle(3.14,1,0.6);

    ROS_INFO("拣货区域任务开始");
    size_t board_count;
    int board_name;
    int flag=0;//判断雷达识别的点是否和视觉对得上
    double lidar_yaw;
    //视觉识别开始，先传个-1把摄像头打开
    std::vector<int> a = {-1,-1,-1,-1,-1,-1};
    mecanumController.detect(a,-1);
    // go_destination(goal,1.25,3.75,0,q,ac);
    // ROS_INFO("第一个找板点是否找到板子%d",flag);
    // if(!flag){
    //     board_name = mecanumController.turn_and_find(1,1,2,0.6);//请求视觉识别板子服务
    //     if (poseget_client.call(pose_result)){
    //         ROS_INFO("小车坐标xyz:%f,%f,%f",pose_result.response.pose_at[0],pose_result.response.pose_at[1],pose_result.response.pose_at[2]);
    //     }
    //     else{
    //         ROS_ERROR("获取位姿失败");
    //     }
    //     where_board.request.lidar_process_start = 1;//雷达获取前方障碍物距离
    //     if (client_find_board.call(where_board)){
    //         double target_x = (where_board.response.lidar_results[0]-0.6)*cos(pose_result.response.pose_at[2])+pose_result.response.pose_at[0];
    //         double target_y = (where_board.response.lidar_results[0]-0.6)*sin(pose_result.response.pose_at[2])+pose_result.response.pose_at[1];
    //         ROS_INFO("目的地%f,%f,%f",target_x,target_y,pose_result.response.pose_at[2]);
    //         go_destination(goal,target_x,target_y,pose_result.response.pose_at[2],q,ac);
    //     }
    //     // waitForContinue();
    //     // mecanumController.turn_and_find(1,1,2,0.3);
    //     waitForContinue();
    mecanumController.adjust(3,0.4);
    //     waitForContinue();
    //     if(mecanumController.forward(2,0.3)){//直接前进，直到目标检测框高超过230
    //         flag = 1;
    //     }
    //     if (!flag){
    //         ROS_INFO("找不到板子，直接走了");
    //     }
    //     mecanumController.cap_close();
    // }
}


// int main(int argc, char *argv[])
// {
//     setlocale(LC_ALL,"");
//     ros::init(argc,argv,"zhltest");
//     ros::NodeHandle nh;

//     MecanumController mecanumController(nh);

//     std::vector<int> a = {-1,-1,-1,-1,-1,-1};
//     mecanumController.detect(a,-1);
// restart:
//     mecanumController.turn_and_find(1,1,1,0.4);
//     ROS_INFO("结束了");
//     while(ros::ok()){
//         ros::spinOnce();
//         if(mecanumController.pid_change_flag == 1){
//             mecanumController.pid_change_flag = 0;
//             goto restart;
//         }
//     }
//     ros::spin();

//     return 0;
// }

// int main(int argc, char *argv[]){
//     setlocale(LC_ALL,"");
//     ros::init(argc,argv,"zhltest");
//     play_audio(voice[0][0]);
//     waitForContinue();
//     play_audio(voice[0][1]);
//     waitForContinue();
//     play_audio(voice[0][2]);
//     waitForContinue();
//     play_audio(voice[1][0]);
//     waitForContinue();
//     play_audio(voice[1][1]);
//     waitForContinue();
//     play_audio(voice[1][2]);
//     waitForContinue();
//     play_audio(voice[1][3]);
//     waitForContinue();
//     play_audio(voice[1][4]);
//     waitForContinue();
//     play_audio(voice[1][5]);
//     play_audio(voice[1][6]);
//     play_audio(voice[1][7]);
//     play_audio(voice[1][8]);
//     play_audio(voice[2][0]);
//     play_audio(voice[2][1]);
//     play_audio(voice[2][2]);
//     play_audio(voice[3][0]);
//     play_audio(voice[3][1]);
//     play_audio(cost[0]);
//     play_audio(cost[1]);
//     play_audio(cost[2]);
//     play_audio(cost[3]);
//     play_audio(cost[4]);
//     play_audio(cost[5]);
//     play_audio(cost[6]);
//     play_audio(cost[7]);
//     play_audio(cost[8]);
//     play_audio(cost[9]);
//     play_audio(cost[10]);
//     play_audio(cost[11]);
//     play_audio(cost[12]);
//     play_audio(cost[13]);
//     play_audio(cost[14]);
//     play_audio(cost[15]);
//     play_audio(cost[16]);
//     play_audio(cost[17]);
// }

