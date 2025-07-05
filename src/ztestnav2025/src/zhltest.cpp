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


int main(int argc, char *argv[])
{
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
    //第一点视觉识别
    // where_board.request.lidar_process_start = 1;//请求雷达识别板子服务
    // if (client_find_board.call(where_board)){
    //     size_t len_of_where_board = where_board.response.lidar_results.size();
    //     board_count = len_of_where_board / 4;
    //     ROS_INFO("找到%zu个板子",board_count);
    //     for(size_t i=0;i<board_count;i++){
    //         ROS_INFO("第%zu个板子位于%.2f,%.2f,%.2f",i,where_board.response.lidar_results[i*4],where_board.response.lidar_results[i*4+1],where_board.response.lidar_results[i*4+3] / where_board.response.lidar_results[i*4+2]);
    //     }
    // }
    // else{
    //     ROS_ERROR("找板服务请求失败");
    //     return 1;
    // }
    //视觉识别开始，先传个-1把摄像头打开
    std::vector<int> a = {-1,-1,-1,-1,-1,-1};
    mecanumController.detect(a,-1);
    // board_name = mecanumController.turn_and_find(1,1,board_class,0.4);//请求视觉识别板子服务
    // if (board_name>=0 && board_name<9) std::cout << name.at(board_name) << std::endl;
    // if(board_name != -1){
    //     if (poseget_client.call(pose_result)){
    //         ROS_INFO("小车坐标xyz:%f,%f,%f",pose_result.response.pose_at[0],pose_result.response.pose_at[1],pose_result.response.pose_at[2]);
    //     }
    //     else{
    //         ROS_ERROR("获取位姿失败");
    //     }

    //     for(int i=0;i<board_count;i++){
    //         lidar_yaw = std::atan2(where_board.response.lidar_results[i*4+1], where_board.response.lidar_results[i*4]);//计算雷达找到的板子在什么方向，是否和视觉识别结果匹配double atan2(double y, double x); 
    //         ROS_INFO("板子相对小车夹角%f",lidar_yaw);
    //         if (std::fabs(lidar_yaw-pose_result.response.pose_at[2])<0.3){
    //             flag = 1;
    //             float slope = where_board.response.lidar_results[i*4+3] / where_board.response.lidar_results[i*4+2];
    //             RobotPose robot = calculate_destination(where_board.response.lidar_results[i*4]+pose_result.response.pose_at[0],
                //                         where_board.response.lidar_results[i*4+1]+pose_result.response.pose_at[1],slope,
                //                         pose_result.response.pose_at[0],pose_result.response.pose_at[1]);//计算小车位姿
                //    ROS_INFO("第%d个板是目标板,即将前往，%.2f,%.2f,%.2f",i,robot.x,robot.y,robot.heading);
    //             waitForContinue();
    //             go_destination(goal,robot.x,robot.y,robot.heading,q,ac);
    //             break;
    //         }
    //     }
    // }

    ROS_INFO("第一个找板点是否找到板子%d",flag);
    if(!flag){
        //前往区域中心找板子
        ROS_INFO("前往中心找板");
        go_destination(goal,1.25,3.75,0,q,ac);
        where_board.request.lidar_process_start = 2;//请求雷达识别板子服务
        if (client_find_board.call(where_board)){
            size_t len_of_where_board = where_board.response.lidar_results.size();
            board_count = len_of_where_board / 4;
            ROS_INFO("找到%zu个板子",board_count);
            for(size_t i=0;i<board_count;i++){
                ROS_INFO("第%zu个板子位于%.2f,%.2f",i,where_board.response.lidar_results[i*4],where_board.response.lidar_results[i*4+1]);
            }
        }
        else{
            ROS_ERROR("找板服务请求失败");
            return 1;
        }
        board_name = mecanumController.turn_and_find(1,1,1);//请求视觉识别板子服务
        if (board_name>=0 && board_name<9) std::cout << name.at(board_name) << std::endl;
        if (board_name != -1){
            if (poseget_client.call(pose_result)){
                ROS_INFO("小车坐标xyz:%f,%f,%f",pose_result.response.pose_at[0],pose_result.response.pose_at[1],pose_result.response.pose_at[2]);
            }
            else{
                ROS_ERROR("获取位姿失败");
            }
            for(int i=0;i<board_count;i++){
                lidar_yaw = std::atan2(where_board.response.lidar_results[i*4+1], where_board.response.lidar_results[i*4]);//计算雷达找到的板子在什么方向，是否和视觉识别结果匹配double atan2(double y, double x); 
                ROS_INFO("板子相对小车夹角%f",lidar_yaw);
                if (std::fabs(lidar_yaw-pose_result.response.pose_at[2])<0.3){
                    flag = 1;
                    float slope = where_board.response.lidar_results[i*4+3] / where_board.response.lidar_results[i*4+2];
                    RobotPose robot = calculate_destination(where_board.response.lidar_results[i*4]+pose_result.response.pose_at[0],
                                        where_board.response.lidar_results[i*4+1]+pose_result.response.pose_at[1],slope,
                                        pose_result.response.pose_at[0],pose_result.response.pose_at[1]);//计算小车位姿
                    ROS_INFO("第%d个板是目标板,即将前往，%.2f,%.2f,%.2f",i,robot.x,robot.y,robot.heading);
                    waitForContinue();
                    go_destination(goal,robot.x,robot.y,robot.heading,q,ac);
                    break;
                }
            }
        }
    }
    if (!flag){
        ROS_INFO("找不到板子，直接走了");
    }
    mecanumController.cap_close();

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


