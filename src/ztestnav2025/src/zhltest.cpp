#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include "ztestnav2025/getpose_server.h"
#include "ztestnav2025/turn_detect.h"
#include "ztestnav2025/lidar_process.h"
#include "line_follow/line_follow.h"
#include "qr_01/qr_srv.h"
#include "communication/msg_1.h"
#include "communication/msg_2.h"

#include <cmath>

// 定义结构体存储机器人的位置和朝向
struct RobotPose {
    double x;        // 机器人的x坐标
    double y;        // 机器人的y坐标
    double heading;  // 机器人的朝向角度（度）
};

// 计算机器人目标位置和朝向
// 输入: 
//   cx, cy - 板子的中心坐标
//   slope - 板子的斜率
//   square_size - 正方形区域尺寸 (默认2.5m)
// 返回: RobotPose结构体包含目标位置和朝向
RobotPose calculate_destination(double cx, double cy, double slope, double square_size = 2.5){//利用雷达数据计算机器人拣货区的目的地，板前定位
    const double center_x = square_size / 2.0;
    const double center_y = square_size / 2.0;
    const double distance = 0.3;  // 30cm

    // 计算指向正方形中心的向量
    const double cp_x = center_x - cx;
    const double cp_y = center_y - cy;

    // 计算两个可能的法向量
    double nx, ny;
    const double dot_product = (-slope) * cp_x + 1.0 * cp_y;
    
    // 计算法向量的模长
    const double norm_length = std::sqrt(slope * slope + 1.0);
    
    // 根据点积选择正确的法线方向
    if (dot_product < 0) {
        // 选择法向量 (-m, 1)
        nx = -slope / norm_length;
        ny = 1.0 / norm_length;
    } else {
        // 选择法向量 (m, -1)
        nx = slope / norm_length;
        ny = -1.0 / norm_length;
    }

    // 计算目标位置
    RobotPose pose;
    pose.x = cx - distance * nx;
    pose.y = cy - distance * ny;
    
    // 计算朝向角度（atan2返回弧度，转换为度）
    double angle_rad = std::atan2(ny, nx);
    pose.heading = angle_rad * 180.0 / M_PI;
    
    // 将角度转换为0-360°范围
    if (pose.heading < 0) {
        pose.heading += 360.0;
    }

    return pose;
}


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"zhltest");
    ros::NodeHandle nh;

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

    ROS_INFO("拣货区域任务开始");
    size_t board_count;
    where_board.request.lidar_process_start = 1;//请求雷达识别板子服务
    if (client_find_board.call(where_board)){
        size_t len_of_where_board = where_board.response.lidar_results.size();
        ROS_INFO("%zu长度",len_of_where_board);
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
    ROS_INFO("测试");
    bool flag=0;//判断雷达识别的点是否和视觉对得上
    double lidar_yaw;
    for(int i=0;i<board_count;i++){
        lidar_yaw = std::atan2(where_board.response.lidar_results[i*4+1], where_board.response.lidar_results[i*4]);//计算雷达找到的板子在什么方向，是否和视觉识别结果匹配double atan2(double y, double x); 
        ROS_INFO("第%f板子角度",lidar_yaw);
        ROS_INFO("第%f误差",std::fabs(lidar_yaw-1.57));
        if (std::fabs(lidar_yaw-1.57)<0.3){
            flag = 1;
            float slope = where_board.response.lidar_results[i*4+3] / where_board.response.lidar_results[i*4+2];
            RobotPose robot = calculate_destination(where_board.response.lidar_results[i*4],where_board.response.lidar_results[i*4+1],slope);//计算小车位姿
            ROS_INFO("第%d个板是目标板,即将前往，%.2f,%.2f,%.2f",i,robot.x+0.5,robot.y+2.25,robot.heading);
        }
    }

    return 0;
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


