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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Sim_talkto_car{
public:
    Sim_talkto_car(ros::NodeHandle& nh):
        nh_(nh),
        pub_(nh.advertise<communication::msg_1>("send_class", 10)),
        sub_(nh.subscribe<communication::msg_2>("send_room_class",10,&Sim_talkto_car::simreturn, this))
    {}

    bool sim_done = 0;
    int sim_room = -1;
    int sim_detect_class = -1;
    void simreturn(const communication::msg_2::ConstPtr& sim_result){
        ROS_INFO("房间编号：%d,检测类型：%d", sim_result->room, sim_result->detected_class);
        sim_room = sim_result->room;
        sim_detect_class = sim_result->detected_class;
        sim_done = 1;
        return;
    }
    
    void car_msg_publish(int target){
        target_msg_.target_class = target;
        pub_.publish(target_msg_);
    }
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    communication::msg_1 target_msg_;
    ros::Subscriber sub_;
};

void init_goals( std::vector<move_base_msgs::MoveBaseGoal> &goals, tf2::Quaternion q)
{

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
    ROS_INFO("主干代码开始，初始化对象，等待服务中"); 
    //-----------------------------------初始化movebase，实例对象---------------------------//
    ros::init(argc,argv,"zhltest");
    ros::NodeHandle nh;
    MoveBaseClient ac("move_base", true); 
    tf2::Quaternion q;
    std::vector<move_base_msgs::MoveBaseGoal> goals(2);
    init_goals(goals,q);    
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
    ros::ServiceClient client_find_board = nh.serviceClient<ztestnav2025::lidar_process>("/lidar_process/lidar_process");
    ztestnav2025::lidar_process where_board;
    client_find_board.waitForExistence();
    ROS_INFO("等待二维码服务中---");
    ros::ServiceClient client_qr = nh.serviceClient<qr_01::qr_srv>("qr_detect");
    qr_01::qr_srv what_qr;
    int board_class = 0;
    client_qr.waitForExistence();
    ROS_INFO("等待坐标获取服务中---");
    ros::ServiceClient poseget_client = nh.serviceClient<ztestnav2025::getpose_server>("getpose_server");
    ztestnav2025::getpose_server pose_result;
    pose_result.request.getpose_start = 1;
    poseget_client.waitForExistence();
    ROS_INFO("等待视觉巡线服务中---");
    ros::ServiceClient line_client = nh.serviceClient<line_follow::line_follow>("line_server");
    line_follow::line_follow linefollow_start;
    linefollow_start.request.line_follow_start = 1;
    line_client.waitForExistence();
    //发布话题以供仿真通信
    Sim_talkto_car sim_talkto_car(nh);

    //--------------------------------------语音唤醒等待--------------------------------//



    //--------------------------------------走廊环境导航，发布目标点--------------------------------//
    ROS_INFO("走廊环境导航开始");
    mecanumController.rotateCircle(3.14,1,0.4);
    mecanumController.rotateCircle(3.14,1,0.4);
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
        if(i==0){//请求二维码识别服务
            ros::Duration(1).sleep();
            what_qr.request.qr_start = 1;
            if (client_qr.call(what_qr)){
                board_class = what_qr.response.qr_result;
                ROS_INFO("二维码结果:%d",what_qr.response.qr_result);
            }
            else{
                ROS_ERROR("请求二维码失败");
            }
        }
    }
    ROS_INFO("走廊环境导航完成");


    //----------------------------------------目标检测区域开始-------------------------------------------//
    ROS_INFO("拣货区域任务开始");
    size_t board_count;
    where_board.request.lidar_process_start = 1;//请求雷达识别板子服务
    if (client_find_board.call(where_board)){
        size_t len_of_where_board = where_board.response.lidar_results.size();
        board_count = len_of_where_board / 6;
        ROS_INFO("找到%zu个板子",board_count);
        for(size_t i=0;i<board_count;i++){
            ROS_INFO("第%zu个板子位于%.2f,%.2f",i,where_board.response.lidar_results[i*6],where_board.response.lidar_results[i*6+1]);
        }
    }
    else{
        ROS_ERROR("找板服务请求失败");
        return 1;
    }
    //视觉识别开始，先传个-1把摄像头打开
    std::vector<int> a = {-1,-1,-1,-1,-1,-1};
    mecanumController.detect(a,-1);
    mecanumController.turn_and_find(1,1,board_class);//请求视觉识别板子服务
    if (poseget_client.call(pose_result)){
        ROS_INFO("小车坐标xyz:%f,%f,%f",pose_result.response.pose_at[0],pose_result.response.pose_at[1],pose_result.response.pose_at[2]);
    }
    else{
        ROS_ERROR("获取位姿失败");
    }
    mecanumController.detect(a,0);//把摄像头关了
    bool flag=0;//判断雷达识别的点是否和视觉对得上
    double lidar_yaw;
    for(int i=0;i<board_count;i++){
        lidar_yaw = std::atan2(where_board.response.lidar_results[i*6+1], where_board.response.lidar_results[i*6]);//计算雷达找到的板子在什么方向，是否和视觉识别结果匹配double atan2(double y, double x); 
        if (std::fabs(lidar_yaw-pose_result.response.pose_at[2]<0.08)){
            flag = 1;
            float slope = where_board.response.lidar_results[i*6+3] / where_board.response.lidar_results[i*6+2];
            RobotPose robot = calculate_destination(where_board.response.lidar_results[i*6],where_board.response.lidar_results[i*6+1],slope);//计算小车位姿
            q.setRPY(0, 0, robot.heading);
            goal.target_pose.pose.position.x = robot.x+pose_result.response.pose_at[0];
            goal.target_pose.pose.position.y = robot.y+pose_result.response.pose_at[1];
            goal.target_pose.pose.position.z = 0.0;
            goal.target_pose.pose.orientation.x = q.x();
            goal.target_pose.pose.orientation.y = q.y();
            goal.target_pose.pose.orientation.z = q.z();
            goal.target_pose.pose.orientation.w = q.w();
            ROS_INFO("第%d个板是目标板,即将前往，%.2f,%.2f,%.2f",i,robot.x+pose_result.response.pose_at[0],robot.y+pose_result.response.pose_at[1],robot.heading);
        }
    }
    if(flag){
        ac.sendGoal(goal);
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("到达板前");
        else
            ROS_INFO("无法到达板前");
    }
    else{
        ROS_INFO("视觉和雷达信息对不上");
    }
    mecanumController.cap_close();


    //-----------------------------------------------仿真开始--------------------------------------------//
    ROS_INFO("前往仿真区域");
    goal.target_pose.header.stamp = ros::Time::now();
    q.setRPY(0, 0, 0);
    goal.target_pose.pose.position.x = 1.25;
    goal.target_pose.pose.position.y = 3.75;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = q.x();
    goal.target_pose.pose.orientation.y = q.y();
    goal.target_pose.pose.orientation.z = q.z();
    goal.target_pose.pose.orientation.w = q.w();
    ac.sendGoal(goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("到达仿真区域");
    else
        ROS_INFO("无法到达仿真区域");
    //发送仿真消息
    ros::Rate rate(1);
    while (ros::ok()) {
        sim_talkto_car.car_msg_publish(board_class);
        ROS_INFO("发布目标类别: %d", board_class);
        rate.sleep();
        ros::spinOnce();
        if(sim_talkto_car.sim_done==1){
            break;
        }
    }

    //--------------------------------------------前往红绿灯识别区域--------------------------------------------//
    q.setRPY(0, 0, -1.57);
    goal.target_pose.pose.position.x = 2.75;
    goal.target_pose.pose.position.y = 3.45;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = q.x();
    goal.target_pose.pose.orientation.y = q.y();
    goal.target_pose.pose.orientation.z = q.z();
    goal.target_pose.pose.orientation.w = q.w();
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("到达视觉巡线区域");
    else
        ROS_INFO("无法到达视觉巡线区域");

    //-----------------------------------------视觉巡线---------------------------------------------//
    if(line_client.call(linefollow_start)){
        ROS_INFO("视觉巡线结束");
    }
    else{
        ROS_ERROR("视觉巡线失败....");
    }

    ros::spin();

    return 0;
}

