#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include "ztestnav2025/getpose_server.h"
#include "ztestnav2025/turn_detect.h"
#include "ztestnav2025/lidar_process.h"
#include "ztestnav2025/traffic_light.h"
#include "line_follow/line_follow.h"
#include "qr_01/qr_srv.h"
#include "communication/msg_1.h"
#include "communication/msg_2.h"

#include <cmath>
// 全局变量定义
int room_index = 0;       // 当前房间号
int awake_flag = 0;      // 语音唤醒标志位

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
    waitForContinue();


    //--------------------------------------走廊环境导航，发布目标点--------------------------------//
    ROS_INFO("走廊环境导航开始");
    mecanumController.rotateCircle(3.14,1,0.6);
    mecanumController.rotateCircle(3.14,1,0.6);
    go_destination(goal,1.25,0.75,3.14,q,ac);
    ros::Duration(0.5).sleep();
    what_qr.request.qr_start = 1;
    while(ros::ok()){
        if (!client_qr.call(what_qr)){
            ROS_INFO("没有请求到服务");
        }
        board_class = what_qr.response.qr_result;
        ROS_INFO("二维码结果:%d",board_class);
        if (board_class>0){
            
            ROS_INFO("二维码结果:%d",what_qr.response.qr_result);
            break;
        }
        else{
            ROS_ERROR("请求二维码失败");
        }
    }
    // board_class 为 1（蔬菜）、2（水果）、3（甜品）
    // if (board_class >= 1 && board_class <= 3  ) {
    //     play_audio(voice[0][board_class-1]);
    // }

    go_destination(goal,0.50,2.25,1.57,q,ac);
    ROS_INFO("走廊环境导航完成");


    //----------------------------------------目标检测区域开始-------------------------------------------//
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
    //             RobotPose robot = calculate_destination(where_board.response.lidar_results[i*4],where_board.response.lidar_results[i*4+1],slope);//计算小车位姿
    //             ROS_INFO("第%d个板是目标板,即将前往，%.2f,%.2f,%.2f",i,robot.x+pose_result.response.pose_at[0],robot.y+pose_result.response.pose_at[1],robot.heading);
    //             waitForContinue();
    //             go_destination(goal,robot.x+pose_result.response.pose_at[0],robot.y+pose_result.response.pose_at[1],robot.heading,q,ac);
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
        board_name = mecanumController.turn_and_find(1,1,board_class);//请求视觉识别板子服务
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
                    RobotPose robot = calculate_destination(where_board.response.lidar_results[i*4],where_board.response.lidar_results[i*4+1],slope);//计算小车位姿
                    ROS_INFO("第%d个板是目标板,即将前往，%.2f,%.2f,%.2f",i,robot.x+pose_result.response.pose_at[0],robot.y+pose_result.response.pose_at[1],robot.heading);
                    waitForContinue();
                    go_destination(goal,robot.x+pose_result.response.pose_at[0],robot.y+pose_result.response.pose_at[1],robot.heading,q,ac);
                    break;
                }
            }
        }
    }
    if (!flag){
        ROS_INFO("找不到板子，直接走了");
    }
    mecanumController.cap_close();
    // if (board_name >= 0 && board_name <= 9 && flag==1) {
    //     play_audio(voice[1][board_name]);
    // }

    //-----------------------------------------------仿真开始--------------------------------------------//
    ROS_INFO("前往仿真区域");
    go_destination(goal,1.25,3.75,0.0,q,ac);
    // //发送仿真消息
    // ros::Rate rate(1);
    // while (ros::ok()) {
    //     sim_talkto_car.car_msg_publish(board_class);
    //     rate.sleep();
    //     ros::spinOnce();
    //     if(sim_talkto_car.sim_done==1){
    //         break;
    //     }
    // }

    //--------------------------------------------前往红绿灯识别区域--------------------------------------------//
    ROS_INFO("前往红绿灯区域路口1");
    go_destination(goal,3.25,4.50,1.57,q,ac);
    if (detectTrafficLightStatus()==2){
        ROS_INFO("路口1可通过");
        go_destination(goal,2.75,3.50,-0.78,q,ac);
    } 
    else {
        ROS_INFO("前往红绿灯区域路口2");
        go_destination(goal,4.25,4.50,1.57,q,ac);
        if (detectTrafficLightStatus()==2){
            ROS_INFO("路口2可通过");
            go_destination(goal,4.75,3.50,-2.31,q,ac);
        } 
    }
    while(1);

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

