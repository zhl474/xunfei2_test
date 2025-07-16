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
#include <std_msgs/Int8.h>

#include <cmath>
//找板优化新增头文件
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// 全局变量定义
int room_index = 0;       // 当前房间号
int awake_flag = 0;      // 语音唤醒标志位

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class AwakeDetector {
public:
    AwakeDetector(ros::NodeHandle& nh) : nh_(nh), awake_received_(false) {
        // 订阅awake_flag话题
        sub_ = nh_.subscribe("/awake_flag", 10, &AwakeDetector::awakeCallback, this);
    }

    // 等待唤醒信号
    bool waitForAwake() {
        ROS_INFO("等待语音唤醒信号...");
        ros::Rate rate(10);  // 10Hz检查频率
        
        while (ros::ok() && !awake_received_) {
            ros::spinOnce();
            rate.sleep();
        }
        
        if (awake_received_) {
            ROS_INFO("已接收到唤醒信号!");
            return true;
        }
        return false;
    }

private:
    void awakeCallback(const std_msgs::Int8ConstPtr& msg) {
        if (msg->data == 1) {
            awake_received_ = true;
            ROS_INFO("检测到唤醒信号 (awake_flag=1)");
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    bool awake_received_;
};

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
    // AwakeDetector awakeDetector(nh);
    // if (!awakeDetector.waitForAwake()) {
    //     ROS_ERROR("未接收到唤醒信号，程序退出");
    //     return 1;
    // }
    
    // ROS_INFO("已唤醒，开始执行任务...");
    

    //--------------------------------------走廊环境导航，发布目标点--------------------------------//
    ROS_INFO("走廊环境导航开始");
    mecanumController.rotateCircle(3.14,0.5);
    mecanumController.rotateCircle(3.14,0.5);//
    // ros::Time start_time = ros::Time::now();
    // geometry_msgs::Twist twist;
    // twist.angular.z = 2;
    // while((ros::Time::now() - start_time).toSec() < 3.14){
    //     mecanumController.cmd_pub_.publish(twist);
    // }//旋转收敛定位

    go_destination(goal,1.25,0.75,3.14,q,ac);
    // ros::Duration(0.5).sleep();
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
    // //board_class 为 1（蔬菜）、2（水果）、3（甜品）
    if (board_class >= 1 && board_class <= 3  ) {
        play_audio(voice[0][board_class-1]);
    }

    go_destination(goal,0.50,2.25,3.14,q,ac);
    ROS_INFO("走廊环境导航完成");


    //----------------------------------------目标检测区域开始-------------------------------------------//
    ROS_INFO("拣货区域任务开始");
    size_t board_count;
    int board_name;
    int flag=0;//判断雷达识别的点是否和视觉对得上
    double lidar_yaw;
    //第一点视觉识别
    //视觉识别开始，先传个-1把摄像头打开
    std::vector<int> a = {-1,-1,-1,-1,-1,-1};
    mecanumController.detect(a,-1);
    board_name = mecanumController.turn_and_find(5.5,1,board_class,-0.4);//请求视觉识别板子服务
    if (board_name != -1) {
        if (poseget_client.call(pose_result)){
            ROS_INFO("小车坐标xyz:%f,%f,%f",pose_result.response.pose_at[0],pose_result.response.pose_at[1],pose_result.response.pose_at[2]);
        }
        else{
            ROS_ERROR("获取位姿失败");
        }
        where_board.request.lidar_process_start = 1;//雷达获取前方障碍物距离
        client_find_board.call(where_board);
        ROS_INFO("查看雷达距离%f",where_board.response.lidar_results[0]);
        if(where_board.response.lidar_results[0] <3.0){
            double target_x = (where_board.response.lidar_results[0]-0.6)*cos(pose_result.response.pose_at[2])+pose_result.response.pose_at[0];
            double target_y = (where_board.response.lidar_results[0]-0.6)*sin(pose_result.response.pose_at[2])+pose_result.response.pose_at[1];
            ROS_INFO("目的地%f,%f,%f",target_x,target_y,pose_result.response.pose_at[2]);
            go_destination(goal,target_x,target_y,pose_result.response.pose_at[2],q,ac);
            mecanumController.adjust(board_class,0.4);
            if(mecanumController.forward(board_class,0.3)){//直接前进，直到目标检测框高超过230
                flag = 1;
            }
        }
        else{
            ROS_INFO("第一点找板板子，但是距离太远不予理会");
        }
    }
    ROS_INFO("第一个找板点是否找到板子%d",flag);
    if(!flag){
        //前往区域中心找板子
        ROS_INFO("前往中心找板");
        go_destination(goal,1.25,3.75,0,q,ac);
        board_name = mecanumController.turn_and_find(17,1,board_class,0.4);//请求视觉识别板子服务
        if(board_name!=-1){
            if (poseget_client.call(pose_result)){
                ROS_INFO("小车坐标xyz:%f,%f,%f",pose_result.response.pose_at[0],pose_result.response.pose_at[1],pose_result.response.pose_at[2]);
            }
            else{
                ROS_ERROR("获取位姿失败");
            }
            where_board.request.lidar_process_start = 1;//雷达获取前方障碍物距离
            if (client_find_board.call(where_board)){
                double target_x = (where_board.response.lidar_results[0]-0.6)*cos(pose_result.response.pose_at[2])+pose_result.response.pose_at[0];
                double target_y = (where_board.response.lidar_results[0]-0.6)*sin(pose_result.response.pose_at[2])+pose_result.response.pose_at[1];
                ROS_INFO("目的地%f,%f,%f",target_x,target_y,pose_result.response.pose_at[2]);
                go_destination(goal,target_x,target_y,pose_result.response.pose_at[2],q,ac);
            }
            mecanumController.adjust(board_class,0.4);
            if(mecanumController.forward(board_class,0.3)){//直接前进，直到目标检测框高超过230
                flag = 1;
            }
        }
    }
    ROS_INFO("中心找板点是否找到板子%d",flag);
    if(!flag){
        //前往第三区域找板子
        ROS_INFO("前往第三点找板");
        go_destination(goal,2.0,4.25,0,q,ac);
        board_name = mecanumController.turn_and_find(17,1,board_class,0.4);//请求视觉识别板子服务
        if(board_name!=-1){
            if (poseget_client.call(pose_result)){
                ROS_INFO("小车坐标xyz:%f,%f,%f",pose_result.response.pose_at[0],pose_result.response.pose_at[1],pose_result.response.pose_at[2]);
            }
            else{
                ROS_ERROR("获取位姿失败");
            }
            where_board.request.lidar_process_start = 1;//雷达获取前方障碍物距离
            client_find_board.call(where_board);
            if(where_board.response.lidar_results[0] <3.0){
                double target_x = (where_board.response.lidar_results[0]-0.6)*cos(pose_result.response.pose_at[2])+pose_result.response.pose_at[0];
                double target_y = (where_board.response.lidar_results[0]-0.6)*sin(pose_result.response.pose_at[2])+pose_result.response.pose_at[1];
                ROS_INFO("目的地%f,%f,%f",target_x,target_y,pose_result.response.pose_at[2]);
                go_destination(goal,target_x,target_y,pose_result.response.pose_at[2],q,ac);
                mecanumController.adjust(board_class,0.4);
                if(mecanumController.forward(board_class,0.3)){//直接前进，直到目标检测框高超过230
                    flag = 1;
                }
            }
        }
    }
    if (!flag){
        ROS_INFO("找不到板子，直接走了");
    }
    mecanumController.cap_close();
    ROS_INFO("%d",board_name);
    if (board_name >= 0 && board_name <= 9 && flag==1) {
        ROS_INFO("播报");
        play_audio(voice[1][board_name]);
    }
    
    //-----------------------------------------------仿真开始--------------------------------------------//
    ROS_INFO("前往仿真区域");
    go_destination(goal,1.25,3.75,0.0,q,ac);
    //发送仿真消息
    ros::Rate rate(1);
    // waitForContinue();
    // while (ros::ok()) {
    //     sim_talkto_car.car_msg_publish(board_name);
    //     rate.sleep();
    //     ros::spinOnce();
    //     if(sim_talkto_car.sim_done==1){
    //         break;
    //     }
    // }
    // if(sim_talkto_car.sim_room>=0){
    //     play_audio(voice[2][sim_talkto_car.sim_room-1]);
    // }
    // else {
    //     ROS_INFO("仿真失败");
    // }
    

    //--------------------------------------------前往红绿灯识别区域--------------------------------------------//
    ROS_INFO("前往红绿灯区域路口1");
    go_destination(goal,3.25,4.50,1.57,q,ac);
    if (detectTrafficLightStatus()==2){
        ROS_INFO("路口1可通过");
        play_audio(voice[3][0]);
        go_destination(goal,2.75,3.50,-0.78,q,ac);
    } 
    else {
        ROS_INFO("前往红绿灯区域路口2");
        go_destination(goal,4.25,4.50,1.57,q,ac);
        if (detectTrafficLightStatus()==2){
            ROS_INFO("路口2可通过");
            play_audio(voice[3][1]);
            go_destination(goal,4.75,3.50,-2.31,q,ac);
        } 
    }

    //-----------------------------------------视觉巡线---------------------------------------------//
    if(line_client.call(linefollow_start)){
        ROS_INFO("视觉巡线结束");
    }
    else{
        ROS_ERROR("视觉巡线失败....");
    }
    
    play_audio(voice[4][board_name*3+sim_talkto_car.sim_detect_class]);
    // play_audio(voice[board_name*3][1]);
    ros::spin();

    return 0;
}

