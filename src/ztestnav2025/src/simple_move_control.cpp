#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "ztestnav2025/set_speed.h"

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

geometry_msgs::Twist twist_msg;
bool start = false;
bool movebaseflag = false;
float xtarget = 0;
float ytarget = 0;
float yawtarget = 0;
// 速度设置服务回调
bool setSpeedCallback(ztestnav2025::set_speed::Request &req,ztestnav2025::set_speed::Response &resp)
{
    if (req.work == false) {
        ROS_INFO("运动控制节点停止");
        start=false;
    }
    else {
        // 业务逻辑：检测成功时设置为正向速度，失败时为0
        if (req.target_twist.angular.x != 0 ||req.target_twist.angular.y !=0 || req.target_twist.linear.z != 0){
            ROS_ERROR("请勿输入不存在的速度");
            return false;
        }
        twist_msg = req.target_twist;
        start = true;
        // ROS_INFO("运动控制节点运行中");
    }
    if(req.movebase_flag){
        movebaseflag = true;
        xtarget = req.target_x;
        ytarget = req.target_y;
        yawtarget = req.target_yaw;
    }
    resp.success = true;
    return true;
}

int main(int argc, char *argv[]) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "simple_move_control");
    ros::NodeHandle nh;
    
    // 创建服务服务器
    ros::ServiceServer speed_service = nh.advertiseService(
        "set_speed", setSpeedCallback);
    
    // 创建速度发布器
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>(
        "/cmd_vel", 1);
    
    // 初始化控制循环
    ros::Rate control_rate(20); // 严格20Hz频率

    MoveBaseClient ac1("move_base", true); 
    tf2::Quaternion q1;  
    //等待action回应
    while(!ac1.waitForServer()){
        ROS_INFO("等待movebase服务中---");
    } 
    move_base_msgs::MoveBaseGoal goal1;
    goal1.target_pose.header.frame_id = "map";
    
    ROS_INFO("运动控制节点已启动 (20Hz 控制循环)");
    
    while (ros::ok()) {
        // 1. 处理回调获取当前速度值
        ros::spinOnce();
        if(start) cmd_pub.publish(twist_msg);
        if(movebaseflag){
            ROS_INFO("开始movebase");
            q1.setRPY(0, 0, yawtarget);
            goal1.target_pose.header.stamp = ros::Time::now();
            goal1.target_pose.pose.position.x = xtarget;
            goal1.target_pose.pose.position.y = ytarget;
            goal1.target_pose.pose.position.z = 0.0;
            goal1.target_pose.pose.orientation.x = q1.x();
            goal1.target_pose.pose.orientation.y = q1.y();
            goal1.target_pose.pose.orientation.z = q1.z();
            goal1.target_pose.pose.orientation.w = q1.w();
            ac1.sendGoal(goal1);
            ac1.waitForResult();
            if(ac1.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("到达目标");
            else
                ROS_INFO("无法到达目标");
            movebaseflag = false;
        }
        control_rate.sleep();
    }

    ROS_INFO("节点关闭，发送停止指令");
    return 0;
}