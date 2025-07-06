#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "detect_client/Messages.h" 

#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <cmath>
#include <vector>

// 为move_base的Action客户端定义一个别名
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// --- 全局常量，方便调整 ---
const int TARGET_CLASS_ID = 0; // <-- 在这里设置您要寻找的目标图案的类型编号
const double SEARCH_ROTATION_SPEED = 0.4; // 原地旋转搜索时的角速度 (rad/s)
const int PIXEL_ALIGN_THRESHOLD = 7; // 像素对准误差阈值，小于此值认为已对准
const double APPROACH_DISTANCE = 0.4; // 最终逼近目标板的距离 (米)

// PID控制器参数 (需要根据实际机器人进行微调)
const double KP = 0.08; // 比例增益
const double KI = 0.01; // 积分增益
const double KD = 0.02; // 微分增益

bool approachTarget(
    const std::vector<int32_t>& detection_result,
    MoveBaseClient& ac,
    tf2_ros::Buffer& tf_buffer)
{
    ROS_INFO("目标已对准，开始计算精确坐标并逼近...");

    // 从传入的检测结果中提取边界框信息
    int best_box_x = detection_result[0];
    int best_box_w = detection_result[2];

    // 获取最新的Lidar数据
    sensor_msgs::LaserScan::ConstPtr scan_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", ros::Duration(2.0));
    if (!scan_msg) {
        ROS_ERROR("无法接收到雷达数据，逼近失败。");
        return false;
    }

    // 计算目标在相机和雷达坐标系中的位置
    const double CAMERA_HFOV = 1.3962634; // 相机物理水平视场角 
    const int IMAGE_WIDTH = 640;          // 检测算法处理的图像宽度 
    // 计算目标中心点的x坐标
    double center_x = best_box_x + best_box_w / 2.0;
    double angle_in_camera = (IMAGE_WIDTH / 2.0 - center_x) / (IMAGE_WIDTH / 2.0) * (CAMERA_HFOV / 2.0);
    
    // int scan_index = (int)((angle_in_camera - scan_msg->angle_min) / scan_msg->angle_increment);
    int scan_index = 168;
    if (scan_index < 0 || scan_index >= scan_msg->ranges.size()) {
        ROS_ERROR("计算出的雷达索引超出范围，逼近失败。");
        return false;
    }

    double distance = scan_msg->ranges[scan_index];
    if (std::isinf(distance) || std::isnan(distance)) {
        ROS_ERROR("雷达距离值无效，逼近失败。");
        return false;
    }

    // 将目标点坐标转换到全局 "map" 坐标系
    geometry_msgs::PointStamped target_point_laser_frame;
    target_point_laser_frame.header.stamp = ros::Time::now();
    target_point_laser_frame.header.frame_id = scan_msg->header.frame_id; // 通常是 "laser_frame"
    target_point_laser_frame.point.x = distance * std::cos(angle_in_camera);
    target_point_laser_frame.point.y = distance * std::sin(angle_in_camera);
    target_point_laser_frame.point.z = 0;

    geometry_msgs::PointStamped target_point_map_frame;
    try {
        tf_buffer.transform(target_point_laser_frame, target_point_map_frame, "map", ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("坐标变换失败: %s", ex.what());
        return false;
    }
    ROS_INFO("目标板位于地图坐标: [x: %.2f, y: %.2f]", target_point_map_frame.point.x, target_point_map_frame.point.y);

    // 计算最终的逼近导航点
    move_base_msgs::MoveBaseGoal approach_goal;
    approach_goal.target_pose.header.frame_id = "map";
    approach_goal.target_pose.header.stamp = ros::Time::now();

    geometry_msgs::TransformStamped robot_pose_transform;
    try {
        robot_pose_transform = tf_buffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("无法获取机器人当前位姿: %s", ex.what());
        return false;
    }
    
    double robot_x = robot_pose_transform.transform.translation.x;
    double robot_y = robot_pose_transform.transform.translation.y;
    double target_x = target_point_map_frame.point.x;
    double target_y = target_point_map_frame.point.y;
    
    double angle_to_target = std::atan2(target_y - robot_y, target_x - robot_x);
    
    approach_goal.target_pose.pose.position.x = target_x - APPROACH_DISTANCE * std::cos(angle_to_target);
    approach_goal.target_pose.pose.position.y = target_y - APPROACH_DISTANCE * std::sin(angle_to_target);
    
    tf2::Quaternion q;
    q.setRPY(0, 0, angle_to_target);
    approach_goal.target_pose.pose.orientation = tf2::toMsg(q);

    // 发送导航指令并等待结果
    ROS_INFO("发送最终逼近指令...");
    ac.sendGoal(approach_goal);
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("成功到达目标板前方！");
        return true;
    } else {
        ROS_ERROR("逼近导航失败。");
        return false;
    }
}

//阶段一：旋转与对准。原地旋转寻找目标，找到后使用PID对准。
// target_id 要寻找的目标类别ID
// detection_client 检测服务的client
//cmd_vel_pub 用于发布速度指令的publisher
// final_detection_result [输出参数] 用于存储最终对准后的目标信息
// 如果成功对准返回true，否则返回false
 
bool turnAndAlign(
    int target_id,
    ros::ServiceClient& detection_client,
    ros::Publisher& cmd_vel_pub,
    std::vector<int32_t>& final_detection_result)
{
    ros::Rate loop_rate(20); // PID控制循环频率20Hz
    geometry_msgs::Twist twist_msg;
    detect_client::Messages srv;

    // --- 搜索阶段 ---
    ROS_INFO("开始原地旋转搜索目标 (ID: %d)...", target_id);
    twist_msg.angular.z = SEARCH_ROTATION_SPEED;
    ros::Time search_start_time = ros::Time::now();

    while (ros::ok()) {
        cmd_vel_pub.publish(twist_msg); // 持续发布旋转指令

        if (detection_client.call(srv) &&!srv.response.position.empty()) {
            for (size_t i = 0; i < srv.response.position.size(); i += 6) {
                if (srv.response.position[i + 4] == target_id) {
                    ROS_INFO("发现目标！停止搜索，开始PID对准。");
                    final_detection_result.assign(srv.response.position.begin() + i, srv.response.position.begin() + i + 6);
                    goto alignment_phase; // 跳转到对准阶段
                }
            }
        }

        // 超时检查
        if ((ros::Time::now() - search_start_time).toSec() > 30.0) {
            ROS_ERROR("搜索超时（30秒），未发现目标。");
            twist_msg.angular.z = 0;
            cmd_vel_pub.publish(twist_msg);
            return false;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

alignment_phase:
    // --- 对准阶段 ---
    double integral = 0;
    double prev_error = 0;
    const int IMAGE_WIDTH = 640;

    while (ros::ok()) {
        // 持续获取最新的目标位置
        if (!detection_client.call(srv) || srv.response.position.empty()) {
            ROS_WARN("在对准过程中丢失目标，重新进入搜索模式...");
            return turnAndAlign(target_id, detection_client, cmd_vel_pub, final_detection_result); // 重新开始
        }
        
        // 找到目标ID在最新检测结果中的位置
        bool target_found_in_frame = false;
        for (size_t i = 0; i < srv.response.position.size(); i += 6) {
            if (srv.response.position[i + 4] == target_id) {
                final_detection_result.assign(srv.response.position.begin() + i, srv.response.position.begin() + i + 6);
                target_found_in_frame = true;
                break;
            }
        }

        if (!target_found_in_frame) {
            ROS_WARN("在对准过程中丢失目标，重新进入搜索模式...");
            return turnAndAlign(target_id, detection_client, cmd_vel_pub, final_detection_result); // 重新开始
        }

        // 计算中心点误差
        int center_x = final_detection_result[0] + final_detection_result[2] / 2;
        double error = (IMAGE_WIDTH / 2.0) - center_x;

        // 检查是否已对准
        if (std::abs(error) < PIXEL_ALIGN_THRESHOLD) {
            ROS_INFO("已精确对准目标！");
            twist_msg.angular.z = 0;
            cmd_vel_pub.publish(twist_msg); // 停止所有运动
            return true;
        }

        // PID计算
        integral += error * (1.0 / 20.0); // dt = 1 / loop_rate
        double derivative = (error - prev_error) * 20.0;
        double output = KP * error + KI * integral + KD * derivative;
        prev_error = error;

        // 限制输出速度
        output = std::max(-0.5, std::min(output, 0.5));
        
        twist_msg.angular.z = output;
        cmd_vel_pub.publish(twist_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return false; // 正常情况下不会执行到这里
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "find_and_approach");
    ros::NodeHandle nh;

    // 初始化所有客户端、发布者和监听器
    MoveBaseClient ac("move_base", true);
    ros::ServiceClient detection_client = nh.serviceClient<detect_client::Messages>("tensorRT_detect");
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // 等待所有外部服务启动
    ROS_INFO("等待move_base action server...");
    ac.waitForServer();
    ROS_INFO("等待 tensorRT_detect service...");
    detection_client.waitForExistence();
    ROS_INFO("所有服务已就绪，任务开始！");

    // --- 任务执行流程 ---
    
    // 1. 执行阶段一：旋转与对准
    std::vector<int32_t> detection_result;// 创建一个空的vector，用于接收对准后的目标信息
    bool aligned = turnAndAlign(TARGET_CLASS_ID, detection_client, cmd_vel_pub, detection_result);

    // 2. 如果对准成功，则执行阶段二：目标逼近
    if (aligned) {
        bool approached = approachTarget(detection_result, ac, tf_buffer);
        if (approached) {
            ROS_INFO("任务成功完成！");
        } else {
            ROS_ERROR("任务失败：逼近阶段出错。");
        }
    } else {
        ROS_ERROR("任务失败：旋转与对准阶段出错。");
    }

    return 0;
}