#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <cstdlib>
#include <string>
#include <vector>

std::string target_node_name = "/qr_detector_node";  // 默认目标节点名称
std::string launch_file = "qr_01/launch/aruco_sub.launch";  // 启动文件路径

// 终止节点服务
bool killNodeService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    std::string command = "rosnode kill " + target_node_name;
    int status = system(command.c_str());
    
    if (status == 0) {
        res.success = true;
        res.message = "节点已成功终止";
        ROS_INFO("成功终止节点: %s", target_node_name.c_str());
    } else {
        res.success = false;
        res.message = "终止节点失败";
        ROS_ERROR("终止节点 %s 失败", target_node_name.c_str());
    }
    return true;
}

// 启动节点服务
bool launchNodeService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    // 使用roslaunch命令启动节点
    std::string command = "roslaunch " + launch_file + " &";
    int status = system(command.c_str());
    
    if (status == 0) {
        res.success = true;
        res.message = "节点已成功启动";
        ROS_INFO("成功启动节点: %s", target_node_name.c_str());
    } else {
        res.success = false;
        res.message = "启动节点失败";
        ROS_ERROR("启动节点 %s 失败", target_node_name.c_str());
    }
    return true;
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "node_manager_server");
    ros::NodeHandle nh("~");  // 私有句柄
    
    // 读取私有参数（可动态指定目标节点名称和启动文件）
    nh.param<std::string>("target_node", target_node_name, "/qr_detector_node");
    nh.param<std::string>("launch_file", launch_file, "qr_01/launch/aruco_sub.launch");
    
    // 注册服务
    ros::ServiceServer kill_service = nh.advertiseService("/kill_node", killNodeService);
    ros::ServiceServer launch_service = nh.advertiseService("/launch_node", launchNodeService);
    
    ROS_INFO("节点管理器已启动，目标节点: %s", target_node_name.c_str());
    ROS_INFO("支持服务: /kill_node（终止节点）, /launch_node（启动节点）");
    
    ros::spin();
    return 0;
}