#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <cstdlib>
#include <string>

// 定义服务调用函数
bool callKillService() {
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("/kill_node");
    
    std_srvs::Trigger srv;
    if (client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("节点终止成功: %s", srv.response.message.c_str());
            return true;
        } else {
            ROS_ERROR("节点终止失败: %s", srv.response.message.c_str());
            return false;
        }
    } else {
        ROS_ERROR("终止服务调用失败");
        return false;
    }
}

bool callLaunchService() {
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("/launch_node");
    
    std_srvs::Trigger srv;
    if (client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("节点启动成功: %s", srv.response.message.c_str());
            return true;
        } else {
            ROS_ERROR("节点启动失败: %s", srv.response.message.c_str());
            return false;
        }
    } else {
        ROS_ERROR("启动服务调用失败");
        return false;
    }
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "node_manager_client");
    
    // 检查参数是否指定操作类型（kill/launch）
    if (argc != 2) {
        ROS_ERROR("请指定操作类型：kill（终止）或 launch（启动）");
        return 1;
    }
    
    std::string operation = argv[1];
    
    if (operation == "kill") {
        return callKillService() ? 0 : 1;
    } else if (operation == "launch") {
        return callLaunchService() ? 0 : 1;
    } else {
        ROS_ERROR("无效操作类型！支持：kill/launch");
        return 1;
    }
}