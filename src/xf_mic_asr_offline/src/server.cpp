#include <ros/ros.h>
#include <xf_mic_asr_offline/Get_Awake_Angle_srv.h>

// 服务回调函数
bool handleAwakeAngleRequest(
    xf_mic_asr_offline::Get_Awake_Angle_srv::Request &req,
    xf_mic_asr_offline::Get_Awake_Angle_srv::Response &res) {
    
    ROS_INFO("收到唤醒角度请求: get_awake_angle = %d", req.get_awake_angle);

    // 检查请求参数是否在有效范围内（0-5）
    if (req.get_awake_angle >= 0 && req.get_awake_angle <= 5) {
        // 模拟获取唤醒角度（实际应用中应替换为真实逻辑）
        res.result = "SUCCESS";  // 状态码对应的文本描述
        res.awake_angle = req.get_awake_angle * 15;  // 映射0-5到0-75度
        res.fail_reason = "";
        ROS_INFO("返回唤醒角度: %d 度", res.awake_angle);
        
        // 返回状态码1表示成功
        return true;
    } else {
        res.result = "FAILURE";  // 状态码对应的文本描述
        res.awake_angle = -1;  // 无效值
        res.fail_reason = "get_awake_angle必须在0-5之间";
        ROS_ERROR("无效请求: 参数超出范围");
        
        // 返回状态码0表示失败
        return false;
    }
}

int main(int argc, char **argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "server");
    ros::NodeHandle nh;

    // 创建服务
    ros::ServiceServer service = nh.advertiseService(
        "get_awake_angle_srv", handleAwakeAngleRequest);

    ROS_INFO("唤醒角度服务已启动，等待请求...");
    ros::spin();

    return 0;
}
