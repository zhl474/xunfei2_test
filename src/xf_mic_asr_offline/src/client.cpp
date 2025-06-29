#include <ros/ros.h>
#include <xf_mic_asr_offline/Get_Awake_Angle_srv.h>

int main(int argc, char **argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "client");
    ros::NodeHandle nh;

    // 创建服务客户端
    ros::ServiceClient client = nh.serviceClient<xf_mic_asr_offline::Get_Awake_Angle_srv>(
        "get_awake_angle_srv");

    // 等待服务上线
    ROS_INFO("等待服务上线...");
    ros::service::waitForService("get_awake_angle_srv");
    ROS_INFO("服务已上线，准备发送请求");

    // 创建服务请求对象
    xf_mic_asr_offline::Get_Awake_Angle_srv srv;

    // 设置请求参数（1表示获取唤醒角度）
    srv.request.get_awake_angle = 1;

    // 发送请求并处理响应
    if (client.call(srv)) {
        ROS_INFO("服务调用成功:");
        ROS_INFO("结果: %s", srv.response.result.c_str());
        if (srv.response.awake_angle >= 0) {
            ROS_INFO("唤醒角度: %d 度", srv.response.awake_angle);
        } else {
            ROS_WARN("未获取有效角度");
        }
        if (!srv.response.fail_reason.empty()) {
            ROS_ERROR("失败原因: %s", srv.response.fail_reason.c_str());
        }
    } else {
        ROS_ERROR("服务调用失败");
        return 1;
    }

    return 0;
}
