/**************************************************************************
作者：caidx1
功能：语音唤醒控制器（依赖 xf_mic/xf_asr_offline_node）
**************************************************************************/
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <xf_mic_asr_offline/Set_Awake_Word_srv.h>
#include <xf_mic_asr_offline/Get_Offline_Result_srv.h>

using namespace std;

int awake_flag = 0;    // 唤醒标志位（0=休眠，1=唤醒）
int confidence_threshold = 18;    // 识别置信度阈值
int seconds_per_order = 3;        // 单次命令最长识别时间（秒）

int main(int argc, char *argv[])
{
    // 初始化唯一节点名（避免重复初始化）
    ros::init(argc, argv, "voice_wakeup_node");
    ros::NodeHandle nh;

    // ======================
    // 1. 创建服务客户端（修改为 xf_mic 路径）
    // ======================
    // 设置唤醒词服务（xf_mic/xf_asr_offline_node 提供的服务）
    ros::ServiceClient set_awake_word_client = 
        nh.serviceClient<xf_mic_asr_offline::Set_Awake_Word_srv>("/xf_mic/xf_asr_offline_node/set_awake_word_srv");
    
    // 离线识别结果服务（根据实际服务路径调整）
    ros::ServiceClient get_offline_result_client = 
        nh.serviceClient<xf_mic_asr_offline::Get_Offline_Result_srv>("/xf_mic/xf_asr_offline_node/get_offline_result_srv");

    // 唤醒状态发布者
    ros::Publisher awake_flag_pub = nh.advertise<std_msgs::Int8>("awake_flag", 1);

    // 加载ROS参数
    nh.param("/confidence", confidence_threshold, 18);
    nh.param("/seconds_per_order", seconds_per_order, 3);

    // ======================
    // 2. 设置唤醒词（默认："小车小车"）
    // ======================
    xf_mic_asr_offline::Set_Awake_Word_srv set_awake_word_srv;
    set_awake_word_srv.request.awake_word = "小车小车";  // 可自定义唤醒词

    // 调用服务并检查结果
    if (!set_awake_word_client.call(set_awake_word_srv)) 
    {
        ROS_FATAL("唤醒词服务调用失败！请检查 xf_mic/xf_asr_offline_node 是否启动");
        return -1;
    }
    else if (set_awake_word_srv.response.result != 0) 
    {
        ROS_FATAL("唤醒词设置失败：%s", set_awake_word_srv.response.fail_reason.c_str());
        return -1;
    }
    ROS_INFO("唤醒词设置成功：%s", set_awake_word_srv.request.awake_word.c_str());

    // ======================
    // 3. 配置离线识别参数
    // ======================
    xf_mic_asr_offline::Get_Offline_Result_srv get_offline_result_srv;
    get_offline_result_srv.request.offline_recognise_start = 1;
    get_offline_result_srv.request.confidence_threshold = confidence_threshold;
    get_offline_result_srv.request.time_per_order = seconds_per_order;

    // ======================
    // 4. 主循环：处理唤醒逻辑
    // ======================
    ros::Rate loop_rate(10);

    while (ros::ok()) 
    {
        // 休眠状态：仅监听唤醒词
        if (!awake_flag) 
        {
            // 调用识别服务获取唤醒结果
            if (get_offline_result_client.call(get_offline_result_srv)) 
            {
                // 解析结果：匹配唤醒词且置信度达标
                if (get_offline_result_srv.response.result == "ok" && 
                    get_offline_result_srv.response.text == "小车小车") 
                {
                    awake_flag = 1;
                    std_msgs::Int8 msg;
                    msg.data = awake_flag;
                    awake_flag_pub.publish(msg);
                    ROS_INFO("成功唤醒！当前状态：唤醒");
                }
                else if (get_offline_result_srv.response.result == "fail") 
                {
                    ROS_DEBUG("未检测到唤醒词");
                }
            } 
            else 
            {
                ROS_WARN("识别服务调用失败，重试中...");
            }
        }
        // 唤醒状态：可扩展其他功能（如命令处理）
        else 
        {
            ROS_INFO("当前状态：唤醒，等待命令...");
            // 此处可添加命令词识别逻辑（如"小车休眠"）
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
