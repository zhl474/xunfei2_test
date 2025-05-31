/**************************************************************************
作者：caidx1
功能：语音唤醒控制器（仅保留唤醒功能）
**************************************************************************/
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <xf_mic_asr_offline/Set_Awake_Word_srv.h>
#include <xf_mic_asr_offline/Get_Offline_Result_srv.h>

using namespace std;

int awake_flag = 0;    // 唤醒标志位（0=休眠，1=唤醒）
int confidence_threshold = 18;    // 识别置信度阈值（可通过ROS参数修改）
int seconds_per_order = 3;        // 单次命令最长识别时间（秒）

/**************************************************************************
函数功能：主函数
入口参数：无
返回  值：无
**************************************************************************/
int main(int argc, char *argv[])
{
	setlocale(LC_ALL,"");
    ros::init(argc, argv, "voice_wakeup_node");    // 初始化ROS节点（唯一节点名）
    ros::NodeHandle nh;

    // 创建唤醒词设置服务客户端
    ros::ServiceClient set_awake_word_client = 
        nh.serviceClient<xf_mic_asr_offline::Set_Awake_Word_srv>("xf_asr_offline_node/set_awake_word_srv");

    // 创建离线识别结果服务客户端
    ros::ServiceClient get_offline_result_client = 
        nh.serviceClient<xf_mic_asr_offline::Get_Offline_Result_srv>("broadcast/get_offline_recognise_result_srv");

    // 创建唤醒标志发布者（用于通知其他节点当前状态）
    ros::Publisher awake_flag_pub = nh.advertise<std_msgs::Int8>("awake_flag", 1);

    // 从ROS参数服务器获取配置（若不存在则使用默认值）
    nh.param("/confidence", confidence_threshold, 18);
    nh.param("/seconds_per_order", seconds_per_order, 3);

    /**************************
     * 1. 设置唤醒词（默认："小车小车"）
     **************************/
    xf_mic_asr_offline::Set_Awake_Word_srv set_awake_word_srv;
    set_awake_word_srv.request.awake_word = "小车小车";    // 可修改为自定义唤醒词

    if (set_awake_word_client.call(set_awake_word_srv)) 
    {
        ROS_INFO("唤醒词设置成功！当前唤醒词：%s", set_awake_word_srv.request.awake_word.c_str());
    } 
    else 
    {
        ROS_ERROR("唤醒词设置失败！请检查服务是否启动");
        return -1;
    }

    /**************************
     * 2. 配置离线识别参数
     **************************/
    xf_mic_asr_offline::Get_Offline_Result_srv get_offline_result_srv;
    get_offline_result_srv.request.offline_recognise_start = 1;    // 启动识别
    get_offline_result_srv.request.confidence_threshold = confidence_threshold;
    get_offline_result_srv.request.time_per_order = seconds_per_order;

    /**************************
     * 3. 主循环：处理唤醒逻辑
     **************************/
    ros::Rate loop_rate(10);    // 10Hz循环频率

    while (ros::ok()) 
    {
        // 休眠状态：仅监听唤醒词
        if (!awake_flag) 
        {
            // 调用离线识别服务，仅检测唤醒词
            if (get_offline_result_client.call(get_offline_result_srv)) 
            {
                // 解析唤醒结果
                if (get_offline_result_srv.response.result == "ok" && 
                    get_offline_result_srv.response.text == "小车小车")    // 匹配唤醒词
                {
                    awake_flag = 1;    // 标记为唤醒状态
                    std_msgs::Int8 msg;
                    msg.data = awake_flag;
                    awake_flag_pub.publish(msg);    // 发布唤醒信号
                    ROS_INFO("已唤醒！等待命令...");
                }
            } 
            else 
            {
                ROS_WARN("识别服务调用失败，重试中...");
            }
        }
        // 唤醒状态：可扩展其他功能（此处仅保留基础状态显示）
        else 
        {
            ROS_INFO("当前为唤醒状态，输入'小车休眠'可进入休眠");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
