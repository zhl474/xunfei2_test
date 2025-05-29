#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <string>
#include <thread>
#include <csignal>
#include <broadcast/Set_Awake_Word_srv.h>
#include <broadcast/Get_Offline_Result_srv.h>

// ======================
// 全局变量定义
// ======================
int room_index = 0;              // 当前房间号
int awake_flag = 0;             // 唤醒标志位（0:休眠，1:唤醒）
int recognize_fail_count = 0;   // 连续识别失败计数
const int recognize_fail_threshold = 5; // 失败阈值
volatile std::sig_atomic_t sig_flag = 0; // 信号处理标志位（线程安全）

// ======================
// 语音资源路径（需根据实际路径修改）
// ======================
// 任务类型语音（组0）：甜点/水果/蔬菜
const std::vector<std::string> TASK_VOICE = {
    "aplay ~/ucar_car/src/broadcast/20_wav/1_task_dessert.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/1_task_fruit.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/1_task_vegetable.wav"
};

// 物品获取语音（组1）
const std::vector<std::string> GET_VOICE = {
    "aplay ~/ucar_car/src/broadcast/20_wav/2_get_apple.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/2_get_banana.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/2_get_cake.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/2_get_chili.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/2_get_cola.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/2_get_milk.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/2_get_poatato.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/2_get_tomato.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/2_get_watermelon.wav"
};

// 房间提示语音（组2）
const std::vector<std::string> ROOM_VOICE = {
    "aplay ~/ucar_car/src/broadcast/20_wav/3_gaozebo_A.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/3_gaozebo_B.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/3_gaozebo_C.wav"
};

// 路径提示语音（组3）
const std::vector<std::string> PATH_VOICE = {
    "aplay ~/ucar_car/src/broadcast/20_wav/4_road_1.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/4_road_2.wav"
};

// 费用语音（一维数组）
const std::vector<std::string> COST_VOICE = {
    "aplay ~/ucar_car/src/broadcast/20_wav/5_apple_apple.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_apple_banana.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_apple_watermelon.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_banana_banana.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_banana_watermelon.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_cake_cola.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_chili_chili.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_chili_potato.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_chili_tomato.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_cola_cola.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_milk_cake.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_milk_cola.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_milk_milk.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_potato_potato.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_tomato_potato.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_tomato_tomato.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_watermelon_watermelon.wav"
};

// 唤醒成功提示音（需确保路径正确）
const std::string WAKEUP_SOUND = "aplay ~/ucar_car/src/broadcast/20_wav/wakeup_success.wav";

// ======================
// 功能函数声明
// ======================
void play_audio(const std::string& cmd);          // 播放语音
void command_interaction();                       // 命令行交互
bool process_voice_wakeup(ros::ServiceClient&, ros::ServiceClient&); // 唤醒处理
void sig_handler(int signo);                      // 信号处理

// ======================
// 功能函数实现
// ======================
/**
 * @brief 播放语音文件
 * @param cmd 系统命令（如 aplay 路径）
 */
void play_audio(const std::string& cmd) {
    system(cmd.c_str());
}

/**
 * @brief 信号处理函数（捕获 Ctrl+C）
 * @param signo 信号类型
 */
void sig_handler(int signo) {
    if (signo == SIGINT) {
        std::cout << "\n[系统] 接收到退出信号，正在关闭系统...\n" << std::endl;
        sig_flag = 1; // 设置退出标志
    }
}

/**
 * @brief 命令行交互界面
 */
void command_interaction() {
    while (!sig_flag) { // 循环条件：未接收到退出信号
        std::cout << "\n=== 语音播报控制 ===" << std::endl;
        std::cout << "1. 唤醒提示音（强制唤醒）" << std::endl;
        std::cout << "2. 播放任务类型语音（索引 0-2）" << std::endl;
        std::cout << "3. 播放物品获取语音（索引 0-8）" << std::endl;
        std::cout << "4. 播放房间提示语音（索引 0-2）" << std::endl;
        std::cout << "5. 播放路径提示语音（索引 0-1）" << std::endl;
        std::cout << "6. 播放费用语音（索引 0-16）" << std::endl;
        std::cout << "7. 切换房间（0-3）" << std::endl;
        std::cout << "0. 退出系统" << std::endl;
        std::cout << "当前状态: " << (awake_flag ? "唤醒" : "休眠") << std::endl;
        std::cout << "请选择操作：";

        int choice;
        std::cin >> choice;
        std::cin.ignore(); // 清理输入缓冲区

        switch (choice) {
            case 0: // 退出系统
                sig_flag = 1;
                std::cout << "[系统] 正在退出...\n" << std::endl;
                break;

            case 1: // 手动唤醒（测试用）
                play_audio(WAKEUP_SOUND);
                awake_flag = 1;
                recognize_fail_count = 0;
                std::cout << "[系统] 已强制唤醒！" << std::endl;
                break;

            case 2: // 任务类型语音
                if (!awake_flag) {
                    std::cout << "[警告] 系统未唤醒，请先唤醒！" << std::endl;
                    break;
                }
                {
                    int index;
                    std::cout << "输入类型索引（0-2）：";
                    std::cin >> index;
                    if (index >= 0 && index < TASK_VOICE.size()) {
                        play_audio(TASK_VOICE[index]);
                    } else {
                        std::cout << "[错误] 索引超出范围！" << std::endl;
                    }
                }
                break;

            // 其他功能选项（3-7）与 case 2 逻辑类似，简化编写
            case 3: // 物品获取语音
                if (!awake_flag) {
                    std::cout << "[警告] 系统未唤醒，请先唤醒！" << std::endl;
                    break;
                }
                {
                    int index;
                    std::cout << "输入物品索引（0-8）：";
                    std::cin >> index;
                    if (index >= 0 && index < GET_VOICE.size()) {
                        play_audio(GET_VOICE[index]);
                    }
                }
                break;

            case 4: // 房间提示语音
                if (!awake_flag) {
                    std::cout << "[警告] 系统未唤醒，请先唤醒！" << std::endl;
                    break;
                }
                {
                    int index;
                    std::cout << "输入房间索引（0-2）：";
                    std::cin >> index;
                    if (index >= 0 && index < ROOM_VOICE.size()) {
                        play_audio(ROOM_VOICE[index]);
                        room_index = index;
                    }
                }
                break;

            case 5: // 路径提示语音
                if (!awake_flag) {
                    std::cout << "[警告] 系统未唤醒，请先唤醒！" << std::endl;
                    break;
                }
                {
                    int index;
                    std::cout << "输入路径索引（0-1）：";
                    std::cin >> index;
                    if (index >= 0 && index < PATH_VOICE.size()) {
                        play_audio(PATH_VOICE[index]);
                    }
                }
                break;

            case 6: // 费用语音
                if (!awake_flag) {
                    std::cout << "[警告] 系统未唤醒，请先唤醒！" << std::endl;
                    break;
                }
                {
                    int index;
                    std::cout << "输入费用索引（0-16）：";
                    std::cin >> index;
                    if (index >= 0 && index < COST_VOICE.size()) {
                        play_audio(COST_VOICE[index]);
                    }
                }
                break;

            case 7: // 切换房间
                if (!awake_flag) {
                    std::cout << "[警告] 系统未唤醒，请先唤醒！" << std::endl;
                    break;
                }
                {
                    int room;
                    std::cout << "输入房间号（0-3）：";
                    std::cin >> room;
                    if (room >= 0 && room <= 3) {
                        room_index = room;
                        std::cout << "[系统] 当前房间：" << room_index << std::endl;
                        // 根据房间号播放语音
                        if (room < 3 && room < ROOM_VOICE.size()) {
                            play_audio(ROOM_VOICE[room]);
                        } else if (room == 3 && !PATH_VOICE.empty()) {
                            play_audio(PATH_VOICE[0]); // 房间3默认播放路径1
                        }
                    }
                }
                break;

            default:
                std::cout << "[错误] 无效选项，请重新输入！" << std::endl;
                break;
        }
    }
}

/**
 * @brief 语音唤醒处理函数
 * @param set_awake_word_client 设置唤醒词的服务客户端
 * @param get_offline_result_client 语音识别结果的服务客户端
 * @return 唤醒成功返回 true，失败返回 false
 */
bool process_voice_wakeup(ros::ServiceClient& set_awake_word_client, 
                         ros::ServiceClient& get_offline_result_client) {
    // 设置唤醒词（可通过 ROS 参数动态配置，此处硬编码）
    broadcast::Set_Awake_Word_srv set_srv;
    set_srv.request.awake_word = "小车小车快速出发执行任务"; // 核心唤醒词

    if (!set_awake_word_client.call(set_srv)) {
        ROS_ERROR("[唤醒失败] 设置唤醒词服务调用失败");
        return false;
    }
    if (set_srv.response.result != "ok") {
        ROS_ERROR("[唤醒失败] 设置唤醒词失败：%s", set_srv.response.result.c_str());
        return false;
    }
    ROS_INFO("[唤醒模块] 唤醒词已设置：%s", set_srv.request.awake_word.c_str());

    // 配置语音识别参数
    broadcast::Get_Offline_Result_srv get_srv;
    get_srv.request.offline_recognise_start = 1;         // 启动识别
    get_srv.request.confidence_threshold = 30;           // 置信度阈值（0-100，越高越严格）
    get_srv.request.time_per_order = 5;                  // 单次识别超时时间（秒）

    ROS_INFO("[唤醒模块] 等待唤醒词...（说出：%s）", set_srv.request.awake_word.c_str());

    while (ros::ok() && !sig_flag) { // 循环条件：ROS运行中且未接收到退出信号
        if (!get_offline_result_client.call(get_srv)) {
            ROS_ERROR("[唤醒失败] 语音识别服务调用失败");
            return false;
        }

        if (get_srv.response.result == "ok") {
            std::string recognition_text = get_srv.response.text;
            ROS_INFO("[识别结果] %s", recognition_text.c_str());

            // 检测唤醒词是否包含在识别结果中
            if (recognition_text.find(set_srv.request.awake_word) != std::string::npos) {
                ROS_INFO("[唤醒成功] 系统已唤醒！");
                play_audio(WAKEUP_SOUND); // 播放唤醒提示音
                awake_flag = 1;
                recognize_fail_count = 0;
                return true;
            }
        } else {
            recognize_fail_count++;
            ROS_INFO("[唤醒提示] 识别失败（%d/%d）", recognize_fail_count, recognize_fail_threshold);
            if (recognize_fail_count >= recognize_fail_threshold) {
                ROS_WARN("[系统提示] 连续识别失败，进入休眠状态");
                recognize_fail_count = 0;
                return false; // 退出本次唤醒流程，等待下次触发
            }
        }

        ros::spinOnce();
        ros::Rate(1).sleep(); // 1Hz循环频率
    }

    return false;
}

// ======================
// 主函数
// ======================
int main(int argc, char **argv) {
    ros::init(argc, argv, "broadcast_node"); // ROS节点初始化（节点名需唯一）
    ros::NodeHandle nh;

    // 注册 Ctrl+C 信号处理函数
    signal(SIGINT, sig_handler);

    std::cout << "\n=== 语音播报系统 v1.0 ===" << std::endl;
    std::cout << "作者：Your Name" << std::endl;
    std::cout << "启动时间：" << __TIME__ << " " << __DATE__ << std::endl;
    std::cout << "=================================\n" << std::endl;

    // 创建 ROS 服务客户端（需与 broadcast 包中的服务匹配）
    ros::ServiceClient set_awake_word_client = 
        nh.serviceClient<broadcast::Set_Awake_Word_srv>("/broadcast_node/set_awake_word"); // 建议与节点名一致的服务路径

    ros::ServiceClient get_offline_result_client = 
        nh.serviceClient<broadcast::Get_Offline_Result_srv>("/broadcast_node/get_offline_result");

    // 启动多线程
    std::thread wakeup_thread([&]() {
        while (ros::ok() && !sig_flag) {
            if (!awake_flag) { // 仅在休眠状态下监听唤醒
