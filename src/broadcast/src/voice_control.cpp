#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <vector>

// 全局变量定义
int room_index = 0;       // 当前房间号
int awake_flag = 0;      // 语音唤醒标志位
// 语音命令数组（二维数组，按类别分组）
std::vector<std::vector<std::string>> voice = {
    // 第0组：任务类型（甜点/水果/蔬菜）
    {
        "aplay ~/ucar_car/src/broadcast/20_wav/1_task_dessert.wav",
        "aplay ~/ucar_car/src/broadcast/20_wav/1_task_fruit.wav",
        "aplay ~/ucar_car/src/broadcast/20_wav/1_task_vegetable.wav"
    },
    // 第1组：物品获取（苹果/香蕉/蛋糕等）
    {
        "aplay ~/ucar_car/src/broadcast/20_wav/2_get_apple.wav",
        "aplay ~/ucar_car/src/broadcast/20_wav/2_get_banana.wav",
        "aplay ~/ucar_car/src/broadcast/20_wav/2_get_cake.wav",
        "aplay ~/ucar_car/src/broadcast/20_wav/2_get_chili.wav",
        "aplay ~/ucar_car/src/broadcast/20_wav/2_get_cola.wav",
        "aplay ~/ucar_car/src/broadcast/20_wav/2_get_milk.wav",
        "aplay ~/ucar_car/src/broadcast/20_wav/2_get_poatato.wav",
        "aplay ~/ucar_car/src/broadcast/20_wav/2_get_tomato.wav",
        "aplay ~/ucar_car/src/broadcast/20_wav/2_get_watermelon.wav"
    },
    // 第2组：房间提示（gaozeboA/B/C）
    {
        "aplay ~/ucar_car/src/broadcast/20_wav/3_gaozebo_A.wav",
        "aplay ~/ucar_car/src/broadcast/20_wav/3_gaozebo_B.wav",
        "aplay ~/ucar_car/src/broadcast/20_wav/3_gaozebo_C.wav"
    },
    // 第3组：路径提示（道路1/道路2）
    {
        "aplay ~/ucar_car/src/broadcast/20_wav/4_road_1.wav",
        "aplay ~/ucar_car/src/broadcast/20_wav/4_road_2.wav"
    },
     //第4组：费用提示
    {
    "aplay ~/ucar_car/src/broadcast/20_wav/5_chili_chili.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_chili_tomato.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_chili_potato.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_tomato_tomato.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_tomato_potato.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_potato_potato.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_banana_banana.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_apple_banana.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_banana_watermelon.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_apple_apple.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_apple_watermelon.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_watermelon_watermelon.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_cola_cola.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_cake_cola.wav",    
    "aplay ~/ucar_car/src/broadcast/20_wav/5_milk_cola.wav",  
    "aplay ~/ucar_car/src/broadcast/20_wav/5_cake_cake.wav",
    "aplay ~/ucar_car/src/broadcast/20_wav/5_milk_cake.wav",    
    "aplay ~/ucar_car/src/broadcast/20_wav/5_milk_milk.wav"    
    }
};

// // 费用相关语音（一维数组）
// std::vector<std::string> cost = {
//     "aplay ~/ucar_car/src/broadcast/20_wav/5_apple_apple.wav",
//     "aplay ~/ucar_car/src/broadcast/20_wav/5_apple_banana.wav",
//     "aplay ~/ucar_car/src/broadcast/20_wav/5_apple_watermelon.wav",
//     "aplay ~/ucar_car/src/broadcast/20_wav/5_banana_banana.wav",
//     "aplay ~/ucar_car/src/broadcast/20_wav/5_banana_watermelon.wav",
//     "aplay ~/ucar_car/src/broadcast/20_wav/5_cake_cola.wav",
//     "aplay ~/ucar_car/src/broadcast/20_wav/5_chili_chili.wav",
//     "aplay ~/ucar_car/src/broadcast/20_wav/5_chili_potato.wav",
//     "aplay ~/ucar_car/src/broadcast/20_wav/5_chili_tomato.wav",
//     "aplay ~/ucar_car/src/broadcast/20_wav/5_cola_cola.wav",
//     "aplay ~/ucar_car/src/broadcast/20_wav/5_milk_cake.wav",
//     "aplay ~/ucar_car/src/broadcast/20_wav/5_milk_cola.wav",
//     "aplay ~/ucar_car/src/broadcast/20_wav/5_milk_milk.wav",
//     "aplay ~/ucar_car/src/broadcast/20_wav/5_potato_potato.wav",
//     "aplay ~/ucar_car/src/broadcast/20_wav/5_tomato_potato.wav",
//     "aplay ~/ucar_car/src/broadcast/20_wav/5_tomato_tomato.wav",
//     "aplay ~/ucar_car/src/broadcast/20_wav/5_watermelon_watermelon.wav"
// };

// 播放语音函数
void play_audio(const std::string& command) {
    system(command.c_str()); // 直接调用系统命令播放音频
}

// 命令行交互函数
void command_interaction() {
    while (true) {
        std::cout << "\n=== 语音播报控制 ===" << std::endl;
        std::cout << "1. 唤醒提示音" << std::endl;
        std::cout << "2. 播放任务类型语音（0-2）" << std::endl;
        std::cout << "3. 播放物品获取语音（0-8）" << std::endl;
        std::cout << "4. 播放房间提示语音（0-2）" << std::endl;
        std::cout << "5. 播放路径提示语音（0-1）" << std::endl;
        std::cout << "6. 播放费用语音（0-16）" << std::endl;
        std::cout << "7. 切换房间（0-3）" << std::endl;
        std::cout << "0. 退出" << std::endl;
        std::cout << "请选择操作：";

        int choice;
        std::cin >> choice;
        std::cin.ignore(); // 清理输入缓冲区

        switch (choice) {
            case 1: // 唤醒提示音
                play_audio("aplay ~/ucar_car/src/broadcast/20_wav/wakeup_success.wav");
                awake_flag = 1;
                break;

            case 2: { // 任务类型（组0）
                int index;
                std::cout << "输入类型索引（0-2）：";
                std::cin >> index;
                if (index >= 0 && index < voice[0].size()) {
                    play_audio(voice[0][index]);
                }
                break;
            }

            case 3: { // 物品获取（组1）
                int index;
                std::cout << "输入物品索引（0-8）：";
                std::cin >> index;
                if (index >= 0 && index < voice[1].size()) {
                    play_audio(voice[1][index]);
                }
                break;
            }

            case 4: { // 房间提示（组2）
                int index;
                std::cout << "输入房间索引（0-2）：";
                std::cin >> index;
                if (index >= 0 && index < voice[2].size()) {
                    play_audio(voice[2][index]);
                    room_index = index; // 更新房间号
                }
                break;
            }

            case 5: { // 路径提示（组3）
                int index;
                std::cout << "输入路径索引（0-1）：";
                std::cin >> index;
                if (index >= 0 && index < voice[3].size()) {
                    play_audio(voice[3][index]);
                }
                break;
            }

            case 6: { // 费用语音
                int a,b;
                // std::cout << "输入费用索引（0-16）：";
                // std::cin >> index;
                std::cout << "输入对应物品1编码：";
                std::cin >> a;
                std::cout << "输入对应物品2编码：";
                std::cin >> b;
                
                    if(a == 0 &&b == 0 )
                    play_audio(voice[4][0]);

                     if(a == 1 &&b == 1 )
                     play_audio(voice[4][1]);

                     if(a == 2 &&b == 2)
                     play_audio(voice[4][2]);

                     if(a == 3 &&b == 3)
                     play_audio(voice[4][3]);

                     if(a == 4 &&b == 4)
                     play_audio(voice[4][4]);

                     if(a == 5 &&b == 5)
                     play_audio(voice[4][5]);

                     if(a == 6 &&b == 6)
                     play_audio(voice[4][6]);

                     if(a == 7 &&b == 7)
                     play_audio(voice[4][7]);

                     if(a == 8 &&b == 8)
                     play_audio(voice[4][8]);

                     if((a == 0 && b == 1) || (a == 1 && b == 0))
                    play_audio(voice[4][9]);

                    if((a == 0 && b == 2) || (a == 2 && b == 0))
                        play_audio(voice[4][10]);

                    if((a == 1 && b == 2) || (a == 2 && b == 1))
                        play_audio(voice[4][11]);

                    if((a == 3 && b == 4) || (a == 4 && b == 3))
                        play_audio(voice[4][12]);

                    if((a == 3 && b == 5) || (a == 5 && b == 3))
                        play_audio(voice[4][13]);

                    if((a == 4 && b == 5) || (a == 5 && b == 4))
                        play_audio(voice[4][14]);

                    if((a == 6 && b == 7) || (a == 7 && b == 6))
                        play_audio(voice[4][15]);

                    if((a == 6 && b == 8) || (a == 8 && b == 6))
                        play_audio(voice[4][16]);

                    if((a == 7 && b == 8) || (a == 8 && b == 7))
                        play_audio(voice[4][17]);
                break;
            }

            case 7: { // 切换房间（直接使用房间号控制）
                int room;
                std::cout << "输入房间号（0-3）：";
                std::cin >> room;
                if (room >= 0 && room <= 3) {
                    room_index = room;
                    std::cout << "当前房间：" << room_index << std::endl;
                    // 根据房间号播放对应语音（组2和组3）
                    if (room < 3 && room < voice[2].size()) {
                        play_audio(voice[2][room]);
                    } else if (room == 3 && voice[3].size() > 0) {
                        play_audio(voice[3][0]); // 房间3默认播放路径1
                    }
                }
                break;
            }

            case 0: // 退出
                std::cout << "退出程序" << std::endl;
                return;

            default:
                std::cout << "无效选择，请重新输入" << std::endl;
        }
    }
}

int main(int argc, char **argv) {
    // setlocale(LC_ALL,"");
    ros::init(argc, argv, "voice_control");
    ros::NodeHandle nh;

    std::cout << "=== 语音播报系统启动 ===" << std::endl;
    command_interaction(); // 启动命令行交互

    return 0;
}