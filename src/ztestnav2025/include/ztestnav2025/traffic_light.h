#ifndef TRAFFIC_LIGHT_H
#define TRAFFIC_LIGHT_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "ros/ros.h"
 
int detectTrafficLightStatus();
// 定义结构体存储机器人的位置和朝向
struct RobotPose {
    double x;        // 机器人的x坐标
    double y;        // 机器人的y坐标
    double heading;  // 机器人的朝向角度（度）
};
RobotPose calculate_destination(double cx, double cy, double slope, double square_size = 2.5);

// 语音命令数组（二维数组，按类别分组）
std::vector<std::vector<std::string>> voice = {
    // 第0组：任务类型（甜点/水果/蔬菜）
    {
        "aplay ~/ucar_car/src/broadcast/20_wav/1_task_vegetable.wav",
        "aplay ~/ucar_car/src/broadcast/20_wav/1_task_fruit.wav",
        "aplay ~/ucar_car/src/broadcast/20_wav/1_task_dessert.wav",
        
    },
    // 第1组：物品获取（苹果/香蕉/蛋糕等）
    {
        "aplay ~/ucar_car/src/broadcast/20_wav/2_get_chili.wav",
        "aplay ~/ucar_car/src/broadcast/20_wav/2_get_tomato.wav",
        "aplay ~/ucar_car/src/broadcast/20_wav/2_get_poatato.wav",  
        "aplay ~/ucar_car/src/broadcast/20_wav/2_get_banana.wav",
        "aplay ~/ucar_car/src/broadcast/20_wav/2_get_apple.wav",
        "aplay ~/ucar_car/src/broadcast/20_wav/2_get_watermelon.wav"
        "aplay ~/ucar_car/src/broadcast/20_wav/2_get_cola.wav",
        "aplay ~/ucar_car/src/broadcast/20_wav/2_get_cake.wav",
        "aplay ~/ucar_car/src/broadcast/20_wav/2_get_milk.wav",  
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
    }
};

// 费用相关语音（一维数组）
std::vector<std::string> cost = {
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

void play_audio(const std::string& command);

#endif