#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

using namespace cv;
using namespace std;
using namespace std::chrono;

// 快速白平衡（灰度世界法 + HSV 滤波保留黄色）
void white_balance_3_fast(Mat& img) {
    CV_Assert(img.type() == CV_8UC3);

    // 转换到 HSV 空间（仅用于滤波）
    Mat hsv;
    cvtColor(img, hsv, COLOR_BGR2HSV);

    // 定义黄色范围（H:20~30，S/V 可调）
    Scalar lower_yellow = Scalar(20, 100, 100);
    Scalar upper_yellow = Scalar(30, 255, 255);
    Mat yellow_mask;
    inRange(hsv, lower_yellow, upper_yellow, yellow_mask);

    // 创建非黄色区域的掩膜
    Mat non_yellow_mask;
    bitwise_not(yellow_mask, non_yellow_mask);

    // 计算非黄色区域的平均值
    Scalar avg = mean(img, non_yellow_mask);
    double k = (avg[0] + avg[1] + avg[2]) / 3.0;

    // 增益计算
    double kb = k / avg[0];
    double kg = k / avg[1];
    double kr = k / avg[2];

    // 手动调增益（可选）
    kb *= 1.2;  // 加强蓝色通道（去黄）
    kr *= 0.9;  // 减弱红色通道（去黄）

    // 分离通道（向量化）
    vector<Mat> channels(3);
    split(img, channels);

    // 应用增益（向量化）
    channels[0].convertTo(channels[0], CV_32F, kb);
    channels[1].convertTo(channels[1], CV_32F, kg);
    channels[2].convertTo(channels[2], CV_32F, kr);

    // 向量化限制范围 [0, 255]
    min(channels[0], Scalar(255), channels[0]); max(channels[0], Scalar(0), channels[0]);
    min(channels[1], Scalar(255), channels[1]); max(channels[1], Scalar(0), channels[1]);
    min(channels[2], Scalar(255), channels[2]); max(channels[2], Scalar(0), channels[2]);

    // 转换回 8U
    channels[0].convertTo(channels[0], CV_8U);
    channels[1].convertTo(channels[1], CV_8U);
    channels[2].convertTo(channels[2], CV_8U);

    merge(channels, img);
}

// 主函数（ROS 兼容）
int main() {
    string input_path = "/home/ucar/ucar_car/ypicture/picture_383.jpg";
    string output_path = "/home/ucar/ucar_car/ypicture/picture_383_re.jpg";

    Mat img = imread(input_path, IMREAD_COLOR);
    if (img.empty()) {
        cout << "无法加载图像: " << input_path << endl;
        return -1;
    }

    // 确保图像是 CV_8UC3 类型
    Mat img_8uc3;
    if (img.type() != CV_8UC3) {
        if (img.channels() == 1) {
            cvtColor(img, img_8uc3, COLOR_GRAY2BGR);
        } else {
            img.convertTo(img_8uc3, CV_8UC3);
        }
    } else {
        img_8uc3 = img;
    }

    // 耗时统计
    auto start = high_resolution_clock::now();

    // 白平衡处理
    white_balance_3_fast(img_8uc3);

    auto end = high_resolution_clock::now();
    duration<double, milli> elapsed = end - start;
    cout << "耗时：" << elapsed.count() << "ms" << endl;

    // 保存结果
    imwrite(output_path, img_8uc3);
    cout << "结果已保存至: " << output_path << endl;

    return 0;
}
