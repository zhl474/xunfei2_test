#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <chrono>

using namespace cv;
using namespace std;
using namespace std::chrono;


// 优化5: 使用矩阵运算和OpenCV内置函数
Mat white_balance_5(const Mat& img) {
    Mat yuv;
    cvtColor(img, yuv, COLOR_BGR2YCrCb);
    vector<Mat> yuv_channels;
    split(yuv, yuv_channels);
    Mat y = yuv_channels[0], u = yuv_channels[1], v = yuv_channels[2];
    
    // 使用mean计算平均值
    Scalar mean_u = mean(u);
    Scalar mean_v = mean(v);
    double avl_u = mean_u[0];
    double avl_v = mean_v[0];
    
    // 计算绝对偏差
    Mat du = abs(u - avl_u);
    Mat dv = abs(v - avl_v);
    double avl_du = mean(du)[0];
    double avl_dv = mean(dv)[0];
    
    // 使用矩阵运算创建掩码
    double sign_u = avl_u < 0 ? -1.0 : 1.0;
    double sign_v = avl_v < 0 ? -1.0 : 1.0;
    double target_u = avl_u + avl_du * sign_u;
    double target_v = avl_v + avl_dv * sign_v;
    
    Mat cond_u = abs(u - target_u) < (0.5 * avl_du);
    Mat cond_v = abs(v - target_v) < (0.5 * avl_dv);
    Mat mask = cond_u | cond_v;
    
    Mat num_y = Mat::zeros(y.size(), CV_8UC1);
    y.copyTo(num_y, mask);
    
    // 直方图计算
    const int histSize = 256;
    float range[] = {0, 256};
    const float* histRange = {range};
    Mat hist;
    calcHist(&num_y, 1, 0, Mat(), hist, 1, &histSize, &histRange);
    
    int ysum = countNonZero(num_y);
    int Y = 255;
    int cumSum = 0;
    int key = 0;
    
    while (Y >= 0) {
        cumSum += static_cast<int>(hist.at<float>(Y));
        if (cumSum > 0.1 * ysum) {
            key = Y;
            break;
        }
        Y--;
    }
    
    // 创建选择掩码
    Mat select_mask = (num_y > key);
    
    // 计算平均值
    Scalar avg = mean(img, select_mask);
    double avg_b = avg[0], avg_g = avg[1], avg_r = avg[2];
    
    // 矩阵运算调整像素值
    vector<Mat> channels;
    split(img, channels);
    
    channels[0].convertTo(channels[0], CV_32F);
    channels[1].convertTo(channels[1], CV_32F);
    channels[2].convertTo(channels[2], CV_32F);
    
    channels[0] *= 255.0 / avg_b;
    channels[1] *= 255.0 / avg_g;
    channels[2] *= 255.0 / avg_r;
    
    Mat result;
    merge(channels, result);
    result.convertTo(result, CV_8UC3);
    return result;
}

// 主函数
int main() {
    string input_path = "/home/ucar/ucar_car/ypicture/picture_130.jpg";
    string output_path = "/home/ucar/ucar_car/ypicture/picture_130_wb.jpg";

    // 加载图像
    Mat img = imread(input_path, IMREAD_COLOR);
    if (img.empty()) {
        cout << "无法加载图像: " << input_path << endl;
        return -1;
    }

    // 执行白平衡并测量时间
    high_resolution_clock::time_point start = high_resolution_clock::now();
    Mat result = white_balance_5(img);
    high_resolution_clock::time_point end = high_resolution_clock::now();
    
    auto elapsed = duration_cast<milliseconds>(end - start);
    cout << "白平衡处理耗时: " << elapsed.count() << " ms" << endl;

    // 保存结果
    if (!imwrite(output_path, result)) {
        cout << "无法保存结果图像: " << output_path << endl;
        return -1;
    }

    cout << "白平衡结果已保存至: " << output_path << endl;

    return 0;
}