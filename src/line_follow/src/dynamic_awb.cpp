#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <chrono>

using namespace cv;
using namespace std;

void dynamic_awb(Mat& im) {
    // 确保输入是32位浮点型三通道图像
    CV_Assert(im.type() == CV_32FC3);
    
    // 分离通道
    vector<Mat> channels(3);
    split(im, channels);
    Mat R = channels[2], G = channels[1], B = channels[0];
    
    // 预分配输出矩阵
    Mat Y(im.size(), CV_32F), Cb(im.size(), CV_32F), Cr(im.size(), CV_32F);
    
    // 计算YCbCr分量
    Y = 0.257f * R + 0.504f * G + 0.098f * B + 16.0f/255;
    Cb = -0.148f * R - 0.291f * G + 0.439f * B + 128.0f/255;
    Cr = -0.439f * R - 0.368f * G - 0.071f * B + 128.0f/255;
    
    // 计算统计量
    Scalar mb = mean(Cb), mr = mean(Cr);
    Mat absCb, absCr;
    absdiff(Cb, mb[0], absCb);
    absdiff(Cr, mr[0], absCr);
    Scalar db = mean(absCb), dr = mean(absCr);
    
    // 白点筛选
    Mat bv = abs(Cb - (mb[0] + db[0] * (mb[0] > (128.0f/255) ? 1.0f : -1.0f)));
    Mat rv = abs(Cr - (1.5f * mr[0] + dr[0] * (mr[0] > (128.0f/255) ? 1.0f : -1.0f)));
    Mat J = (bv < 1.5f*db[0]) & (rv < 1.5f*dr[0]);
    
    // 亮度筛选
    Mat Y_filtered;
    Y.copyTo(Y_filtered, J);
    
    // 收集候选点
    vector<float> candidate;
    for(int i = 0; i < Y_filtered.rows; i++) {
        for(int j = 0; j < Y_filtered.cols; j++) {
            float val = Y_filtered.at<float>(i,j);
            if(val > 0) {
                candidate.push_back(val);
            }
        }
    }
    
    // 排序并获取阈值
    sort(candidate.begin(), candidate.end(), greater<float>());
    float min_v = candidate.empty() ? 0 : candidate[candidate.size()*0.1];
    
    // 创建亮度掩模
    Mat Y1 = Y > min_v;
    
    // 计算增益
    Scalar ravg = sum(R.mul(Y1)) / sum(Y1)[0];
    Scalar gavg = sum(G.mul(Y1)) / sum(Y1)[0];
    Scalar bavg = sum(B.mul(Y1)) / sum(Y1)[0];
    double ymax;
    minMaxLoc(Y, nullptr, &ymax);
    
    // 应用增益
    channels[2] = R * (ymax/(ravg[0]+1e-6f));
    channels[1] = G * (ymax/(gavg[0]+1e-6f));
    channels[0] = B * (ymax/(bavg[0]+1e-6f));
    
    // 截断处理
    threshold(channels[2], channels[2], 1.0, 1.0, THRESH_TRUNC);
    threshold(channels[1], channels[1], 1.0, 1.0, THRESH_TRUNC);
    threshold(channels[0], channels[0], 1.0, 1.0, THRESH_TRUNC);
    
    threshold(channels[2], channels[2], 0.0, 0.0, THRESH_TOZERO);
    threshold(channels[1], channels[1], 0.0, 0.0, THRESH_TOZERO);
    threshold(channels[0], channels[0], 0.0, 0.0, THRESH_TOZERO);
    
    // 合并通道
    merge(channels, im);
}

int main() {
    auto start = chrono::high_resolution_clock::now();
    
    string input_path = "/home/ucar/ucar_car/ypicture/picture_130.jpg";
    string output_path = "/home/ucar/ucar_car/ypicture/picture_130_awb4.jpg";
    
    // 读取图像
    Mat I = imread(input_path, IMREAD_COLOR);
    if(I.empty()) {
        cout << "图片加载失败" << endl;
        return -1;
    }
    
    // 转换类型并处理
    I.convertTo(I, CV_32FC3, 1.0/255);
    dynamic_awb(I);
    I.convertTo(I, CV_8UC3, 255);
    
    // 保存结果
    imwrite(output_path, I);
    
    auto end = chrono::high_resolution_clock::now();
    chrono::duration<double, milli> elapsed = end - start;
    cout << "耗时：" << elapsed.count() << "ms" << endl;
    
    return 0;
}

// catkin_make --only-pkg-with-deps line_follow //vscode
// rosrun line_follow dynamic_awb //mobax

// #include <opencv2/opencv.hpp>
// #include <vector>
// #include <algorithm>
// #include <cmath>
// #include <chrono>

// using namespace cv;
// using namespace std;

// void dynamic_awb(Mat& im) {
//     // 分离通道
//     vector<Mat> channels;
//     split(im, channels);
//     Mat R = channels[2], G = channels[1], B = channels[0];
    
    // // 计算YCbCr
    // Mat Y = (0.257f * R + 0.504f * G + 0.098f * B) + (16.0f/255);
    // Mat Cb = (-0.148f * R - 0.291f * G + 0.439f * B) + (128.0f/255);
    // Mat Cr = (-0.439f * R - 0.368f * G - 0.071f * B) + (128.0f/255);
    
//     // 全局统计
//     Scalar mb = mean(Cb), mr = mean(Cr);
    
//     Mat absCb, absCr;
//     absdiff(Cb, mb[0], absCb);
//     absdiff(Cr, mr[0], absCr);
//     Scalar db = mean(absCb), dr = mean(absCr);
    
//     // 向量化白点筛选
//     Mat bv = abs(Cb - (mb[0] + db[0] * (mb[0] > 128.0/255 ? 1 : -1)));
//     Mat rv = abs(Cr - (1.5 * mr[0] + dr[0] * (mr[0] > 128.0/255 ? 1 : -1)));
//     Mat J = (bv < 1.5*db[0]) & (rv < 1.5*dr[0]);
    
//     // 亮度筛选
//     Mat Y_filtered;
//     Y.copyTo(Y_filtered, J);
//     vector<float> candidate;
//     for(int i=0; i<Y_filtered.rows; i++) {
//         for(int j=0; j<Y_filtered.cols; j++) {
//             if(Y_filtered.at<float>(i,j) > 0) {
//                 candidate.push_back(Y_filtered.at<float>(i,j));
//             }
//         }
//     }
//     sort(candidate.begin(), candidate.end(), greater<float>());
//     float min_v = candidate.empty() ? 0 : candidate[candidate.size()*0.1];
//     Mat Y1 = Y > min_v;
    
//     // 增益计算
//     Scalar ravg = sum(R.mul(Y1)) / sum(Y1)[0];
//     Scalar gavg = sum(G.mul(Y1)) / sum(Y1)[0];
//     Scalar bavg = sum(B.mul(Y1)) / sum(Y1)[0];
//     double ymax;
//     minMaxLoc(Y, nullptr, &ymax);
    
//     // 应用增益
//     channels[2] = min(1.0, max(0.0, R * (ymax/(ravg[0]+1e-6))));
//     channels[1] = min(1.0, max(0.0, G * (ymax/(gavg[0]+1e-6))));
//     channels[0] = min(1.0, max(0.0, B * (ymax/(bavg[0]+1e-6))));
    
//     merge(channels, im);
// }

// int main() {
//     auto start = chrono::high_resolution_clock::now();
    
//     // 输入输出路径
//     string input_path = "/home/ucar/ucar_car/ypicture/picture_130.jpg";
//     string output_path = "/home/ucar/ucar_car/ypicture/picture_130_awb3.jpg";
    
//     // 读取图像
//     Mat I = imread(input_path, IMREAD_COLOR);
//     if(I.empty()) {
//         cout << "图片加载失败" << endl;
//         return -1;
//     }
    
//     // 转换类型并处理
//     I.convertTo(I, CV_32F, 1.0/255);
//     dynamic_awb(I);
//     I.convertTo(I, CV_8U, 255);
    
//     // 保存结果
//     imwrite(output_path, I);
    
//     auto end = chrono::high_resolution_clock::now();
//     chrono::duration<double, milli> elapsed = end - start;
//     cout << "耗时：" << elapsed.count() << "ms" << endl;
    
//     return 0;
// }