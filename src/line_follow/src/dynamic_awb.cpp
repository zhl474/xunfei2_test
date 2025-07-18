#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <chrono>

using namespace cv;
using namespace std;

void dynamic_awb(Mat& im) {
    CV_Assert(im.type() == CV_32FC3);
    
    vector<Mat> channels(3);
    split(im, channels);
    Mat R = channels[2], G = channels[1], B = channels[0];
    
    // === 显式分步计算 ===
    Mat Y(im.size(), CV_32F), Cb(im.size(), CV_32F), Cr(im.size(), CV_32F);
    Mat temp1, temp2;
    
    // 计算Y分量
    multiply(R, 0.257f, temp1, 1, CV_32F);
    multiply(G, 0.504f, temp2, 1, CV_32F);
    addWeighted(temp1, 1.0, temp2, 1.0, 16.0f/255, Y, CV_32F);
    multiply(B, 0.098f, temp1, 1, CV_32F);
    addWeighted(Y, 1.0, temp1, 1.0, 0.0, Y, CV_32F);
    
    // 计算Cb分量
    multiply(R, -0.148f, temp1, 1, CV_32F);
    multiply(G, -0.291f, temp2, 1, CV_32F);
    addWeighted(temp1, 1.0, temp2, 1.0, 128.0f/255, Cb, CV_32F);
    multiply(B, 0.439f, temp1, 1, CV_32F);
    addWeighted(Cb, 1.0, temp1, 1.0, 0.0, Cb, CV_32F);
    
    // 计算Cr分量
    multiply(R, -0.439f, temp1, 1, CV_32F);
    multiply(G, -0.368f, temp2, 1, CV_32F);
    addWeighted(temp1, 1.0, temp2, 1.0, 128.0f/255, Cr, CV_32F);
    multiply(B, -0.071f, temp1, 1, CV_32F);
    addWeighted(Cr, 1.0, temp1, 1.0, 0.0, Cr, CV_32F);
    
    // === 统计计算 ===
    Scalar mb = mean(Cb), mr = mean(Cr);
    Mat absCb, absCr;
    absdiff(Cb, mb[0], absCb);
    absdiff(Cr, mr[0], absCr);
    Scalar db = mean(absCb), dr = mean(absCr);
    
    // === 白点筛选 ===
    Mat bv, rv, J1, J2, J;
    absdiff(Cb, mb[0] + db[0] * (mb[0] > (128.0f/255) ? 1.0f : -1.0f), bv);
    absdiff(Cr, 1.5f * mr[0] + dr[0] * (mr[0] > (128.0f/255) ? 1.0f : -1.0f), rv);
    compare(bv, 1.5f*db[0], J1, CMP_LT);
    compare(rv, 1.5f*dr[0], J2, CMP_LT);
    bitwise_and(J1, J2, J);
    
    // === 亮度筛选 ===
    Mat Y_filtered;
    Y.copyTo(Y_filtered, J);
    
    vector<float> candidate;
    for(int i = 0; i < Y_filtered.rows; i++) {
        for(int j = 0; j < Y_filtered.cols; j++) {
            float val = Y_filtered.at<float>(i,j);
            if(val > 0) candidate.push_back(val);
        }
    }
    
    sort(candidate.begin(), candidate.end(), greater<float>());
    float min_v = candidate.empty() ? 0 : candidate[candidate.size()*0.1];
    
    Mat Y1;
    compare(Y, min_v, Y1, CMP_GT);
    Y1.convertTo(Y1, CV_32F, 1.0/255);
    
    // === 增益计算 ===
    Scalar ravg = sum(R.mul(Y1)) / (sum(Y1)[0] + 1e-6f);
    Scalar gavg = sum(G.mul(Y1)) / (sum(Y1)[0] + 1e-6f);
    Scalar bavg = sum(B.mul(Y1)) / (sum(Y1)[0] + 1e-6f);
    double ymax;
    minMaxLoc(Y, nullptr, &ymax);
    
    // === 应用增益 ===
    multiply(R, ymax/(ravg[0]+1e-6f), channels[2], 1, CV_32F);
    multiply(G, ymax/(gavg[0]+1e-6f), channels[1], 1, CV_32F);
    multiply(B, ymax/(bavg[0]+1e-6f), channels[0], 1, CV_32F);
    
    threshold(channels[2], channels[2], 1.0, 1.0, THRESH_TRUNC);
    threshold(channels[1], channels[1], 1.0, 1.0, THRESH_TRUNC);
    threshold(channels[0], channels[0], 1.0, 1.0, THRESH_TRUNC);
    
    threshold(channels[2], channels[2], 0.0, 0.0, THRESH_TOZERO);
    threshold(channels[1], channels[1], 0.0, 0.0, THRESH_TOZERO);
    threshold(channels[0], channels[0], 0.0, 0.0, THRESH_TOZERO);
    
    merge(channels, im);
}

int main() {
    auto start = chrono::high_resolution_clock::now();
    
    string input_path = "/home/ucar/ucar_car/ypicture/picture_130.jpg";
    string output_path = "/home/ucar/ucar_car/ypicture/picture_130_awb4.jpg";
    
    Mat I = imread(input_path, IMREAD_COLOR);
    if(I.empty()) {
        cout << "图片加载失败" << endl;
        return -1;
    }
    
    I.convertTo(I, CV_32FC3, 1.0/255);
    dynamic_awb(I);
    I.convertTo(I, CV_8UC3, 255);
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
    
//     // 计算YCbCr
//     Mat Y = (0.257f * R + 0.504f * G + 0.098f * B) + (16.0f/255);
//     Mat Cb = (-0.148f * R - 0.291f * G + 0.439f * B) + (128.0f/255);
//     Mat Cr = (-0.439f * R - 0.368f * G - 0.071f * B) + (128.0f/255);
    
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