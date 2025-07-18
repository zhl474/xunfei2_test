#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

using namespace cv;
using namespace std;

Mat dynamic_awb(const Mat& input) {
    // 确保输入是3通道图像
    CV_Assert(input.channels() == 3);
    
    // 转换为32位浮点并归一化
    Mat image;
    input.convertTo(image, CV_32FC3, 1.0f/255.0f);
    
    // 分离通道 (BGR顺序)
    vector<Mat> channels(3);
    split(image, channels);
    Mat B = channels[0].clone();  // 显式克隆确保独立内存
    Mat G = channels[1].clone();
    Mat R = channels[2].clone();
    
    // 预分配结果矩阵
    Mat Y(image.size(), CV_32FC1);
    Mat Cb(image.size(), CV_32FC1);
    Mat Cr(image.size(), CV_32FC1);
    
    // 计算Y分量 (完全类型安全)
    Mat tmp1, tmp2, tmp3;
    multiply(R, Scalar(0.257f), tmp1, 1, CV_32F);
    multiply(G, Scalar(0.504f), tmp2, 1, CV_32F);
    multiply(B, Scalar(0.098f), tmp3, 1, CV_32F);
    add(tmp1, tmp2, Y, noArray(), CV_32F);
    add(Y, tmp3, Y, noArray(), CV_32F);
    add(Y, Scalar(16.0f/255.0f), Y, noArray(), CV_32F);
    
    // 计算Cb分量
    multiply(R, Scalar(-0.148f), tmp1, 1, CV_32F);
    multiply(G, Scalar(-0.291f), tmp2, 1, CV_32F);
    multiply(B, Scalar(0.439f), tmp3, 1, CV_32F);
    add(tmp1, tmp2, Cb, noArray(), CV_32F);
    add(Cb, tmp3, Cb, noArray(), CV_32F);
    add(Cb, Scalar(128.0f/255.0f), Cb, noArray(), CV_32F);
    
    // 计算Cr分量
    multiply(R, Scalar(-0.439f), tmp1, 1, CV_32F);
    multiply(G, Scalar(-0.368f), tmp2, 1, CV_32F);
    multiply(B, Scalar(-0.071f), tmp3, 1, CV_32F);
    add(tmp1, tmp2, Cr, noArray(), CV_32F);
    add(Cr, tmp3, Cr, noArray(), CV_32F);
    add(Cr, Scalar(128.0f/255.0f), Cr, noArray(), CV_32F);
    
    // 计算统计量
    Scalar Mb = mean(Cb);
    Scalar Mr = mean(Cr);
    
    Mat Cb_diff, Cr_diff;
    absdiff(Cb, Mb[0], Cb_diff);
    absdiff(Cr, Mr[0], Cr_diff);
    
    Scalar Db = mean(Cb_diff);
    Scalar Dr = mean(Cr_diff);
    
    // 计算白点条件
    float sign_Mb = Mb[0] > 0 ? 1.0f : -1.0f;
    float sign_Mr = Mr[0] > 0 ? 1.0f : -1.0f;
    
    Mat bv, rv;
    absdiff(Cb, (Mb[0] + Db[0] * sign_Mb), bv);
    absdiff(Cr, (1.5f * Mr[0] + Dr[0] * sign_Mr), rv);
    
    Mat J;
    compare(bv, 1.5f * Db[0], J, CMP_LT);
    Mat temp;
    compare(rv, 1.5f * Dr[0], temp, CMP_LT);
    bitwise_and(J, temp, J);
    
    // 亮度筛选
    Mat Y1;
    if (countNonZero(J) > 0) {
        Mat Y_J;
        Y.copyTo(Y_J, J);
        
        // 转换为单行进行排序
        Y_J = Y_J.reshape(1, 1);
        cv::sort(Y_J, Y_J, SORT_DESCENDING);
        
        int k = max(1, static_cast<int>(Y_J.cols * 0.1f));
        float min_v = Y_J.at<float>(0, k-1);
        
        compare(Y, min_v, Y1, CMP_GE);
    } else {
        Y1 = Mat::zeros(Y.size(), CV_8U);
    }
    
    // 计算和应用增益
    if (countNonZero(Y1) > 0) {
        Scalar Ravg = mean(R, Y1);
        Scalar Gavg = mean(G, Y1);
        Scalar Bavg = mean(B, Y1);
        
        double maxVal;
        minMaxLoc(Y, nullptr, &maxVal, nullptr, nullptr, Y1);
        float Ymax = static_cast<float>(maxVal);
        
        // 应用增益 (完全类型安全)
        multiply(R, Scalar(Ymax/(Ravg[0] + 1e-6f)), R, 1, CV_32F);
        multiply(G, Scalar(Ymax/(Gavg[0] + 1e-6f)), G, 1, CV_32F);
        multiply(B, Scalar(Ymax/(Bavg[0] + 1e-6f)), B, 1, CV_32F);
        
        // 裁剪到[0,1]范围
        threshold(R, R, 1.0f, 1.0f, THRESH_TRUNC);
        threshold(R, R, 0.0f, 0.0f, THRESH_TOZERO);
        threshold(G, G, 1.0f, 1.0f, THRESH_TRUNC);
        threshold(G, G, 0.0f, 0.0f, THRESH_TOZERO);
        threshold(B, B, 1.0f, 1.0f, THRESH_TRUNC);
        threshold(B, B, 0.0f, 0.0f, THRESH_TOZERO);
    }
    
    // 合并通道
    vector<Mat> merged_channels = {B, G, R};
    Mat result;
    merge(merged_channels, result);
    return result;
}

int main(int argc, char** argv) {
    string input_path = "/home/ucar/ucar_car/ypicture/picture_130.jpg";
    string output_path = "/home/ucar/ucar_car/ypicture/picture_130_awb_cpp.jpg";
    
    try {
        // 读取图像
        Mat input = imread(input_path, IMREAD_COLOR);
        if (input.empty()) {
            cerr << "错误: 无法加载图像 " << input_path << endl;
            return -1;
        }
        
        auto start = chrono::high_resolution_clock::now();
        
        // 处理图像
        Mat result = dynamic_awb(input);
        
        // 保存结果
        vector<int> compression_params = {IMWRITE_JPEG_QUALITY, 95};
        if (!imwrite(output_path, result * 255, compression_params)) {
            cerr << "错误: 无法保存图像 " << output_path << endl;
            return -1;
        }
        
        auto end = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsed = end - start;
        
        cout << "处理完成，耗时: " << elapsed.count() << "秒" << endl;
    } catch (const cv::Exception& e) {
        cerr << "OpenCV错误: " << e.what() << endl;
        return -1;
    } catch (const exception& e) {
        cerr << "错误: " << e.what() << endl;
        return -1;
    }
    
    return 0;
}


// #include <opencv2/opencv.hpp>
// #include <vector>
// #include <algorithm>
// #include <chrono>

// using namespace cv;
// using namespace std;

// void dynamic_awb(Mat& im) {
//     CV_Assert(im.type() == CV_32FC3);
    
//     vector<Mat> channels(3);
//     split(im, channels);
//     Mat R = channels[2], G = channels[1], B = channels[0];
    
//     // === 显式分步计算 ===
//     Mat Y(im.size(), CV_32F), Cb(im.size(), CV_32F), Cr(im.size(), CV_32F);
//     Mat temp1, temp2;
    
//     // 计算Y分量
//     multiply(R, 0.257f, temp1, 1, CV_32F);
//     multiply(G, 0.504f, temp2, 1, CV_32F);
//     addWeighted(temp1, 1.0, temp2, 1.0, 16.0f/255, Y, CV_32F);
//     multiply(B, 0.098f, temp1, 1, CV_32F);
//     addWeighted(Y, 1.0, temp1, 1.0, 0.0, Y, CV_32F);
    
//     // 计算Cb分量
//     multiply(R, -0.148f, temp1, 1, CV_32F);
//     multiply(G, -0.291f, temp2, 1, CV_32F);
//     addWeighted(temp1, 1.0, temp2, 1.0, 128.0f/255, Cb, CV_32F);
//     multiply(B, 0.439f, temp1, 1, CV_32F);
//     addWeighted(Cb, 1.0, temp1, 1.0, 0.0, Cb, CV_32F);
    
//     // 计算Cr分量
//     multiply(R, -0.439f, temp1, 1, CV_32F);
//     multiply(G, -0.368f, temp2, 1, CV_32F);
//     addWeighted(temp1, 1.0, temp2, 1.0, 128.0f/255, Cr, CV_32F);
//     multiply(B, -0.071f, temp1, 1, CV_32F);
//     addWeighted(Cr, 1.0, temp1, 1.0, 0.0, Cr, CV_32F);
    
//     // === 统计计算 ===
//     Scalar mb = mean(Cb), mr = mean(Cr);
//     Mat absCb, absCr;
//     absdiff(Cb, mb[0], absCb);
//     absdiff(Cr, mr[0], absCr);
//     Scalar db = mean(absCb), dr = mean(absCr);
    
//     // === 白点筛选 ===
//     Mat bv, rv, J1, J2, J;
//     absdiff(Cb, mb[0] + db[0] * (mb[0] > (128.0f/255) ? 1.0f : -1.0f), bv);
//     absdiff(Cr, 1.5f * mr[0] + dr[0] * (mr[0] > (128.0f/255) ? 1.0f : -1.0f), rv);
//     compare(bv, 1.5f*db[0], J1, CMP_LT);
//     compare(rv, 1.5f*dr[0], J2, CMP_LT);
//     bitwise_and(J1, J2, J);
    
//     // === 亮度筛选 ===
//     Mat Y_filtered;
//     Y.copyTo(Y_filtered, J);
    
//     vector<float> candidate;
//     for(int i = 0; i < Y_filtered.rows; i++) {
//         for(int j = 0; j < Y_filtered.cols; j++) {
//             float val = Y_filtered.at<float>(i,j);
//             if(val > 0) candidate.push_back(val);
//         }
//     }
    
//     sort(candidate.begin(), candidate.end(), greater<float>());
//     float min_v = candidate.empty() ? 0 : candidate[candidate.size()*0.1];
    
//     Mat Y1;
//     compare(Y, min_v, Y1, CMP_GT);
//     Y1.convertTo(Y1, CV_32F, 1.0/255);
    
//     // === 增益计算 ===
//     Scalar ravg = sum(R.mul(Y1)) / (sum(Y1)[0] + 1e-6f);
//     Scalar gavg = sum(G.mul(Y1)) / (sum(Y1)[0] + 1e-6f);
//     Scalar bavg = sum(B.mul(Y1)) / (sum(Y1)[0] + 1e-6f);
//     double ymax;
//     minMaxLoc(Y, nullptr, &ymax);
    
//     // === 应用增益 ===
//     multiply(R, ymax/(ravg[0]+1e-6f), channels[2], 1, CV_32F);
//     multiply(G, ymax/(gavg[0]+1e-6f), channels[1], 1, CV_32F);
//     multiply(B, ymax/(bavg[0]+1e-6f), channels[0], 1, CV_32F);
    
//     threshold(channels[2], channels[2], 1.0, 1.0, THRESH_TRUNC);
//     threshold(channels[1], channels[1], 1.0, 1.0, THRESH_TRUNC);
//     threshold(channels[0], channels[0], 1.0, 1.0, THRESH_TRUNC);
    
//     threshold(channels[2], channels[2], 0.0, 0.0, THRESH_TOZERO);
//     threshold(channels[1], channels[1], 0.0, 0.0, THRESH_TOZERO);
//     threshold(channels[0], channels[0], 0.0, 0.0, THRESH_TOZERO);
    
//     merge(channels, im);
// }

// int main() {
//     auto start = chrono::high_resolution_clock::now();
    
//     string input_path = "/home/ucar/ucar_car/ypicture/picture_130.jpg";
//     string output_path = "/home/ucar/ucar_car/ypicture/picture_130_awb4.jpg";
    
//     Mat I = imread(input_path, IMREAD_COLOR);
//     if(I.empty()) {
//         cout << "图片加载失败" << endl;
//         return -1;
//     }
    
//     I.convertTo(I, CV_32FC3, 1.0/255);
//     dynamic_awb(I);
//     I.convertTo(I, CV_8UC3, 255);
//     imwrite(output_path, I);
    
//     auto end = chrono::high_resolution_clock::now();
//     chrono::duration<double, milli> elapsed = end - start;
//     cout << "耗时：" << elapsed.count() << "ms" << endl;
    
//     return 0;
// }


// catkin_make --only-pkg-with-deps line_follow //vscode
// rosrun line_follow dynamic_awb //mobax

