#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <chrono>
#include <iostream>

using namespace cv;
using namespace std;

void dynamic_awb(Mat& im) {
    CV_Assert(im.type() == CV_32FC3);
    
    // 分离通道 (BGR顺序)
    vector<Mat> channels(3);
    split(im, channels);
    Mat R = channels[2].clone();  // 显式克隆确保独立内存
    Mat G = channels[1].clone();
    Mat B = channels[0].clone();
    
    // 预分配输出矩阵并显式指定类型
    Mat Y(im.size(), CV_32F);
    Mat Cb(im.size(), CV_32F);
    Mat Cr(im.size(), CV_32F);
    
    // 分步计算YCbCr - 显式类型转换
    Mat tmp;
    R.convertTo(tmp, CV_32F);
    multiply(tmp, 0.257f, Y, 1, CV_32F);
    
    G.convertTo(tmp, CV_32F);
    multiply(tmp, 0.504f, tmp, 1, CV_32F);
    add(Y, tmp, Y, noArray(), CV_32F);
    
    B.convertTo(tmp, CV_32F);
    multiply(tmp, 0.098f, tmp, 1, CV_32F);
    add(Y, tmp, Y, noArray(), CV_32F);
    add(Y, Scalar(16.0f/255.0f), Y, noArray(), CV_32F);
    
    // 计算Cb分量
    R.convertTo(tmp, CV_32F);
    multiply(tmp, -0.148f, Cb, 1, CV_32F);
    
    G.convertTo(tmp, CV_32F);
    multiply(tmp, -0.291f, tmp, 1, CV_32F);
    add(Cb, tmp, Cb, noArray(), CV_32F);
    
    B.convertTo(tmp, CV_32F);
    multiply(tmp, 0.439f, tmp, 1, CV_32F);
    add(Cb, tmp, Cb, noArray(), CV_32F);
    add(Cb, Scalar(128.0f/255.0f), Cb, noArray(), CV_32F);
    
    // 计算Cr分量
    R.convertTo(tmp, CV_32F);
    multiply(tmp, -0.439f, Cr, 1, CV_32F);
    
    G.convertTo(tmp, CV_32F);
    multiply(tmp, -0.368f, tmp, 1, CV_32F);
    add(Cr, tmp, Cr, noArray(), CV_32F);
    
    B.convertTo(tmp, CV_32F);
    multiply(tmp, -0.071f, tmp, 1, CV_32F);
    add(Cr, tmp, Cr, noArray(), CV_32F);
    add(Cr, Scalar(128.0f/255.0f), Cr, noArray(), CV_32F);
    
    // 全局统计
    Scalar Mb = mean(Cb), Mr = mean(Cr);
    Mat absCb, absCr;
    absdiff(Cb, Mb[0], absCb);
    absdiff(Cr, Mr[0], absCr);
    Scalar Db = mean(absCb), Dr = mean(absCr);
    
    // 白点筛选 - 显式类型处理
    Mat sign_b = (Cb > (128.0f/255.0f)) / 255.0f;
    sign_b.convertTo(sign_b, CV_32F);
    Mat sign_r = (Cr > (128.0f/255.0f)) / 255.0f;
    sign_r.convertTo(sign_r, CV_32F);
    
    Mat bv, rv;
    subtract(Cb, Mb[0] + Db[0] * (sign_b - 0.5f)*2.0f, bv, noArray(), CV_32F);
    abs(bv, bv);
    subtract(Cr, 1.5f * Mr[0] + Dr[0] * (sign_r - 0.5f)*2.0f, rv, noArray(), CV_32F);
    abs(rv, rv);
    
    Mat J1, J2;
    compare(bv, 1.5f * Db[0], J1, CMP_LT);
    compare(rv, 1.5f * Dr[0], J2, CMP_LT);
    bitwise_and(J1, J2, J1);
    
    // 亮度筛选
    Mat Y_filtered;
    Y.copyTo(Y_filtered, J1);
    
    // 使用OpenCV的sortIdx进行排序
    Mat sortedIndices;
    sortIdx(Y_filtered.reshape(1,1), sortedIndices, SORT_EVERY_ROW + SORT_DESCENDING);
    
    int idx = static_cast<int>(sortedIndices.total() * 0.1);
    float min_v = idx > 0 ? Y_filtered.at<float>(sortedIndices.at<int>(idx)) : 0;
    
    // 增益计算
    Mat Y1;
    compare(Y, min_v, Y1, CMP_GT);
    Y1.convertTo(Y1, CV_32F, 1.0f/255.0f);
    
    Scalar sum_Y1 = sum(Y1);
    Scalar Ravg = sum(R.mul(Y1)) / (sum_Y1[0] + 1e-6f);
    Scalar Gavg = sum(G.mul(Y1)) / (sum_Y1[0] + 1e-6f);
    Scalar Bavg = sum(B.mul(Y1)) / (sum_Y1[0] + 1e-6f);
    
    double Ymax;
    minMaxLoc(Y, nullptr, &Ymax);
    
    // 应用增益 - 显式类型转换
    multiply(R, Ymax/(Ravg[0]+1e-6f), channels[2], 1, CV_32F);
    multiply(G, Ymax/(Gavg[0]+1e-6f), channels[1], 1, CV_32F);
    multiply(B, Ymax/(Bavg[0]+1e-6f), channels[0], 1, CV_32F);
    
    // 限制范围
    threshold(channels[2], channels[2], 1.0f, 1.0f, THRESH_TRUNC);
    threshold(channels[2], channels[2], 0.0f, 0.0f, THRESH_TOZERO);
    threshold(channels[1], channels[1], 1.0f, 1.0f, THRESH_TRUNC);
    threshold(channels[1], channels[1], 0.0f, 0.0f, THRESH_TOZERO);
    threshold(channels[0], channels[0], 1.0f, 1.0f, THRESH_TRUNC);
    threshold(channels[0], channels[0], 0.0f, 0.0f, THRESH_TOZERO);
    
    merge(channels, im);
}

int main() {
    try {
        string input_path = "/home/ucar/ucar_car/ypicture/picture_130.jpg";
        string output_path = "/home/ucar/ucar_car/ypicture/picture_130_awb_cpp.jpg";
        
        Mat img = imread(input_path, IMREAD_COLOR);
        if(img.empty()) {
            cerr << "Error loading image!" << endl;
            return -1;
        }
        
        // 转换为浮点并处理
        Mat fimg;
        img.convertTo(fimg, CV_32FC3, 1.0f/255.0f);
        
        auto start = chrono::high_resolution_clock::now();
        dynamic_awb(fimg);
        auto end = chrono::high_resolution_clock::now();
        
        // 保存结果
        Mat output;
        fimg.convertTo(output, CV_8UC3, 255.0f);
        imwrite(output_path, output);
        
        chrono::duration<double, milli> elapsed = end - start;
        cout << "Processing time: " << elapsed.count() << "ms" << endl;
        
    } catch (const cv::Exception& e) {
        cerr << "OpenCV error: " << e.what() << endl;
        return -1;
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
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

