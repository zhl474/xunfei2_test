#include <opencv2/opencv.hpp>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <omp.h>  // OpenMP 并行支持

using namespace cv;
using namespace std;
using namespace std::chrono;

// 白平衡1：均值法
void white_balance_1(Mat& img) {
    CV_Assert(img.type() == CV_8UC3);
    vector<Mat> channels;
    split(img, channels);
    Scalar r_avg = mean(channels[2]), g_avg = mean(channels[1]), b_avg = mean(channels[0]);
    double k = (r_avg[0] + g_avg[0] + b_avg[0]) / 3.0;

    channels[2].convertTo(channels[2], CV_32F, k / r_avg[0]);
    channels[1].convertTo(channels[1], CV_32F, k / g_avg[0]);
    channels[0].convertTo(channels[0], CV_32F, k / b_avg[0]);

    min(channels[2], Scalar(255), channels[2]); max(channels[2], Scalar(0), channels[2]);
    min(channels[1], Scalar(255), channels[1]); max(channels[1], Scalar(0), channels[1]);
    min(channels[0], Scalar(255), channels[0]); max(channels[0], Scalar(0), channels[0]);

    channels[2].convertTo(channels[2], CV_8U);
    channels[1].convertTo(channels[1], CV_8U);
    channels[0].convertTo(channels[0], CV_8U);

    merge(channels, img);
}

// 白平衡2：完美反射法（已修复 calcHist 参数问题）
void white_balance_2(Mat& img) {
    CV_Assert(img.type() == CV_8UC3);
    vector<Mat> channels;
    split(img, channels);
    Mat b = channels[0], g = channels[1], r = channels[2];
    int rows = img.rows, cols = img.cols;

    Mat sum_bgr = Mat::zeros(rows, cols, CV_32S);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            sum_bgr.at<int>(i, j) =
                static_cast<int>(b.at<uchar>(i, j)) +
                static_cast<int>(g.at<uchar>(i, j)) +
                static_cast<int>(r.at<uchar>(i, j));
        }
    }

    // 转换为 CV_32F，支持 calcHist
    Mat sum_bgr_float;
    sum_bgr.convertTo(sum_bgr_float, CV_32F);

    const int histSize = 766;
    float range[] = {0, 766};
    const float* histRange = range;

    Mat hist;
    int channels_idx[] = {0};
    const float* histRange_[] = {histRange};

    // ✅ 修复后的 calcHist 调用
    calcHist(&sum_bgr_float, 1, channels_idx, Mat(), hist, 1, &histSize, histRange_, true, false);

    int total_pixels = rows * cols;
    int threshold = 765;
    double ratio = 0.01;
    int count = 0;

    while (threshold >= 0) {
        count += static_cast<int>(hist.at<float>(threshold));
        if (count > total_pixels * ratio / 100.0) break;
        --threshold;
    }

    double sum_b = 0, sum_g = 0, sum_r = 0;
    int valid_count = 0;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (sum_bgr.at<int>(i, j) >= threshold) {
                sum_b += b.at<uchar>(i, j);
                sum_g += g.at<uchar>(i, j);
                sum_r += r.at<uchar>(i, j);
                ++valid_count;
            }
        }
    }

    if (valid_count == 0) return;

    double avg_b = sum_b / valid_count;
    double avg_g = sum_g / valid_count;
    double avg_r = sum_r / valid_count;

    double max_value = 255.0;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            Vec3b& pixel = img.at<Vec3b>(i, j);
            pixel[0] = saturate_cast<uchar>(pixel[0] * max_value / avg_b);
            pixel[1] = saturate_cast<uchar>(pixel[1] * max_value / avg_g);
            pixel[2] = saturate_cast<uchar>(pixel[2] * max_value / avg_r);
        }
    }
}

// 白平衡3：灰度世界法
void white_balance_3(Mat& img) {
    CV_Assert(img.type() == CV_8UC3);
    Scalar avg = mean(img);
    double k = (avg[0] + avg[1] + avg[2]) / 3.0;

    vector<Mat> channels;
    split(img, channels);

    channels[0].convertTo(channels[0], CV_32F, k / avg[0]);
    channels[1].convertTo(channels[1], CV_32F, k / avg[1]);
    channels[2].convertTo(channels[2], CV_32F, k / avg[2]);

    min(channels[0], Scalar(255), channels[0]); max(channels[0], Scalar(0), channels[0]);
    min(channels[1], Scalar(255), channels[1]); max(channels[1], Scalar(0), channels[1]);
    min(channels[2], Scalar(255), channels[2]); max(channels[2], Scalar(0), channels[2]);

    channels[0].convertTo(channels[0], CV_8U);
    channels[1].convertTo(channels[1], CV_8U);
    channels[2].convertTo(channels[2], CV_8U);

    merge(channels, img);
}

// 白平衡4：偏色检测法
void white_balance_4(Mat& img) {
    CV_Assert(img.type() == CV_8UC3);
    vector<Mat> channels;
    split(img, channels);
    Mat b = channels[0], g = channels[1], r = channels[2];
    int rows = img.rows, cols = img.cols;

    double sum_r2 = 0, sum_g = 0, sum_b2 = 0, sum_r = 0, sum_b = 0;
    double max_r2 = 0, max_r = 0, max_b2 = 0, max_b = 0, max_g = 0;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            int ri = r.at<uchar>(i, j);
            int bi = b.at<uchar>(i, j);
            int gi = g.at<uchar>(i, j);

            sum_r2 += ri * ri; sum_r += ri; max_r2 = max(max_r2, static_cast<double>(ri * ri)); max_r = max(max_r, static_cast<double>(ri));
            sum_b2 += bi * bi; sum_b += bi; max_b2 = max(max_b2, static_cast<double>(bi * bi)); max_b = max(max_b, static_cast<double>(bi));
            sum_g += gi; max_g = max(max_g, static_cast<double>(gi));
        }
    }

    Mat A_b = (Mat_<double>(2, 2) << sum_b2, sum_b, max_b2, max_b);
    Mat B_b = (Mat_<double>(2, 1) << sum_g, max_g);
    Mat A_r = (Mat_<double>(2, 2) << sum_r2, sum_r, max_r2, max_r);
    Mat B_r = (Mat_<double>(2, 1) << sum_g, max_g);

    Mat X_b, X_r;
    solve(A_b, B_b, X_b);
    solve(A_r, B_r, X_r);

    double ub = X_b.at<double>(0), vb = X_b.at<double>(1);
    double ur = X_r.at<double>(0), vr = X_r.at<double>(1);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            int bi = b.at<uchar>(i, j);
            int ri = r.at<uchar>(i, j);
            b.at<uchar>(i, j) = saturate_cast<uchar>(ub * bi * bi + vb * bi);
            r.at<uchar>(i, j) = saturate_cast<uchar>(ur * ri * ri + vr * ri);
        }
    }

    merge(channels, img);
}

// 白平衡5：动态阈值法
void white_balance_5(Mat& img) {
    CV_Assert(img.type() == CV_8UC3);
    Mat yuv;
    cvtColor(img, yuv, COLOR_BGR2YCrCb);
    vector<Mat> yuv_channels;
    split(yuv, yuv_channels);
    Mat y = yuv_channels[0], u = yuv_channels[1], v = yuv_channels[2];

    int rows = y.rows, cols = y.cols;
    double sum_u = 0, sum_v = 0;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            sum_u += u.at<uchar>(i, j);
            sum_v += v.at<uchar>(i, j);
        }
    }

    double avg_u = sum_u / (rows * cols);
    double avg_v = sum_v / (rows * cols);

    double du = 0, dv = 0;
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            du += abs(u.at<uchar>(i, j) - avg_u);
            dv += abs(v.at<uchar>(i, j) - avg_v);
        }
    }

    double avg_du = du / (rows * cols);
    double avg_dv = dv / (rows * cols);

    vector<Mat> bgr_channels;
    split(img, bgr_channels);
    Mat b = bgr_channels[0], g = bgr_channels[1], r = bgr_channels[2];

    vector<int> yhistogram(256, 0);
    int ysum = 0;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            double u_val = u.at<uchar>(i, j);
            double v_val = v.at<uchar>(i, j);

            if (abs(u_val - (avg_u + avg_du * copysign(1, avg_u))) < 0.5 * avg_du ||
                abs(v_val - (avg_v + avg_dv * copysign(1, avg_v))) < 0.5 * avg_dv) {
                int y_val = y.at<uchar>(i, j);
                yhistogram[y_val]++;
                ysum++;
            }
        }
    }

    int key = 255, count = 0;
    double ratio = 0.1;
    for (int Y = 255; Y >= 0; --Y) {
        count += yhistogram[Y];
        if (count > ysum * ratio) {
            key = Y;
            break;
        }
    }

    double sum_r = 0, sum_g = 0, sum_b = 0;
    int num_rgb = 0;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (y.at<uchar>(i, j) > key) {
                sum_r += r.at<uchar>(i, j);
                sum_g += g.at<uchar>(i, j);
                sum_b += b.at<uchar>(i, j);
                ++num_rgb;
            }
        }
    }

    if (num_rgb == 0) return;

    double avg_r = sum_r / num_rgb;
    double avg_g = sum_g / num_rgb;
    double avg_b = sum_b / num_rgb;

    double max_y = 255.0;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            b.at<uchar>(i, j) = saturate_cast<uchar>(b.at<uchar>(i, j) * max_y / avg_b);
            g.at<uchar>(i, j) = saturate_cast<uchar>(g.at<uchar>(i, j) * max_y / avg_g);
            r.at<uchar>(i, j) = saturate_cast<uchar>(r.at<uchar>(i, j) * max_y / avg_r);
        }
    }

    merge(bgr_channels, img);
}

// 主函数
int main() {
    string input_path = "/home/ucar/ucar_car/ypicture/picture_130.jpg";
    string output_path = "/home/ucar/ucar_car/ypicture/picture_130_wb3.jpg";

    Mat I = imread(input_path, IMREAD_COLOR);
    if (I.empty()) {
        cout << "图片加载失败" << endl;
        return -1;
    }

    

    auto start = high_resolution_clock::now();

    // 选择任意白平衡方法
    white_balance_3(I);  // 可替换为 white_balance_1/3/4/5

    auto end = high_resolution_clock::now();
    duration<double, milli> elapsed = end - start;
    cout << "耗时：" << elapsed.count() << "ms" << endl;

    imwrite(output_path, I);
    cout << "结果保存至: " << output_path << endl;

    return 0;
}