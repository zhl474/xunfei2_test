#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <chrono>

using namespace cv;
using namespace std;
using namespace std::chrono;

Mat white_balance_1(const Mat& img) {
    vector<Mat> channels;
    split(img, channels);

    Scalar r_avg = mean(channels[2]);
    Scalar g_avg = mean(channels[1]);
    Scalar b_avg = mean(channels[0]);

    double k = (r_avg[0] + g_avg[0] + b_avg[0]) / 3.0;
    double kr = k / r_avg[0];
    double kg = k / g_avg[0];
    double kb = k / b_avg[0];

    Mat b, g, r;
    channels[0].convertTo(b, CV_32F);
    channels[1].convertTo(g, CV_32F);
    channels[2].convertTo(r, CV_32F);

    b = b * kb;
    g = g * kg;
    r = r * kr;

    vector<Mat> balanced_channels = {b, g, r};
    Mat balanced;
    merge(balanced_channels, balanced);

    balanced.convertTo(balanced, CV_8UC3);
    return balanced;
}

Mat white_balance_2(const Mat& img) {
    Mat src = img.clone();
    vector<Mat> channels;
    split(src, channels);
    Mat b = channels[0], g = channels[1], r = channels[2];

    int m = src.rows, n = src.cols;
    Mat sum = Mat::zeros(m, n, CV_32F);

    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            sum.at<float>(i, j) = b.at<uchar>(i, j) + g.at<uchar>(i, j) + r.at<uchar>(i, j);
        }
    }

    // Histogram
    const int histSize = 766;
    float range[] = {0, 766};
    const float* histRange = {range};
    Mat hist;
    calcHist(&sum, 1, 0, Mat(), hist, 1, &histSize, &histRange);

    int Y = 765;
    int num = 0;
    double ratio = 0.01;
    int key = 0;
    while (Y >= 0) {
        num += (int)hist.at<float>(Y);
        if (num > m * n * ratio) {
            key = Y;
            break;
        }
        Y--;
    }

    double sum_b = 0, sum_g = 0, sum_r = 0;
    int time = 0;
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            if (sum.at<float>(i, j) >= key) {
                sum_b += b.at<uchar>(i, j);
                sum_g += g.at<uchar>(i, j);
                sum_r += r.at<uchar>(i, j);
                time++;
            }
        }
    }

    double avg_b = sum_b / time;
    double avg_g = sum_g / time;
    double avg_r = sum_r / time;

    double maxvalue = 255.0;

    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            double b_val = b.at<uchar>(i, j) * maxvalue / avg_b;
            double g_val = g.at<uchar>(i, j) * maxvalue / avg_g;
            double r_val = r.at<uchar>(i, j) * maxvalue / avg_r;

            b.at<uchar>(i, j) = saturate_cast<uchar>(b_val);
            g.at<uchar>(i, j) = saturate_cast<uchar>(g_val);
            r.at<uchar>(i, j) = saturate_cast<uchar>(r_val);
        }
    }

    Mat result;
    merge(vector<Mat>{b, g, r}, result);
    return result;
}

Mat white_balance_3(const Mat& img) {
    Mat dst = Mat::zeros(img.size(), CV_64FC3);
    vector<Mat> channels(3);
    split(img, channels);

    for (int i = 0; i < 3; ++i)
        channels[i].convertTo(channels[i], CV_64F);

    double avg_b = mean(channels[0])[0];
    double avg_g = mean(channels[1])[0];
    double avg_r = mean(channels[2])[0];

    double K = (avg_b + avg_g + avg_r) / 3.0;
    double kb = K / avg_b;
    double kg = K / avg_g;
    double kr = K / avg_r;

    channels[0] = channels[0].mul(kb);
    channels[1] = channels[1].mul(kg);
    channels[2] = channels[2].mul(kr);

    merge(channels, dst);
    dst = dst.reshape(0, dst.rows);
    dst.convertTo(dst, CV_8UC3);
    return dst;
}

Mat white_balance_4(const Mat& img) {
    Mat bgr = img.clone();
    vector<Mat> channels;
    split(bgr, channels);
    Mat r = channels[2], g = channels[1], b = channels[0];
    int m = img.rows, n = img.cols;

    // 转换为浮点类型
    Mat r_float, b_float;
    r.convertTo(r_float, CV_32F);
    b.convertTo(b_float, CV_32F);

    Mat I_r_2 = r_float.mul(r_float);  // r^2
    Mat I_b_2 = b_float.mul(b_float);  // b^2

    double sum_I_r_2 = sum(I_r_2)[0], sum_I_r = sum(r_float)[0];
    double sum_I_b_2 = sum(I_b_2)[0], sum_I_b = sum(b_float)[0];
    double sum_I_g = sum(g)[0];

    double max_I_r_2 = 0, max_I_b_2 = 0, max_I_r = 0, max_I_b = 0, max_I_g = 0;

    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            if (r.at<uchar>(i, j) > max_I_r)
                max_I_r = r.at<uchar>(i, j);
            if (b.at<uchar>(i, j) > max_I_b)
                max_I_b = b.at<uchar>(i, j);
            if (g.at<uchar>(i, j) > max_I_g)
                max_I_g = g.at<uchar>(i, j);
        }
    }

    max_I_r_2 = max_I_r * max_I_r;
    max_I_b_2 = max_I_b * max_I_b;

    // 构造方程组求解 u_b, v_b
    Mat A = (Mat_<double>(2, 2) << sum_I_b_2, sum_I_b, max_I_b_2, max_I_b);
    Mat B = (Mat_<double>(2, 1) << sum_I_g, max_I_g);
    Mat sol_b;
    solve(A, B, sol_b);
    double u_b = sol_b.at<double>(0);
    double v_b = sol_b.at<double>(1);

    // 构造方程组求解 u_r, v_r
    A = (Mat_<double>(2, 2) << sum_I_r_2, sum_I_r, max_I_r_2, max_I_r);
    B = (Mat_<double>(2, 1) << sum_I_g, max_I_g);
    Mat sol_r;
    solve(A, B, sol_r);
    double u_r = sol_r.at<double>(0);
    double v_r = sol_r.at<double>(1);

    Mat b0 = Mat::zeros(b.size(), CV_8UC1);
    Mat r0 = Mat::zeros(r.size(), CV_8UC1);
    Mat g0 = g.clone();

    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            double b_val = u_b * pow(b.at<uchar>(i, j), 2) + v_b * b.at<uchar>(i, j);
            double r_val = u_r * pow(r.at<uchar>(i, j), 2) + v_r * r.at<uchar>(i, j);

            b0.at<uchar>(i, j) = saturate_cast<uchar>(b_val);
            r0.at<uchar>(i, j) = saturate_cast<uchar>(r_val);
        }
    }

    vector<Mat> balanced = {b0, g0, r0};
    Mat result;
    merge(balanced, result);
    return result;
}


Mat white_balance_5(const Mat& img) {
    Mat yuv;
    cvtColor(img, yuv, COLOR_BGR2YCrCb);
    vector<Mat> yuv_channels;
    split(yuv, yuv_channels);
    Mat y = yuv_channels[0], u = yuv_channels[1], v = yuv_channels[2];

    int m = img.rows, n = img.cols;
    double sum_u = 0, sum_v = 0;
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            sum_u += u.at<uchar>(i, j);
            sum_v += v.at<uchar>(i, j);
        }
    }

    double avl_u = sum_u / (m * n);
    double avl_v = sum_v / (m * n);

    double du = 0, dv = 0;
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            du += abs(u.at<uchar>(i, j) - avl_u);
            dv += abs(v.at<uchar>(i, j) - avl_v);
        }
    }

    double avl_du = du / (m * n);
    double avl_dv = dv / (m * n);

    Mat num_y = Mat::zeros(y.size(), CV_8UC1);
    double radio = 0.5;
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            double cond_u = abs(u.at<uchar>(i, j) - (avl_u + avl_du * copysign(1.0, avl_u)));
            double cond_v = abs(v.at<uchar>(i, j) - (avl_v + avl_dv * copysign(1.0, avl_v)));
            if (cond_u < radio * avl_du || cond_v < radio * avl_dv) {
                num_y.at<uchar>(i, j) = y.at<uchar>(i, j);
            }
        }
    }

    // Histogram
    const int histSize = 256;
    float range[] = {0, 256};
    const float* histRange = {range};
    Mat hist;
    calcHist(&num_y, 1, 0, Mat(), hist, 1, &histSize, &histRange);

    int Y = 255;
    int num = 0, key = 0;
    int ysum = countNonZero(num_y);
    while (Y >= 0) {
        num += (int)hist.at<float>(Y);
        if (num > 0.1 * ysum) {
            key = Y;
            break;
        }
        Y--;
    }

    vector<Mat> channels;
    split(img, channels);
    Mat b = channels[0], g = channels[1], r = channels[2];

    double sum_r = 0, sum_g = 0, sum_b = 0;
    int num_rgb = 0;
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            if (num_y.at<uchar>(i, j) > key) {
                sum_b += b.at<uchar>(i, j);
                sum_g += g.at<uchar>(i, j);
                sum_r += r.at<uchar>(i, j);
                num_rgb++;
            }
        }
    }

    double avg_b = sum_b / num_rgb;
    double avg_g = sum_g / num_rgb;
    double avg_r = sum_r / num_rgb;

    double max_y = 255.0;

    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            double b_val = b.at<uchar>(i, j) * max_y / avg_b;
            double g_val = g.at<uchar>(i, j) * max_y / avg_g;
            double r_val = r.at<uchar>(i, j) * max_y / avg_r;

            b.at<uchar>(i, j) = saturate_cast<uchar>(b_val);
            g.at<uchar>(i, j) = saturate_cast<uchar>(g_val);
            r.at<uchar>(i, j) = saturate_cast<uchar>(r_val);
        }
    }

    Mat result;
    merge(vector<Mat>{b, g, r}, result);
    return result;
}
// 主函数（ROS 兼容）
int main() {
    string input_path = "/home/ucar/ucar_car/ypicture/picture_383.jpg";
    string output_dir = "/home/ucar/ucar_car/ypicture/";

    Mat img = imread(input_path, IMREAD_COLOR);
    if (img.empty()) {
        cout << "无法加载图像: " << input_path << endl;
        return -1;
    }

    // 用于保存结果
    Mat result1, result2, result3, result4, result5;

    // 记录每个算法的耗时
    high_resolution_clock::time_point start, end;
    duration<double, milli> elapsed;

    // --- 白平衡 1 ---
    start = high_resolution_clock::now();
    result1 = white_balance_1(img);
    end = high_resolution_clock::now();
    elapsed = end - start;
    cout << "white_balance_1 耗时: " << elapsed.count() << " ms" << endl;

    // --- 白平衡 2 ---
    start = high_resolution_clock::now();
    result2 = white_balance_2(img);
    end = high_resolution_clock::now();
    elapsed = end - start;
    cout << "white_balance_2 耗时: " << elapsed.count() << " ms" << endl;

    // --- 白平衡 3 ---
    start = high_resolution_clock::now();
    result3 = white_balance_3(img);
    end = high_resolution_clock::now();
    elapsed = end - start;
    cout << "white_balance_3 耗时: " << elapsed.count() << " ms" << endl;

    // --- 白平衡 4 ---
    start = high_resolution_clock::now();
    result4 = white_balance_4(img);
    end = high_resolution_clock::now();
    elapsed = end - start;
    cout << "white_balance_4 耗时: " << elapsed.count() << " ms" << endl;

    // --- 白平衡 5 ---
    start = high_resolution_clock::now();
    result5 = white_balance_5(img);
    end = high_resolution_clock::now();
    elapsed = end - start;
    cout << "white_balance_5 耗时: " << elapsed.count() << " ms" << endl;

    // 保存结果
    imwrite(output_dir + "picture_383_wb1.jpg", result1);
    imwrite(output_dir + "picture_383_wb2.jpg", result2);
    imwrite(output_dir + "picture_383_wb3.jpg", result3);
    imwrite(output_dir + "picture_383_wb4.jpg", result4);
    imwrite(output_dir + "picture_383_wb5.jpg", result5);

    cout << "所有结果已保存至: " << output_dir << endl;

    return 0;
}