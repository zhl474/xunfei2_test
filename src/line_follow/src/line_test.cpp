#include <opencv2/opencv.hpp>
#include <opencv2/xphoto.hpp>
#include <iostream>
#include <vector>
#include<ros/ros.h>
#include <geometry_msgs/Twist.h>

typedef struct {
    float Kp;           // 比例系数（响应速度）
    float Ki;           // 积分系数（消除静差）
    float Kd;           // 微分系数（抑制振荡）
    int last_error;     // 上次偏差 e(k-1)
    int prev_error;     // 上上次偏差 e(k-2)
    float integral;     // 积分累计项
    int integral_limit; // 积分限幅（防饱和）
} PID_Controller;

// 完美反射白平衡算法（基于OpenCV实现）
cv::Mat applyPerfectReflectionWB(cv::Mat src) {
    cv::Mat dst = src.clone();
    int row = src.rows;
    int col = src.cols;
    int HistRGB[767] = {0};  // 存储像素和直方图（RGB三通道最大和为765）
    int maxVal = 0;

    // 1. 计算各通道最大值及像素和直方图
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            cv::Vec3b pixel = src.at<cv::Vec3b>(i, j);
            maxVal = std::max({maxVal, static_cast<int>(pixel[0]), static_cast<int>(pixel[1]), static_cast<int>(pixel[2])});
            int sum = pixel[0] + pixel[1] + pixel[2];
            HistRGB[sum]++;
        }
    }

    // 2. 确定前10%亮度的阈值T
    int threshold = 0;
    int pixelCount = 0;
    int targetCount = static_cast<int>(row * col * 0.1);  // 取前10%最亮像素
    for (int i = 766; i >= 0; --i) {
        pixelCount += HistRGB[i];
        if (pixelCount > targetCount) {
            threshold = i;
            break;
        }
    }

    // 3. 计算亮区通道均值
    double avgB = 0, avgG = 0, avgR = 0;
    int validPixels = 0;
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            cv::Vec3b pixel = src.at<cv::Vec3b>(i, j);
            int sum = pixel[0] + pixel[1] + pixel[2];
            if (sum > threshold) {
                avgB += pixel[0];
                avgG += pixel[1];
                avgR += pixel[2];
                validPixels++;
            }
        }
    }
    avgB /= validPixels;
    avgG /= validPixels;
    avgR /= validPixels;

    // 4. 调整通道增益
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            cv::Vec3b& pixel = dst.at<cv::Vec3b>(i, j);
            pixel[0] = cv::saturate_cast<uchar>(pixel[0] * maxVal / avgB);  // 蓝通道
            pixel[1] = cv::saturate_cast<uchar>(pixel[1] * maxVal / avgG);  // 绿通道
            pixel[2] = cv::saturate_cast<uchar>(pixel[2] * maxVal / avgR);  // 红通道
        }
    }
    return dst;
}


float CalculateSteering(PID_Controller* pid, int center_error) {
    // === 增量计算 ===
    float delta_p = pid->Kp * (center_error - pid->last_error);  // P项：偏差变化量
    float delta_i = pid->Ki * center_error;                       // I项：当前偏差（离散积分近似）
    float delta_d = pid->Kd * (center_error - 2*pid->last_error + pid->prev_error); // D项：加速度

    // 合并增量
    float delta_pwm = delta_p + delta_i + delta_d;

    // === 积分抗饱和处理 ===
    pid->integral += center_error;
    // 积分限幅（避免长时间偏差导致积分项过大）
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }

    // === 更新历史误差 ===
    pid->prev_error = pid->last_error;
    pid->last_error = center_error;

    return delta_pwm;
}

void FindTrackCenter(const cv::Mat& binaryImg,
                     std::vector<int>& leftBoundary,
                     std::vector<int>& rightBoundary,
                     std::vector<int>& centerLine) {

    const int height = binaryImg.rows;
    const int width = binaryImg.cols;
    const int centerX = width / 2;  // 图像水平中心

    // 初始化输出向量
    leftBoundary.resize(height);
    rightBoundary.resize(height);
    centerLine.resize(height);
    std::vector<int> y;
    for (int i = 0; i <= 279; ++i) {
        y.push_back(i);
    }

    int rightBoundary_conut = 0;

    // 从最底层（height-1）向上扫描
    for (int row = height - 1; row >= 0; --row) {
        const uchar* pixelRow = binaryImg.ptr<uchar>(row);
        bool leftFound = false, rightFound = false;

        // --- 向左搜索左边界（白->黑跳变） ---
        for (int col = centerX; col >= 1; --col) {
            // 抗噪声：连续2个黑点确认边界[2,5](@ref)
            if (pixelRow[col] == 255 && 
                pixelRow[col - 1] == 0 && 
                (col < 2 || pixelRow[col - 2] == 0)) {
                leftBoundary[row] = col;
                leftFound = true;
                break;
            }
        }
        if (!leftFound) leftBoundary[row] = 0;  // 未找到则用图像左边界[1](@ref)

        // --- 向右搜索右边界（黑->白跳变） ---
        for (int col = centerX; col < width - 2; ++col) {
            // 抗噪声：连续2个白点确认边界
            if (pixelRow[col] == 0 && 
                pixelRow[col + 1] == 255 && 
                pixelRow[col + 2] == 255) {
                rightBoundary[row] = col;
                rightFound = true;
                break;
            }
        }
        if (!rightFound) {rightBoundary[row] = width - 1;rightBoundary_conut++;}  // 未找到则用图像右边界

        // --- 计算中线 ---
        centerLine[row] = (leftBoundary[row] + rightBoundary[row]) / 2;
    }
    std::vector<float> result;
    result.reserve(rightBoundary.size());

    // 使用迭代器遍历
    auto it_rb = rightBoundary.begin();
    auto it_y = y.begin();
    while (it_rb != rightBoundary.end() && it_y != y.end()) {
        float value = *it_rb - 320 + (*it_y * 1.2);
        result.push_back(value);
        ++it_rb;
        ++it_y;
    }
    float weighted_error = 0;
    float total_weight = 0;
    for (int row = height - 1; row >= 0; row--) {
        float weight = 0.3 - 0.01 * (height - 1 - row); // 线性递减权重
        weighted_error += (centerLine[row]-centerX) * weight;
        total_weight += weight;
    }
    double error = weighted_error / total_weight;
}

void drawGrayHistogram(cv::Mat &grayImage, cv::Mat &histogramOutput, int histSize = 256) {
    // 初始化直方图参数
    float range[] = {0, 256};
    const float* histRange = {range};
    bool uniform = true, accumulate = false;

    // 计算直方图
    cv::Mat hist;
    cv::calcHist(&grayImage, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

    // 归一化直方图（高度400像素）
    cv::normalize(hist, hist, 0, 400, cv::NORM_MINMAX);

    // 创建直方图画布
    int histWidth = 512, histHeight = 400;
    histogramOutput = cv::Mat::zeros(histHeight, histWidth, CV_8UC3);
    histogramOutput.setTo(cv::Scalar(255, 255, 255)); // 白色背景

    // 绘制直方图
    int binWidth = cvRound((double)histWidth / histSize);
    for (int i = 1; i < histSize; i++) {
        cv::line(
            histogramOutput,
            cv::Point(binWidth * (i - 1), histHeight - cvRound(hist.at<float>(i - 1))),
            cv::Point(binWidth * i, histHeight - cvRound(hist.at<float>(i))),
            cv::Scalar(0, 0, 255), 2, 8, 0  // 红色折线
        );
    }

    // 添加坐标轴
    cv::line(histogramOutput, cv::Point(0, histHeight - 1), cv::Point(histWidth, histHeight - 1), cv::Scalar(0, 0, 0), 2); // X轴
    cv::line(histogramOutput, cv::Point(0, 0), cv::Point(0, histHeight), cv::Scalar(0, 0, 0), 2); // Y轴

    // 添加刻度标签
    cv::putText(histogramOutput, "0", cv::Point(5, histHeight - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0));
    cv::putText(histogramOutput, "255", cv::Point(histWidth - 25, histHeight - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0));
}

cv::Mat twoPassLabeling(const cv::Mat& binaryImg, int minArea) {
    std::vector<int> areas;
    if (binaryImg.empty() || binaryImg.type() != CV_8UC1) {
        std::cout << "Input must be a binary image (CV_8UC1)." << std::endl;
        return cv::Mat();
    }

    int rows = binaryImg.rows;
    int cols = binaryImg.cols;
    cv::Mat labelImg = cv::Mat::zeros(rows, cols, CV_32SC1); // 标签矩阵（整数类型）
    int currentLabel = 1; // 起始标签值

    // 第一遍扫描：临时标签分配
    std::vector<int> parent(rows * cols / 2, 0); // 等价关系映射表（动态数组）
    std::vector<int> neighborLabels; // 存储邻域标签

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            if (binaryImg.at<uchar>(y, x) == 0) continue; // 跳过背景

            neighborLabels.clear();
            // 检查8邻域（左上、上、右上、左）
            if (y > 0 && x > 0 && labelImg.at<int>(y-1, x-1) > 0) 
                neighborLabels.push_back(labelImg.at<int>(y-1, x-1));
            if (y > 0 && labelImg.at<int>(y-1, x) > 0) 
                neighborLabels.push_back(labelImg.at<int>(y-1, x));
            if (y > 0 && x < cols-1 && labelImg.at<int>(y-1, x+1) > 0) 
                neighborLabels.push_back(labelImg.at<int>(y-1, x+1));
            if (x > 0 && labelImg.at<int>(y, x-1) > 0) 
                neighborLabels.push_back(labelImg.at<int>(y, x-1));

            if (neighborLabels.empty()) {
                // 无邻域标签：分配新标签
                labelImg.at<int>(y, x) = currentLabel;
                parent[currentLabel] = currentLabel; // 初始化等价关系
                currentLabel++;
            } else {
                // 取最小邻域标签，并合并等价关系
                int minLabel = *min_element(neighborLabels.begin(), neighborLabels.end());
                labelImg.at<int>(y, x) = minLabel;
                for (int label : neighborLabels) {
                    while (parent[label] != label) label = parent[label]; // 追溯根标签
                    if (label != minLabel) parent[label] = minLabel; // 合并等价关系
                }
            }
        }
    }

    // 标签合并：统一等价关系的根标签
    std::vector<int> rootMap(parent.size(), 0);
    int validLabelCount = 1;
    for (int i = 1; i < currentLabel; ++i) {
        int root = i;
        while (parent[root] != root) root = parent[root]; // 找到根标签
        if (rootMap[root] == 0) {
            rootMap[root] = validLabelCount++; // 分配新标签
        }
        rootMap[i] = rootMap[root];
    }

    // 第二遍扫描：应用最终标签 + 计算面积
    areas.resize(validLabelCount, 0);
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            int label = labelImg.at<int>(y, x);
            if (label == 0) continue;
            label = rootMap[label]; // 映射到最终标签
            labelImg.at<int>(y, x) = label;
            areas[label]++;
        }
    }

    // 生成结果：仅保留面积大于阈值的连通域
    cv::Mat result = cv::Mat::zeros(rows, cols, CV_8UC1);
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            int label = labelImg.at<int>(y, x);
            if (label > 0 && areas[label] >= minArea) {
                result.at<uchar>(y, x) = 255; // 保留前景
            }
        }
    }
    return result;
}

int main(int argc, char **argv) {
    setlocale(LC_ALL,"");
    ROS_INFO("111");
    ros::init(argc, argv, "line_test");
    ros::NodeHandle nh;
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Rate rate(20);
    cv::VideoCapture cap("/dev/video0", cv::CAP_V4L2);
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open camera!");
        return -1;
    }
    // cap.set(cv::CAP_PROP_SETTINGS, 1);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    // cap.set(cv::CAP_PROP_WB_TEMPERATURE, 0);
    // cap.set(cv::CAP_PROP_AUTO_WB, 1); 
    // cap.set(cv::CAP_PROP_WB_TEMPERATURE, 5800);

    cv::Rect roi(0, 200, 640, 280);  // (x, y, width, height)
    cv::Mat gray;
    cv::Mat src;
    cv::Mat binary;
    cv::Mat cropped;
    cv::Mat blurred;
    cv::Mat balancedFrame;

    PID_Controller steering_pid;
    steering_pid.Kp = 0.8f;
    steering_pid.Ki = 0.05f;
    steering_pid.Kd = 0.3f;
    steering_pid.integral_limit = 100; // 积分限幅值
    steering_pid.last_error = 0;
    steering_pid.prev_error = 0;
    steering_pid.integral = 0;

    while(ros::ok()) {
        cap.read(src);
        if (src.empty()) continue;
        // ros::Time ros_start = ros::Time::now();
        // balancedFrame = applyPerfectReflectionWB(src);
        // double ros_duration = (ros::Time::now() - ros_start).toSec();
        // ROS_INFO("[ROS时间] 耗时: %.6f 秒", ros_duration);
        // cv::imshow("原始画面", src);
        // cv::imshow("白平衡后", balancedFrame);
        cropped = src(roi);
        cv::cvtColor(cropped, gray, cv::COLOR_BGR2GRAY); // BGR转灰度
        cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 1.5);  // 5×5核，标准差1.5
        // double otsu_threshold = cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::adaptiveThreshold(
            gray,                          // 输入图像（单通道灰度图）
            binary,                          // 输出二值图像
            255,                          // 二值化最大值（满足条件时赋予的像素值）
            cv::ADAPTIVE_THRESH_GAUSSIAN_C, // 自适应方法：高斯加权均值
            cv::THRESH_BINARY,            // 二值化类型：大于阈值设为255，否则0
            11,                           // 邻域块大小（奇数）
            -15                             // 调整常数C
        );
        binary = twoPassLabeling(binary,70);
        // // std::cout << "Otsu自动计算的阈值: " << otsu_threshold << std::endl;
        // cv::imshow("原始图像", src);
        // // cv::imshow("灰度图", gray);
        cv::imshow("二值化结果", binary);
        // cv::Mat histImage;
        // drawGrayHistogram(gray, histImage); 
        // cv::imshow("灰度直方图", histImage);
        cv::waitKey(1);
        rate.sleep();
    }
    return 0;
}

// #include <opencv2/opencv.hpp>
// #include <opencv2/xphoto.hpp>  // 白平衡模块


// int main() {
//     // 初始化摄像头
//     cv::VideoCapture cap("/dev/video0", cv::CAP_V4L2);
//     if (!cap.isOpened()) {
//         std::cerr << "摄像头打开失败！检查设备连接或索引号" << std::endl;
//         return -1;
//     }

//     // 设置分辨率（可选）
//     cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
//     cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

//     // 创建显示窗口
//     cv::namedWindow("原始画面", cv::WINDOW_AUTOSIZE);
//     cv::namedWindow("白平衡后", cv::WINDOW_AUTOSIZE);

//     while (true) {
//         cv::Mat frame;
//         cap >> frame;  // 捕获一帧
//         if (frame.empty()) break;

//         // 应用白平衡
//         cv::Mat balancedFrame = applyPerfectReflectionWB(frame);

//         // 显示结果
//         cv::imshow("原始画面", frame);
//         cv::imshow("白平衡后", balancedFrame);

//         // 按ESC退出
//         if (cv::waitKey(300) == 27) break;
//     }

//     // 释放资源
//     cap.release();
//     cv::destroyAllWindows();
//     return 0;
// }