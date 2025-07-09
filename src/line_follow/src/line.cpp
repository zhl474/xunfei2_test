#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include<ros/ros.h>
#include <string>

using namespace cv;
using namespace std;

// 从图像底部向上搜索指定行数，分别独立寻找左右两侧的赛道边缘起始点
void find_track_edge(Mat& gray_img, vector<Point>& left_points, vector<Point>& right_points, 
                     int& last_scanned_y, int scan_rows = 200, int brightness_threshold = 20) {
    int height = gray_img.rows;
    int width = gray_img.cols;
    int middle_x = width / 2;
    last_scanned_y = max(0, height - scan_rows);

    left_points.clear();
    right_points.clear();

    cout << "扫描底部" << scan_rows << "行寻找赛道起始点..." << endl;

    // 从底部向上逐行扫描
    for (int y = height - 1; y >= last_scanned_y; y--) {
        // 向左搜索边界
        if (left_points.empty()) {
            for (int x = middle_x - 1; x > 0; x--) {
                int current = gray_img.at<uchar>(y, x);
                int next = gray_img.at<uchar>(y, x + 1);
                int brightness_change = current - next;
                
                if (brightness_change >= brightness_threshold) {
                    left_points.push_back(Point(x, y));
                    cout << "找到左侧边界起始点 (x=" << x << ", y=" << y 
                         << "), 亮度变化: " << brightness_change << endl;
                    break;
                }
            }
        }

        // 向右搜索边界
        if (right_points.empty()) {
            for (int x = middle_x + 1; x < width - 1; x++) {
                int current = gray_img.at<uchar>(y, x);
                int prev = gray_img.at<uchar>(y, x - 1);
                int brightness_change = current - prev;
                
                if (brightness_change >= brightness_threshold) {
                    right_points.push_back(Point(x, y));
                    cout << "找到右侧边界起始点 (x=" << x << ", y=" << y 
                         << "), 亮度变化: " << brightness_change << endl;
                    break;
                }
            }
        }
    }
}

// 从起始点开始追踪赛道边线（添加断裂检测机制）
void trace_edge(Point start_point, Mat& gray_img, vector<Point>& traced_points, bool& broken,
                bool is_left_edge = true, int search_range = 20, 
                int brightness_threshold = 20, Mat* visual_img = nullptr) {
    int height = gray_img.rows;
    int width = gray_img.cols;
    
    traced_points.clear();
    traced_points.push_back(start_point);
    broken = false;
    
    // 计数器：记录连续未找到点的行数
    int fail_count = 0;
    
    // 初始化搜索中心
    int center_x = start_point.x;
    int center_y = start_point.y - 1;  // 从起始点上方开始搜索
    
    string color_name = is_left_edge ? "左侧" : "右侧";
    cout << "开始追踪" << color_name << "边界 (起始点: (" 
         << start_point.x << "," << start_point.y << "))..." << endl;

    while (center_y > 0) {  // 向上追踪直到图像顶部
        bool found = false;
        Point best_point;
        int max_brightness_change = 0;

        // 在当前行搜索范围内检查所有可能点
        for (int dx = -search_range; dx <= search_range; dx++) {
            // 计算候选点位置
            int cand_x = center_x + dx;
            int cand_y = center_y;

            // 边界检查
            if (cand_x < 1 || cand_x >= width - 1 || cand_y < 0 || cand_y >= height) {
                continue;
            }

            // 跳过已追踪点
            bool already_traced = false;
            for (const auto& pt : traced_points) {
                if (pt.x == cand_x && pt.y == cand_y) {
                    already_traced = true;
                    break;
                }
            }
            if (already_traced) continue;

            // 计算亮度变化（根据边界类型确定方向）
            int brightness_change;
            if (is_left_edge) {
                // 左侧边缘：检测右边暗到左边亮的变化
                int current = gray_img.at<uchar>(cand_y, cand_x);
                int next = gray_img.at<uchar>(cand_y, cand_x + 1);
                brightness_change = current - next;
            } else {
                // 右侧边缘：检测左边暗到右边亮的变化
                int current = gray_img.at<uchar>(cand_y, cand_x);
                int prev = gray_img.at<uchar>(cand_y, cand_x - 1);
                brightness_change = current - prev;
            }

            // 检查是否满足阈值
            if (brightness_change >= brightness_threshold) {
                // 选择变化最大的点
                if (brightness_change > max_brightness_change) {
                    max_brightness_change = brightness_change;
                    best_point = Point(cand_x, cand_y);
                    found = true;
                }
            }
        }

        if (found) {
            // 添加到追踪点列表
            traced_points.push_back(best_point);
            // 重置失败计数器
            fail_count = 0;
            // 更新搜索中心（继续向上移动）
            center_x = best_point.x;
            center_y = best_point.y - 1;

            // 调试输出
            if (traced_points.size() % 20 == 0) {
                cout << color_name << "边界追踪进度: y=" << center_y 
                     << ", 已找到" << traced_points.size() << "个点" << endl;
            }
        } else {
            // 没有找到符合条件的点
            fail_count++;
            // 向上移动一行继续搜索
            center_y--;

            // 调试输出
            cout << color_name << "边界在y=" << center_y + 1 
                 << "未找到边界点 (失败次数: " << fail_count << "/2)" << endl;

            // 如果连续2行找不到点，判定为赛道断裂
            if (fail_count >= 2) {
                cout << "!! " << color_name << "边界发生断裂 !! (连续" 
                     << fail_count << "行未找到边界点)" << endl;
                broken = true;
                break;
            }
        }

        // 如果已经到达图像顶部，结束追踪
        if (center_y <= 0) {
            break;
        }
    }

    cout << color_name << "边界追踪完成: 共找到" << traced_points.size() << "个点" << endl;

    // 可视化追踪过程
    if (visual_img != nullptr) {
        Scalar color = is_left_edge ? Scalar(0, 0, 255) : Scalar(0, 255, 0);  // 红色:左, 绿色:右

        // 绘制所有追踪点
        for (const auto& point : traced_points) {
            circle(*visual_img, point, 2, color, -1);
        }

        // 绘制起始点（更大更显眼）
        circle(*visual_img, start_point, 5, color, -1);

        // 绘制断裂点（如果有）
        if (broken) {
            Point break_point(center_x, center_y + 1);
            circle(*visual_img, break_point, 8, Scalar(255, 255, 0), -1);  // 青色标记断裂点
            putText(*visual_img, "Break Point", Point(break_point.x - 50, break_point.y - 10),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
        }
    }
}

int main(int argc, char **argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "line");
    // 1. 读取图像并转换为灰度图
    cv::VideoCapture cap("/dev/video0", cv::CAP_V4L2);
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open camera!");
        return -1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    Mat image;

    while(1){
        cap.read(image);
        if (image.empty()) continue;

        Mat gray_img;
        cvtColor(image, gray_img, COLOR_BGR2GRAY);
        int height = gray_img.rows;
        int width = gray_img.cols;
        cout << "图像尺寸: " << width << "x" << height << endl;

        // 创建可视化图像
        Mat result_image = image.clone();

        // 2. 搜索赛道边缘点
        int scan_rows = 200;  // 向上搜索的行数
        int brightness_threshold = 20;  // 亮度变化阈值
        vector<Point> left_edge_points, right_edge_points;
        int last_scanned_y;
        
        find_track_edge(gray_img, left_edge_points, right_edge_points, last_scanned_y, 
                        scan_rows, brightness_threshold);

        // 3. 追踪赛道边线
        vector<Point> traced_left, traced_right;
        bool left_broken = false, right_broken = false;

        // 追踪左侧边线
        if (!left_edge_points.empty()) {
            Point start_point = left_edge_points[0];
            trace_edge(start_point, gray_img, traced_left, left_broken, true,
                    20, brightness_threshold, &result_image);
            cout << "左侧边界追踪结果: 点数=" << traced_left.size() 
                << ", 是否断裂=" << (left_broken ? "是" : "否") << endl;
        } else {
            cout << "未找到左侧起始点，跳过左侧追踪" << endl;
        }

        // 追踪右侧边线
        if (!right_edge_points.empty()) {
            Point start_point = right_edge_points[0];
            trace_edge(start_point, gray_img, traced_right, right_broken, false,
                    20, brightness_threshold, &result_image);
            cout << "右侧边界追踪结果: 点数=" << traced_right.size() 
                << ", 是否断裂=" << (right_broken ? "是" : "否") << endl;
        } else {
            cout << "未找到右侧起始点，跳过右侧追踪" << endl;
        }

        // 4. 根据追踪结果判断赛道类型
        cout << "\n=== 赛道分析报告 ===" << endl;
        cout << "左侧边界点数: " << traced_left.size() 
            << ", 断裂状态: " << (left_broken ? "是" : "否") << endl;
        cout << "右侧边界点数: " << traced_right.size() 
            << ", 断裂状态: " << (right_broken ? "是" : "否") << endl;

        // 简单的赛道类型判断
        string track_type;
        if (traced_left.empty() && traced_right.empty()) {
            track_type = "未识别到赛道";
            cout << "赛道类型: " << track_type << endl;
        } else if (left_broken && right_broken) {
            track_type = "完全断裂（可能是急转弯或障碍区）";
            cout << "赛道类型: " << track_type << endl;
        } else if (!left_broken && !right_broken) {
            track_type = "完整赛道";
            cout << "赛道类型: " << track_type << endl;
        } else if (left_broken && !right_broken) {
            track_type = "左侧断裂（可能是左急转弯）";
            cout << "赛道类型: " << track_type << endl;
        } else if (!left_broken && right_broken) {
            track_type = "右侧断裂（可能是右急转弯）";
            cout << "赛道类型: " << track_type << endl;
        } else {
            track_type = "复杂情况，需要进一步分析";
            cout << "赛道类型: " << track_type << endl;
        }

        // 5. 绘制扫描区域
        rectangle(result_image, Point(0, last_scanned_y), Point(width - 1, height - 1), 
                Scalar(255, 0, 255), 1);
        putText(result_image, "Scan Area: " + to_string(height - last_scanned_y) + " rows",
                Point(10, last_scanned_y - 5), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 255), 1);

        // 6. 添加文本说明
        if (!traced_left.empty()) {
            string status = left_broken ? "断裂" : "连续";
            putText(result_image, "Left: " + to_string(traced_left.size()) + "点 " + status,
                    Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
        }
        
        if (!traced_right.empty()) {
            string status = right_broken ? "断裂" : "连续";
            putText(result_image, "Right: " + to_string(traced_right.size()) + "点 " + status,
                    Point(width - 200, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
        }
        
        // 在图像底部显示赛道类型
        putText(result_image, "Track Type: " + track_type, Point(width / 2 - 150, height - 20),
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
        
        putText(result_image, "Threshold: " + to_string(brightness_threshold), Point(10, height - 20),
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 1);

        // 7. 显示结果
        imshow("Gray Image", gray_img);
        imshow("Track Edge Detection", result_image);

        // 8. 保存调试图像（可选）
        imwrite("/home/zhl/track_edge_result.jpg", gray_img);

        // 9. 等待按键后退出
        waitKey(1);
    }

    return 0;
}