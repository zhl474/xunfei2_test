#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include<ros/ros.h>
#include <random>
#include <string>


using namespace cv;
using namespace std;

// RANSAC直线拟合函数
// 输入：点集 points，距离阈值 distThreshold，最大迭代次数 maxIterations
// 输出：直线参数 (a, b, c) 满足 ax+by+c=0，以及内点索引
std::pair<std::vector<double>, std::vector<int>> fitLineRANSAC(
    std::vector<cv::Point2f>& points, 
    float distThreshold, 
    int maxIterations) 
{
    // 随机数生成器初始化
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_int_distribution<int> uni(0, points.size()-1);

    // 最佳模型和内点
    std::vector<double> bestModel(3);
    std::vector<int> bestInliers;
    int bestInlierCount = 0;

    // RANSAC主循环
    for (int i = 0; i < maxIterations; ++i) {
        // 1. 随机选两个点
        int idx1 = uni(rng);
        int idx2 = uni(rng);
        while (idx2 == idx1) idx2 = uni(rng);  // 确保不同点
        
        const auto& p1 = points[idx1];
        const auto& p2 = points[idx2];
        
        // 2. 计算直线参数 (a, b, c)
        double a = p2.y - p1.y;
        double b = p1.x - p2.x;
        double c = p2.x * p1.y - p1.x * p2.y;
        
        // 处理重合点 (分母接近0)
        double denom = std::sqrt(a*a + b*b);
        if (denom < 1e-5) continue;  // 跳过无效直线
        
        // 3. 计算内点
        std::vector<int> inliers;
        for (int j = 0; j < points.size(); ++j) {
            const auto& pt = points[j];
            double dist = std::abs(a*pt.x + b*pt.y + c) / denom;
            if (dist < distThreshold) inliers.push_back(j);
        }
        
        // 4. 更新最佳模型
        if (inliers.size() > bestInlierCount) {
            bestInlierCount = inliers.size();
            bestModel = {a, b, c};
            bestInliers = std::move(inliers);
        }
    }
    
    return {bestModel, bestInliers};
}

// 使用内点精确拟合直线
cv::Vec4f refineLine(const std::vector<cv::Point2f>& points, 
                    const std::vector<int>& inliers) 
{
    std::vector<cv::Point2f> inlierPoints;
    for (int idx : inliers) inlierPoints.push_back(points[idx]);
    
    cv::Vec4f line;
    cv::fitLine(inlierPoints, line, cv::DIST_L2, 0, 0.01, 0.01);
    return line;  // 返回格式: (vx, vy, x0, y0)
}

// 从图像底部向上搜索指定行数，分别独立寻找左右两侧的赛道边缘起始点
void find_track_edge(Mat& gray_img, Point& right_point, 
                     int& last_scanned_y, int scan_rows = 200, int brightness_threshold = 20) {
    int height = gray_img.rows;
    int width = gray_img.cols;
    int middle_x = width / 2;
    bool flag = 1;
    last_scanned_y = height - scan_rows;
    for (int y = height - 1; y >= last_scanned_y; y--) {
        // 向右搜索边界
        if (right_point.x = -1) {
            for (int x = middle_x + 1; x < width - 1; x++) {
                int current = gray_img.at<uchar>(y, x);
                int prev = gray_img.at<uchar>(y, x - 1);
                int brightness_change = current - prev;
                
                if (brightness_change >= brightness_threshold) {
                    right_point = Point(x, y);
                    break;
                }
            }
        }
        else break;
    }
}

// 从起始点开始追踪赛道边线（添加断裂检测机制）
void trace_edge(Point start_point, Mat& gray_img, vector<Point>& traced_points, bool& broken, int search_range = 20, 
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

    while (center_y > 0) {  // 向上追踪直到图像顶部
        bool found = false;
        Point best_point;
        int max_brightness_change = 0;

        // 在当前行搜索范围内检查所有可能点
        for (int dx = 0; dx <= search_range/2; dx++) {
            // 计算候选点位置
            int cand_x = center_x + dx;
            int cand_x2 = center_x - dx;
            bool left_check = 1;
            bool right_check = 1;
            // 边界检查
            if (cand_x >= width - 1) {
                right_check = 0;
            }
            if (cand_x2 < 1) {
                left_check = 0;
            }

            // 计算亮度变化（根据边界类型确定方向）
            int brightness_change;
            //右减左
            if (left_check){
                int current = gray_img.at<uchar>(center_y, cand_x2);
                int prev = gray_img.at<uchar>(center_y, cand_x2 - 1);
                brightness_change = current - prev;
            }
            if (brightness_change >= brightness_threshold) {
                if (brightness_change > max_brightness_change) {
                    max_brightness_change = brightness_change;
                    best_point = Point(cand_x2, center_y);
                    found = true;
                }
            }
            else{
                if (right_check) {
                    int current = gray_img.at<uchar>(center_y, cand_x);
                    int next = gray_img.at<uchar>(center_y, cand_x + 1);
                    brightness_change = next-current;
                } 
                if (brightness_change >= brightness_threshold) {
                    if (brightness_change > max_brightness_change) {
                        max_brightness_change = brightness_change;
                        best_point = Point(cand_x, center_y);
                        found = true;
                    }
                }
            }
        }

        if (found) {
            traced_points.push_back(best_point);
            // 重置失败计数器
            fail_count = 0;
            // 更新搜索中心（继续向上移动）
            center_x = best_point.x;
            center_y = best_point.y - 1;

        } else {
            // 没有找到符合条件的点
            fail_count++;
            // 向上移动一行继续搜索
            center_y--;
            // 如果连续2行找不到点，判定为赛道断裂
            if (fail_count >= 2) {
                broken = true;
                break;
            }
        }

        // 如果已经到达图像顶部，结束追踪
        if (center_y <= 0) {
            break;
        }
    }

    // 可视化追踪过程
    if (visual_img != nullptr) {
        Scalar color = Scalar(0, 255, 0);  // 红色:左, 绿色:右
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
//如果右边丢线，就只看左边，因为右边丢线了，所以直接从最右边开始找，找到就是左线，然后把左线拟合成直线，如果左线碰到图片底端，就开始旋转，通过定位来判断是否到达终点，到达终点前不启用停车逻辑
bool find_left_edge(Mat gray_img,vector<Point>& left_edge_points,int brightness_threshold){
    int height = gray_img.rows;
    int width = gray_img.cols;
    bool flag = 1;
    for (int y = height - 1; y >= 200; y--) {
        for (int x = width -1; x > 1; x--) {
            int brightness_change = gray_img.at<uchar>(y, x - 1) - gray_img.at<uchar>(y, x);
            if (brightness_change >= brightness_threshold) {
                left_edge_points.push_back(Point(x, height-y));
                break;
            }
        }
    }
    if(left_edge_points.empty()) return false;
    else {
        cv::Vec4d lineModel;
        cv::fitLine(left_edge_points, lineModel, DIST_L2, 0, 0.01, 0.01); // 额外的计算开销
        if(lineModel[2]-(lineModel[3]*lineModel[0]/lineModel[1])>0){
            return true;
        }//如果与x轴交点小于0，那么还可以直线前进，大于0，则原地转弯
        else {
            return false;
        }
    }
}


double speed_calculater(vector<Point>& traced_points,int ystart,Mat visualizeImg = Mat()){
    double total_error;
    for (int i=0;i<traced_points.size();i++){
        int y = ystart-i;
        double mid_error = ((traced_points[i].x - 320 + y * 1.2)-320)*y/480.0;
        total_error += mid_error;
    }
    if (!visualizeImg.empty()) {
        // 可视化代码（例如在图像上绘制轨迹）
        for (int i=0;i<traced_points.size();i++) {
            int y = ystart-i;
            Point pt = Point(traced_points[i].x - 320 + y * 1.2,ystart-i);
            circle(visualizeImg, pt, 3, Scalar(0, 255, 0), -1);
        }
        imshow("visualize",visualizeImg);
        waitKey(1);
    }
    return total_error*0.01;
}

int main(int argc, char **argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "line");
    FileStorage fs("/home/ucar/ucar_car/src/line_follow/camera_info/pinhole.yaml", FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "无法打开标定文件" << endl;
        return -1;
    }
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    geometry_msgs::Twist twist;
    Mat cameraMatrix, distCoeffs;
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();
    // 1. 读取图像并转换为灰度图
    cv::VideoCapture cap("/dev/video0", cv::CAP_V4L2);
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open camera!");
        return -1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    Mat map1, map2;
    Mat optimalMatrix = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, Size(640, 480), 1,Size(640, 480));
    initUndistortRectifyMap(
        cameraMatrix, 
        distCoeffs, 
        Mat(), // 无旋转
        optimalMatrix, 
        Size(640, 480), 
        CV_32FC1, // 32位浮点类型（速度优化）
        map1, 
        map2
    );
    Mat image,undistorted;

    while(1){
        cap.read(image);
        if (image.empty()) continue;
        remap(
            image, 
            undistorted, 
            map1, 
            map2, 
            INTER_LINEAR, 
            BORDER_CONSTANT, 
            Scalar(0, 0, 0) // 黑边填充
        );
        Mat gray_img;
        // cvtColor(undistorted, gray_img, COLOR_BGR2GRAY);
        Mat red_channel;
        vector<Mat> channels;
        split(undistorted, channels);
        gray_img = channels[2];//红色通道代替灰度图
        flip(gray_img, gray_img, 1);
        int height = gray_img.rows;
        int width = gray_img.cols;
        
        // 创建可视化图像
        Mat result_image = undistorted.clone();

        // 2. 搜索赛道边缘点
        int scan_rows = 200;  // 向上搜索的行数
        int brightness_threshold = 20;  // 亮度变化阈值
        Point right_edge_point = Point(-1, -1);
        int last_scanned_y;
        
        find_track_edge(gray_img,right_edge_point, last_scanned_y, scan_rows, brightness_threshold);

        vector<Point> traced_right,left_edge_points;
        bool right_broken = false;
        // 追踪右侧边线
        if (right_edge_point.x != -1) {
            trace_edge(right_edge_point, gray_img, traced_right, right_broken,20, brightness_threshold, &result_image);
            double speed = speed_calculater(traced_right,right_edge_point.y,undistorted);
            twist.linear.x = 0.3;
            twist.angular.z = speed;
        } else {
            find_left_edge(gray_img, left_edge_points,brightness_threshold);
            std::pair<std::vector<double>, std::vector<int>> result;
            result = fitLineRANSAC(left_edge_points,7,1000);
            if(result[0][0]*result[0][2]<0){
                twist.linear.x = 0;
                twist.angular.z = 0.3;
            }
            else {
                twist.linear.x = 0.3;
                twist.angular.z = 0;
            }
        }

        cmd_pub.publish(twist);
        imshow("gray",gray_img);
        imshow("Track Edge Detection", result_image);

        waitKey(1);
    }

    return 0;
}