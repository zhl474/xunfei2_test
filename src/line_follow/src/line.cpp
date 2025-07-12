#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include<ros/ros.h>
#include <random>
#include <string>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <sstream>


using namespace cv;
using namespace std;

string output_file = "/home/ucar/ucar_car/src/line_follow/image/line.avi";//录制视频避免网络传输卡顿
VideoWriter out;
int fourcc = VideoWriter::fourcc('X', 'V', 'I', 'D'); // MP4V编码
ostringstream displayStream;


void drawLineFromEquation(cv::Mat& img, double a, double b, double c, const cv::Scalar& color, int thickness) {
    int width = img.cols;
    int height = img.rows;
    
    // 特殊情况处理
    if (fabs(a) < 1e-6 && fabs(b) < 1e-6) {
        return; // 无效直线
    }
    
    cv::Point pt1, pt2;
    
    if (fabs(b) > fabs(a)) {
        // 更接近水平线，使用左右边界
        pt1.x = 0;
        pt1.y = static_cast<int>(-c / b); // x=0 时的 y 值
        
        pt2.x = width - 1;
        pt2.y = static_cast<int>((-a * (width - 1) - c) / b); // x=width-1 时的 y 值
    } else {
        // 更接近垂直线，使用上下边界
        pt1.y = 0;
        pt1.x = static_cast<int>(-c / a); // y=0 时的 x 值
        
        pt2.y = height - 1;
        pt2.x = static_cast<int>((-b * (height - 1) - c) / a); // y=height-1 时的 x 值
    }
    
    // 裁剪直线到图像边界
    cv::Rect rect(0, 0, width, height);
    if (cv::clipLine(rect, pt1, pt2)) {
        cv::line(img, pt1, pt2, color, thickness);
    }
}

// RANSAC直线拟合函数
// 输入：点集 points，距离阈值 distThreshold，最大迭代次数 maxIterations
// 输出：直线参数 (a, b, c) 满足 ax+by+c=0，以及内点索引
std::pair<std::vector<double>, std::vector<int>> fitLineRANSAC(
    std::vector<cv::Point>& points, 
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
cv::Vec4f refineLine(const std::vector<cv::Point>& points, 
                    const std::vector<int>& inliers) 
{
    std::vector<cv::Point> inlierPoints;
    for (int idx : inliers) inlierPoints.push_back(points[idx]);
    
    cv::Vec4f line;
    cv::fitLine(inlierPoints, line, cv::DIST_L2, 0, 0.01, 0.01);
    return line;  // 返回格式: (vx, vy, x0, y0)
}

// 从图像底部向上搜索指定行数，分别独立寻找左右两侧的赛道边缘起始点
void find_track_edge(Mat& gray_img, Point& right_point, int scan_rows = 200, int brightness_threshold = 20) {
    int height = gray_img.rows;
    int width = gray_img.cols;
    int middle_x = width / 2;
    bool flag = 1;
    int last_scanned_y = height - scan_rows;

    for (int y = height - 1; y >= last_scanned_y; y--) {
        // 向右搜索边界
        if (right_point.x == -1) {
            for (int x = middle_x + 1; x < width - 2; x++) {
                int current = (int)gray_img.at<uchar>(y, x);
                int next = (int)gray_img.at<uchar>(y, x + 1);
                if (next>=120){ 
                    int brightness_change = next - current;    
                    if (brightness_change >= brightness_threshold) {
                        right_point = Point(x, y);
                        break;
                    }
                }
                
            }
        }
        else break;
    }
}

// 从起始点开始追踪赛道边线（添加断裂检测机制）
void trace_edge(Point start_point, Mat& gray_img, vector<Point>& traced_points, bool& broken, int search_range = 20, 
                int brightness_threshold = 20,Mat* visual_img = nullptr) {
    int height = gray_img.rows;
    int width = gray_img.cols;
    
    traced_points.clear();
    traced_points.push_back(start_point);
    broken = false;
    // int right_total = 0;//计算右线的平均坐标，用来判断正前方到底是右边线还是左边线
    // int effective_point = 1;
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
                    // right_total += cand_x2;
                    // effective_point++;
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
                        // right_total += cand_x;
                        // effective_point++;
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
            if (fail_count >= 7) {
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
bool find_left_edge(Mat gray_img,Point& left_edge_point,int brightness_threshold,Mat visualizeImg = Mat()){
    int height = gray_img.rows;
    int width = gray_img.cols;
    bool flag = false;
    // ROS_INFO("进入左边巡线");
    left_edge_point = Point(-1,-1);
    for (int y = height - 1; y >= 200; y--) {
        for (int x = width -1; x > 1; x--) {
            int brightness_change = gray_img.at<uchar>(y, x - 1) - gray_img.at<uchar>(y, x);
            if (brightness_change >= brightness_threshold && gray_img.at<uchar>(y, x - 1)>150 && gray_img.at<uchar>(y, x)>80) {
                // left_edge_points.push_back(Point(x, height-y));
                left_edge_point=Point(x, y);
                // ROS_INFO("左边界");
                flag = true;
                break;
            }
        }
        if (flag) break;
    }
    if(left_edge_point.x == -1){
        // ROS_INFO("没找到左线");
        return false;
    } 
    else {
        if (!visualizeImg.empty()) {
            circle(visualizeImg, left_edge_point, 3, Scalar(0, 0, 255), -1);
            // imshow("visualize",visualizeImg);
            // waitKey(1);
            out.write(visualizeImg);
        }
        return false;
    }
}

bool find_left_line(Mat gray_img,vector<Point>& left_edge_points,int brightness_threshold,Mat visualizeImg = Mat()){
    int height = gray_img.rows;
    int width = gray_img.cols;
    bool flag = 1;
    // ROS_INFO("进入左边巡线");
    for (int y = height - 1; y >= 200; y--) {
        for (int x = width -1; x > 1; x--) {
            int brightness_change = gray_img.at<uchar>(y, x - 1) - gray_img.at<uchar>(y, x);
            if (brightness_change >= brightness_threshold && gray_img.at<uchar>(y, x - 1)>150 && gray_img.at<uchar>(y, x)>80) {
                // left_edge_points.push_back(Point(x, height-y));
                left_edge_points.push_back(Point(x, y));
                break;
            }
        }
    }
    if(left_edge_points.empty()){
        ROS_INFO("没找到左线");
        return false;
    } 
    else {
        std::pair<std::vector<double>, std::vector<int>> result;
        result = fitLineRANSAC(left_edge_points,7,1000);
        if (!visualizeImg.empty()) {
            drawLineFromEquation(visualizeImg,result.first[0],result.first[1],result.first[2],cv::Scalar(0, 0, 255),2);
            // imshow("visualize",visualizeImg);
            // waitKey(1);
        }
        if (!visualizeImg.empty()) {
            // ROS_INFO("左赛道可视化");
            // 可视化代码（例如在图像上绘制轨迹）
            for (int i=0;i<left_edge_points.size();i++) {
                circle(visualizeImg, left_edge_points[i], 3, Scalar(255, 0, 0), -1);
            }
            // imshow("visualize",visualizeImg);
            // waitKey(1);
            out.write(visualizeImg);
        }
        ROS_INFO("直线方程abc%f,%f,%f",result.first[0],result.first[1],result.first[2]);
        ROS_INFO("直线与图像底部交点%f",(-1*result.first[2]-(480.0*result.first[1]))/result.first[0]);
        if((-1*result.first[2]-(480*result.first[1]))/result.first[0]>320){//直线和图像底部的交点
            return true;
        }else{
            return false;
        }
    }
}


double error_calculater(vector<Point>& traced_points,int ystart,Mat& visualizeImg){
    double total_error;
    size_t count = std::min(traced_points.size(),static_cast<size_t>(150));
    for (size_t i=0;i<count;i++){
        int y = ystart-i;
        double mid_error = (traced_points[i].x - (280 - (424-y)*1.34)-320);
        total_error += mid_error;
    }
    double error_normalize = total_error/count*-1;

    // 可视化代码（例如在图像上绘制轨迹）
    for (int i=0;i<count;i++) {
        int y = ystart-i;
        Point pt = Point(traced_points[i].x - (320 - (424-y)*1.34),ystart-i);
        circle(visualizeImg, pt, 3, Scalar(0, 255, 0), -1);
    }
    // imshow("visualize",visualizeImg);
    // waitKey(1);

    return error_normalize;
}

int main(int argc, char **argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "line");
    FileStorage fs("/home/ucar/ucar_car/src/line_follow/camera_info/pinhole.yaml", FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "无法打开标定文件" << endl;
        return -1;
    }
    ros::NodeHandle nh;
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

    double p,i,d,integration,pre_error;
    p = 0.006;
    i = 0.1;
    d = 0.1;
    bool right = true;//判断现在是右边线还是左边线
    int first_point_x_last = 320;//上一帧的赛道起点，发生突变就说明右赛道变成左赛道了
    bool left_forward = true;
    bool point_forward = true;

    out.open(output_file, fourcc, 15, Size(640, 480));
    while(ros::ok()){
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
        flip(undistorted, undistorted, 1);
        Mat gray_img;
        // cvtColor(undistorted, gray_img, COLOR_BGR2GRAY);
        Mat red_channel;
        vector<Mat> channels;
        split(undistorted, channels);
        gray_img = channels[2];//红色通道代替灰度图
        
        int height = gray_img.rows;
        int width = gray_img.cols;
        
        // 创建可视化图像
        Mat result_image = undistorted.clone();

        // 2. 搜索赛道边缘点
        int scan_rows = 200;  // 向上搜索的行数
        int brightness_threshold = 30;  // 亮度变化阈值
        Point right_edge_point = Point(-1, -1);//
        int last_scanned_y;
        
        find_track_edge(gray_img,right_edge_point, scan_rows, brightness_threshold);
        // ROS_INFO("上帧起点位置%d,现在起点位置%d",first_point_x_last,right_edge_point.x);
        if (right && (first_point_x_last - right_edge_point.x>90 || (right_edge_point.x < 380&&right_edge_point.y<360))){//如果右边线丢了或者右边界首个点发生剧烈左移动
            right = false;
            ROS_INFO("进入左巡线逻辑");
        }
        if(!right && (right_edge_point.x-first_point_x_last>50||right_edge_point.x>480)){//左线发生剧烈偏移说明又看到右线了
            right = true;
            ROS_INFO("进入右巡线逻辑");
        }
        vector<Point> traced_right,left_edge_points;
        Point left_edge_point;

        bool right_broken = false;
        // 追踪右侧边线

        if (right) {
            left_forward = true;
            point_forward = true;
            trace_edge(right_edge_point, gray_img, traced_right, right_broken,40, brightness_threshold, &result_image);
            double error = error_calculater(traced_right,right_edge_point.y,undistorted);//有调试图片输出
            // imshow("Track Edge Detection", result_image);
            // double error = error_calculater(traced_right,right_edge_point.y);
            // ROS_INFO("误差%f",error);
            twist.linear.x = 0.3 / exp(abs(error) / 100.0);
            integration += error*0.02;
            integration = std::max(std::min(integration,1.0),-1.0);
            double diff = (error - pre_error)/0.02;
            diff = std::max(std::min(diff,1.0),-1.0);
            twist.angular.z = std::max(std::min(error*p+integration*i+diff*d,1.0),-1.0);
            displayStream << "error: " << error << "x" << twist.linear.x <<"z"<< twist.angular.z;

            string displayText = displayStream.str();
            putText(result_image, displayText, Point(50, 50),
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
            out.write(result_image);
            // ROS_INFO("输出速度%f",twist.angular.z);
        } else {
            if(left_forward){
                if(point_forward){
                    // ROS_INFO("前进");
                    find_left_edge(gray_img, left_edge_point,brightness_threshold,result_image);
                    // find_left_edge(gray_img, left_edge_point,brightness_threshold);
                    ROS_INFO("转折点坐标x%d,y%d",left_edge_point.x,left_edge_point.y);
                    if(left_edge_point.x>320 && left_edge_point.y >370){
                        point_forward = false;
                    }
                    twist.linear.x = std::max(std::min((480-left_edge_point.y)/100.0,0.15),-0.15);
                    twist.angular.z = std::max(std::min((330-left_edge_point.x)/100.0,0.1),-0.1);
                }
                else{
                    ROS_INFO("出圆环");
                    if(find_left_line(gray_img,left_edge_points,brightness_threshold,result_image)){
                        left_forward = false;
                    }
                    // if(find_left_line(gray_img,left_edge_points,brightness_threshold)){
                    //     left_forward = false;
                    // }
                    else{
                        twist.linear.x = 0.1;
                        twist.angular.z = 0.0;
                    }
                }
            }
            else{
                twist.linear.x = 0;
                twist.angular.z = -0.3;
                out.write(result_image);
            }
        }
        first_point_x_last = right_edge_point.x;//更新起点位置

        cmd_pub.publish(twist);
        // imshow("gray",gray_img);
        waitKey(1);
        // if (waitKey(20) == 32) {
        //     imwrite("/home/ucar/ucar_car/src/line_follow/image/test.jpg", gray_img);
        // }
    }
    cap.release();
    out.release();
    return 0;
}