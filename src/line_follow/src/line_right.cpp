#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include<ros/ros.h>
#include <random>
#include <string>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <sstream>
#include "line_follow/line_follow.h"
#include "ztestnav2025/getpose_server.h"
#include "ztestnav2025/lidar_process.h"
#include "ztestnav2025/set_speed.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

using namespace cv;
using namespace std;

string output_file = "/home/ucar/ucar_car/src/line_follow/image/line_right.avi";//录制视频避免网络传输卡顿
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

int brightness_threshold_calculator(Mat& gray_img){//寻找跳变最剧烈的那个点，这个点的左值就是图像二值化阈值
    int max_brightness_change = 0;
    int best_binary_brightness = 180;//给个默认值，别一会没找到
    for (int y = 269; y > 69; y--) {
        for (int x = 1; x < 638; x++) {
            int current = (int)gray_img.at<uchar>(y, x);
            int next = (int)gray_img.at<uchar>(y, x + 1);
            if (next>=150&&current>70){   
                if (next - current >= max_brightness_change) {
                    max_brightness_change = next - current;
                    best_binary_brightness = next-20;
                }
            }
        }
    }
    return best_binary_brightness;
}

bool stop_car(Mat& gray,int brightness_threshold,int& point){
    int white_count = 0;
    for (int y = 227; y >= 200; y--) {//
        for (int x = 1; x < 639; x++) {
            if (gray.at<uchar>(y, x)>=brightness_threshold){ 
                white_count++;
            }
        }
    }
    point = white_count;
    if (white_count>6058){
        return true;
    }
    return false;
}

// 从图像底部向上搜索指定行数，分别独立寻找左右两侧的赛道边缘起始点
void find_track_edge(Mat& gray_img, Point& right_point, int scan_rows, int brightness_threshold) {
    int height = gray_img.rows;
    int width = gray_img.cols;
    int middle_x = width / 2;
    bool flag = 1;
    int last_scanned_y = height - scan_rows;

    for (int y = height - 1; y >= last_scanned_y; y--) {//
        // 向右搜索边界
        if (right_point.x == -1) {
            for (int x = middle_x + 1; x < width - 2; x++) {
                int current = (int)gray_img.at<uchar>(y, x);
                int next = (int)gray_img.at<uchar>(y, x + 1);
                if (next>=brightness_threshold){ 
                    right_point = Point(x, y);
                    break;
                }
            }
        }
        else break;
    }
}

// 从起始点开始追踪赛道边线（添加断裂检测机制）
void trace_edge(Point start_point, Mat& gray_img, vector<Point>& traced_points, bool& right,  
                int brightness_threshold,Mat* visual_img = nullptr) {
    int height = gray_img.rows;
    int width = gray_img.cols;
    int search_range = 60;
    traced_points.clear();
    traced_points.push_back(start_point);
    bool broken = false;
    // 计数器：记录连续未找到点的行数
    int fail_count = 0;
    
    // 初始化搜索中心
    int center_x = start_point.x;
    int center_y = start_point.y - 1;  // 从起始点上方开始搜索
    int number = 1;//没必要找太多，60个点够了

    while (center_y > start_point.y-100) {  // 只看往上130行
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

            //根据阈值
            int brightness_change;
            //右减左
            if (left_check){
                int current = gray_img.at<uchar>(center_y, cand_x2);
                int prev = gray_img.at<uchar>(center_y, cand_x2 - 1);
                brightness_change = current - prev;
                if (current >= brightness_threshold) {
                    if (brightness_change > max_brightness_change) {
                        max_brightness_change = brightness_change;
                        best_point = Point(cand_x2, center_y);
                        found = true;
                    }
                }
                else{
                    if (right_check) {
                        current = gray_img.at<uchar>(center_y, cand_x);
                        int next = gray_img.at<uchar>(center_y, cand_x + 1);
                        brightness_change = next-current;
                    } 
                    if (current >= brightness_threshold) {
                        if (brightness_change > max_brightness_change) {
                            max_brightness_change = brightness_change;
                            best_point = Point(cand_x, center_y);
                            found = true;
                        }
                    }
                }
            }
        }

        if (found) {
            traced_points.push_back(best_point);
            // 重置失败计数器
            fail_count = 0;
            number++;
            // 更新搜索中心（继续向上移动）
            center_x = best_point.x;
            center_y = best_point.y - 1;

        } else {
            // 没有找到符合条件的点
            fail_count++;
            // 向上移动一行继续搜索
            center_y--;
            // 如果连续40行找不到点，判定为赛道断裂
            if (fail_count >= 20) {
                broken = true;
                break;
            }
        }
        // 如果已经到达图像顶部，结束追踪
        if (number>60|| center_y <= 0) {
            break;
        }
    }
    
    Vec4f lineParams; // 存放结果的 Vec4f
    fitLine(traced_points, lineParams, DIST_L2, 0, 0.01, 0.01);
    // ROS_INFO("有效点数%zu",traced_points.size());
    if((lineParams[1]/lineParams[0]<-0.1&&lineParams[1]/lineParams[0]>-10)||traced_points.size()<15){//不接受右线向右倾斜数量太少不要
        right = false;
    }
    // 可视化追踪过程
    if (visual_img != nullptr) {
        Scalar color = Scalar(0, 255, 0);  // 红色:左, 绿色:右
        for (const auto& point : traced_points) {
            circle(*visual_img, point, 2, color, -1);
        }
        circle(*visual_img, start_point, 2, Scalar(0, 0, 0), -1);
        ostringstream displayStream1;
        displayStream1 << fixed << setprecision(2);
        displayStream1 << "line_slope:  " << lineParams[1]/lineParams[0];
        string displayText1 = displayStream1.str();
        putText(*visual_img, displayText1, Point(50, 100),
        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
        // imshow("test",*visual_img);
        // waitKey(1);
    }
}
//如果右边丢线，就只看左边，因为右边丢线了，所以直接从最右边开始找，找到就是左线，然后把左线拟合成直线，如果左线碰到图片底端，就开始旋转，通过定位来判断是否到达终点，到达终点前不启用停车逻辑
bool find_left_edge(Mat gray_img,Point& left_edge_point,int brightness_threshold,Mat& visualizeImg){
    int height = gray_img.rows;
    int width = gray_img.cols;
    bool flag = false;
    left_edge_point = Point(-1,-1);
    Mat blurred = gray_img;
    // GaussianBlur(gray_img, blurred, cv::Size(5,5), 0);//这个点很重要，务必不能找错
    for (int y = height - 1; y >= 69; y--) {
        for (int x = width -1; x > 150; x--) {
            if (blurred.at<uchar>(y, x) >= brightness_threshold) {
                left_edge_point=Point(x, y);
                flag = true;
                break;
            }
        }
        if (flag) break;
    }
    if(left_edge_point.x == -1){
        ROS_INFO("没找到左点");
        return false;
    } 
    else {
        if (!visualizeImg.empty()) {
            circle(visualizeImg, left_edge_point, 7, Scalar(0, 0, 255), -1);
        }
        return false;
    }
}

bool find_left_line(Mat gray_img,vector<Point>& left_edge_points,int brightness_threshold,Mat visualizeImg = Mat()){
    int height = gray_img.rows;
    int width = gray_img.cols;
    bool flag = 1;
    // ROS_INFO("进入左边巡线");
    for (int y = height - 1; y >= 69; y--) {
        for (int x = width -1; x > 1; x--) {
            if (gray_img.at<uchar>(y, x) >= brightness_threshold) {
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
        drawLineFromEquation(visualizeImg,result.first[0],result.first[1],result.first[2],cv::Scalar(0, 0, 255),2);
        for (int i=0;i<left_edge_points.size();i++) {
            circle(visualizeImg, left_edge_points[i], 3, Scalar(255, 0, 0), -1);
        }
        out.write(visualizeImg);
        if((-1*result.first[2]-(270*result.first[1]))/result.first[0]>320){//直线和图像底部的交点
            return true;
        }else{
            return false;
        }
    }
}

double double_find(Mat gray_img,int brightness_threshold)//最后阶段采用双边巡线，避免单边巡线导致的偏离从而无法停车
{
    vector<int> left_total;
    vector<int> right_total;
    double error = 0.0;

    bool falg = false;//赛道先有后无，就是没线了
    int failed = 0;//连续几行都没找到才算丢线
    for (int y = 269; y >= 50; y--) {//计算每一行的误差
        int left = 0;
        bool find = false;
        for (int x = 319; x > 1; x--) {
            if (gray_img.at<uchar>(y, x) >= brightness_threshold) {
                left = x;
                find = true;
                falg = true;
                failed = 0;
                break;
            }
        }
        if (falg && !find){
            failed++;
            if (failed>10){
                break;
            }
        }
        left_total.push_back(left);
    }
    falg = false;//赛道先有后无，就是没线了
    failed = 0;//连续几行都没找到才算丢线
    for (int y = 269; y >= 50; y--) {//计算每一行的误差
        int right = 639;
        bool find = false;
        for (int x = 319; x < 639; x++) {
            if ((int)gray_img.at<uchar>(y, x) >= brightness_threshold) {
                right = x;
                find = true;
                falg = true;
                failed = 0;
                break;
            }
        }
        if (falg && !find){
            failed++;
            if (failed>10){
                break;
            }
        }
        right_total.push_back(right);
    }
    float row = min(left_total.size(),right_total.size())/1.0;
    for(int i=0;i<row;i++){
        error += (640-(left_total[i]+right_total[i]))*(1-i/row);//error += (left_total+right_total-640)/2.0*(1-i/row) error = error/(row/2)归一化的除以2整理到前面
    }
    return error/row;
}


double error_calculater(vector<Point>& traced_points,int ystart,Mat& visualizeImg){
    double total_error = 0;
    // size_t count = std::min(traced_points.size(),static_cast<size_t>(60));//注意了这里number最大就到80记得把前面对number的限制也改了
    for (size_t i=0;i<traced_points.size();i++){
        int y = ystart-i;
        if (i <= 30.0) {
            double mid_error = (traced_points[i].x - (280 - (214-y)*1.34)-320)*(1-i/100);
            total_error += mid_error;
        }
        else {
            double mid_error = (traced_points[i].x - (280 - (214-y)*1.34)-320)*0.7 * exp(-0.064 * (i - 30.0));
            total_error += mid_error;
        }
    }

    // 可视化代码（例如在图像上绘制轨迹）
    for (int i=0;i<traced_points.size();i++) {
        int y = ystart-i;
        Point pt = Point(traced_points[i].x - (320 - (214-y)*1.34),ystart-i);
        circle(visualizeImg, pt, 3, Scalar(0, 255, 0), -1);
    }
    // imshow("visualize",visualizeImg);
    // waitKey(1);
    if (traced_points.size()==0){
        return 100.0;
    }
    else{
        return total_error/traced_points.size()*-1;
    }
}

bool line_server_callback(line_follow::line_follow::Request& req,line_follow::line_follow::Response& resp){
    FileStorage fs("/home/ucar/ucar_car/src/line_follow/camera_info/pinhole.yaml", FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "无法打开标定文件" << endl;
        return -1;
    }
    ros::NodeHandle nh;
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    geometry_msgs::Twist twist;
    ROS_INFO("等待lidar_process服务中---");
    ros::ServiceClient client_line_board = nh.serviceClient<ztestnav2025::lidar_process>("/lidar_process/lidar_process");
    ztestnav2025::lidar_process board;
    board.request.lidar_process_start = -2;
    client_line_board.waitForExistence();
    ROS_INFO("等待坐标获取服务中---");
    ros::ServiceClient pose_client = nh.serviceClient<ztestnav2025::getpose_server>("getpose_server");
    ztestnav2025::getpose_server pose;
    pose.request.getpose_start = 1;
    pose_client.waitForExistence();
    ROS_INFO("等待movebase服务中---");
    ros::ServiceClient client_movebase = nh.serviceClient<ztestnav2025::set_speed>("set_speed");
    ztestnav2025::set_speed target_info;
    target_info.request.movebase_flag = true;
    client_movebase.waitForExistence();
    ROS_INFO("tf变换");
    tf::TransformListener* tf_listener_;
    tf_listener_ = new tf::TransformListener();


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
    Rect roi(0, 210, 640, 270);

    double p,i,d,integration,pre_error,leftpoint_p_,leftpoint_I_;
    nh.getParam("/line_right/right_P", p);
    nh.getParam("/line_right/right_I", i);
    nh.getParam("/line_right/right_D", d);
    nh.getParam("/line_right/leftpoint_p", leftpoint_p_);
    nh.getParam("/line_right/leftpoint_I", leftpoint_I_);
    ROS_INFO("参数加载P: %f", p);
    integration = 0;
    pre_error = 0;
    double pointx_integration = 0;
    double pointx_pre_error = 0;
    double pointy_integration = 0;
    double pointy_pre_error = 0;
    bool right = true;//判断现在是右边线还是左边线
    int first_point_x_last = 320;//上一帧的赛道起点，发生突变就说明右赛道变成左赛道了
    bool left_forward = true;
    bool point_forward = true;

    out.open(output_file, fourcc, 5, Size(640, 270));
    displayStream << fixed << setprecision(2);

    ros::Time start_time = ros::Time::now();
    ros::Time frame_start = ros::Time::now();
    bool double_line = false;//最后进入两边巡线逻辑
    int point_confirm = 0;//要连续看到那个点20帧才行，不然会超调
            
    int height = 270;
    int width = 640;
    int scan_rows = 180;  // 向上搜索的行数
    while(ros::ok()){
        // ROS_INFO("耗时:%f",(ros::Time::now()-frame_start).toSec());
        // frame_start = ros::Time::now();
        //----------------------------------避障逻辑----------------------------//
        client_line_board.call(board);
        pose_client.call(pose);
        if(board.response.lidar_results[0] != -1){
            ROS_INFO("最短距离%f",board.response.lidar_results[0]);

            float vx = board.response.lidar_results[4];//存储xy分量
            float vy = board.response.lidar_results[5];

            double d = std::sqrt(1 + board.response.lidar_results[3]*board.response.lidar_results[3]);
            geometry_msgs::PointStamped lidar_point;
            lidar_point.header.frame_id = "laser_frame";
            lidar_point.point.x = board.response.lidar_results[1] + 0.15*board.response.lidar_results[3]/d;
            lidar_point.point.y = board.response.lidar_results[2] - 0.15/d;
            // lidar_point.point.z = std::atan(1/board.response.lidar_results[3]);
            lidar_point.point.z = std::atan2(vy, vx);//使用atan2不会有角度180度跳变
            ROS_INFO("板子在雷达坐标系下的斜率%f",lidar_point.point.z);
            geometry_msgs::PointStamped point_base;
            tf_listener_->transformPoint("map", lidar_point, point_base);
            ROS_INFO("坐标变换结果: (%.2f, %.2f, %.2f)",
                    point_base.point.x, 
                    point_base.point.y, 
                    point_base.point.z);
            
            target_info.request.target_x = point_base.point.x;
            target_info.request.target_y = point_base.point.y;
            target_info.request.target_yaw = point_base.point.z;
            client_movebase.call(target_info);
        }


        //----------------------------------巡线逻辑----------------------------//
        displayStream.str("");
        cap.read(image);
        if (image.empty()) continue;
        remap(image, undistorted, map1, map2, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
        Mat cropped = undistorted(roi);
        flip(cropped, cropped, 1);
        Mat gray_img;
        vector<Mat> channels;
        split(cropped, channels);
        gray_img = channels[2];//红色通道代替灰度图
        int brightness_threshold = brightness_threshold_calculator(gray_img);  // 亮度变化阈值就是跳变最剧烈点的左值
        // ROS_INFO("%d",brightness_threshold);
        Point right_edge_point = Point(-1, -1);//
        int last_scanned_y;
        find_track_edge(gray_img,right_edge_point, scan_rows, brightness_threshold);
        if (right && (first_point_x_last - right_edge_point.x>150) &&pose.response.pose_at[0]>3.0){//如果右边线丢了或者右边界首个点发生剧烈左移动
            right = false;
            ROS_INFO("左跳变%d,%d",first_point_x_last,right_edge_point.x);
        }
        else if(!right && (right_edge_point.x-first_point_x_last>250||right_edge_point.x>500 )){//左线发生剧烈偏移说明又看到右线了左跳右的幅度一般很剧烈|| (right_edge_point.y>170&&right_edge_point.x>300)
            right = true;
            ROS_INFO("右跳变:%d,%d",right_edge_point.x,first_point_x_last);
        }
        vector<Point> traced_right,left_edge_points;
        Point left_edge_point;
        // 追踪右侧边线
        bool right_checker = true;//右线不一定真的是右线，可能是太偏的左线，不接受右线向右倾斜，不满足条件切换逻辑
        if (right) {
            first_point_x_last = right_edge_point.x;
            double line_error = 0;
            if(!double_line){
                trace_edge(right_edge_point, gray_img, traced_right, right_checker, brightness_threshold, &cropped);
                line_error = error_calculater(traced_right,right_edge_point.y,cropped);//有调试图片输出
            }
            else{
                line_error = double_find(gray_img,brightness_threshold);
            }

            twist.linear.x = 0.5 / exp(abs(line_error) / 100.0);
            integration += line_error*0.03;
            integration = std::max(std::min(integration,1.0),-1.0);
            double diff = line_error - pre_error;
            diff = std::max(std::min(diff,50.0),-50.0);
            twist.angular.z = std::max(std::min(line_error*p+integration*i+diff*d,1.0),-1.0);
            pre_error = line_error;
            displayStream << "error: " << line_error << "p: " << line_error*p << "i: " << integration*i << "d: " << diff*d<<"z: "<< twist.angular.z;
            string displayText = displayStream.str();
            putText(cropped, displayText, Point(50, 50),
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
            out.write(cropped);
            // ROS_INFO("积分项%f",integration);

            if(!right_checker){
                ROS_INFO("右线斜率出错，舍弃");
                twist.angular.z = std::max(twist.angular.z-0.05,-0.3);
            }
            else{
                if (!left_forward && !point_forward && (ros::Time::now()-start_time).toSec()>1.0){
                    double_line = true;
                    nh.getParam("/line_right/double_P", p);
                    nh.getParam("/line_right/double_I", i);
                    nh.getParam("/line_right/double_D", d);
                    ROS_INFO("p%f",p);
                    ROS_INFO("双边巡线");
                }
                point_confirm = 0;
                left_forward = true;
                point_forward = true;
            }
        } else {
            if(left_forward){
                if(point_forward){
                    // ROS_INFO("左点");
                    start_time = ros::Time::now();
                    find_left_edge(gray_img, left_edge_point,brightness_threshold,cropped);
                    first_point_x_last = left_edge_point.x;
                    // find_left_edge(gray_img, left_edge_point,brightness_threshold);
                    double error_x = 320-left_edge_point.x;//double error_y = 160-left_edge_point.y;
                    pointx_integration += error_x*0.02;//pointy_integration += error_y*0.02;
                    // ROS_INFO("转折点坐标x%d,y%d",left_edge_point.x,left_edge_point.y);
                    // if(abs(left_edge_point.x-320) < 20 && abs(left_edge_point.y -160)<20){
                    if(abs(left_edge_point.x-320) < 20){
                        point_confirm++;
                        if(point_confirm>7){
                            pointx_integration = 0;
                            // pointy_integration = 0;
                            point_forward = false;
                        }
                    }
                    pointx_integration = std::max(std::min(pointx_integration,1.0),-1.0);//pointy_integration = std::max(std::min(pointy_integration,1.0),-1.0);
                    // ROS_INFO("P%f,I%f",error_y/400,pointy_integration/400);
                    twist.linear.x = std::max(twist.linear.x-0.15,0.0);
                    twist.angular.z = std::max(std::min(error_x*leftpoint_p_ + pointx_integration * leftpoint_I_,0.5),-0.5);
                    pointx_pre_error = error_x;//pointy_pre_error = error_y;
                    displayStream <<"z:  "<< twist.angular.z<<"errorx:  "<<error_x<<"pointx_integration:"<<pointx_integration;
                    string displayText = displayStream.str();
                    putText(cropped, displayText, Point(50, 50),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
                    out.write(cropped);
                }
                else{
                    // ROS_INFO("左线");
                    if(find_left_line(gray_img,left_edge_points,brightness_threshold,cropped)){
                        left_forward = false;
                    }
                    else{
                        twist.linear.x = 0.1;
                        twist.angular.z = -0.05;
                    }
                }
            }
            else{
                twist.linear.x = 0;
                twist.angular.z = -0.8;
                out.write(cropped);
            }
        }
        int test;
        if(stop_car(gray_img,brightness_threshold,test)){
            ROS_INFO("巡线结束");
            twist.linear.x = 0;
            twist.angular.z = 0;
            cmd_pub.publish(twist);
            break;
        }
        cmd_pub.publish(twist);
    }
    cap.release();
    out.release();
    return true;
}
int main(int argc, char **argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "line_right");
    ros::NodeHandle nh_;
    ros::ServiceServer line_server = nh_.advertiseService("line_server", line_server_callback);
    ROS_INFO("视觉巡线初始化");
    ros::spin();
    return 0;
}