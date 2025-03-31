#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int8.h>
#include "std_msgs/Int32.h"  // 添加头文件
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include "actionlib/client/simple_goal_state.h"
#include "actionlib/client/simple_client_goal_state.h"
#include "actionlib/client/terminal_state.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/OccupancyGrid.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <cstdlib>
#include <math.h>
#include "amcl/AMCLConfig.h"
#include <vector>  
#include <cmath>  
#include <limits>  
#include <map>  
#include <locale.h>
#include <boost/shared_ptr.hpp> 
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h" 
#include <numeric>

using namespace cv;
using namespace std;


// PID控制参数
const double Kp = 0.00445; //0.00595
const double Kd = 0.0025; //0.0009
double previous_error = 0.0;
int boundary_flag = 0;
int stop_flag = 0;
int stop_flaglast = 0;
int startflag = 0;
ros::Publisher cmd_pub;
// 图像分辨率 640*480

int awake_flag = 0;
int terrorist_count = -1; // 全局变量，用于存储恐怖分子的数量
std_msgs::Int8 start_detect;//恐怖分子识别启动，为1时启动
std_msgs::Int8 start_detect1;//恐怖分子识别启动，为1时启动
std_msgs::Int8 start_detect2;//恐怖分子识别启动，为1时启动
std::string sound_file;  //恐怖分子音频文件的路径
std::string sound_file1;  //急救
bool terrorist_data_received = false; // 全局标志，表示是否已接收到恐怖分子数据
int laser_flag = 0; //开启雷达扫描标志位 
int cplus_flag = 0;
int laser_process = 0; 
int laser_flag_g = 0; 
int aid_kit_flag = 0; //急救包识别完标志位 
int xun_flag = 0; //开启巡线标志位 
int stop_vedio_flag = 0;
std::set<int> used_edges; 

move_base_msgs::MoveBaseGoal lasergoal;

struct BoardEdge {  
    float angle;  
    float distance;  
    bool is_left; // 标记为左边缘或右边缘  
};

std::map<int, std::pair<BoardEdge, BoardEdge>> boards; // 板子ID，右边缘和左边缘  
int next_board_id = 1;

boost::shared_ptr<sensor_msgs::LaserScan> g_laser_scan;  
std::vector<std::pair<int, bool>> edges; // 用于存储边缘的索引


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Pose {  
    geometry_msgs::Point position;  
    geometry_msgs::Quaternion orientation;  
};  
Pose current_pose;//位姿结构体

Pose largest_board_pose;//获取雷达序号最大板子的实时位置与位姿

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)  
{  
    
    ROS_INFO("已经进入odom回调函数");
    current_pose.position.x = msg->pose.pose.position.x;  
    current_pose.position.y = msg->pose.pose.position.y;  
    current_pose.position.z = msg->pose.pose.position.z;
    current_pose.orientation.x = msg->pose.pose.orientation.x;
    current_pose.orientation.y = msg->pose.pose.orientation.y;  
    current_pose.orientation.z = msg->pose.pose.orientation.z;  
    current_pose.orientation.w = msg->pose.pose.orientation.w;     
    
}  
  

// 获取小车的弧度
float quaternionToYawRadian(float qx, float qy, float qz, float qw) {  
    float yaw_radian = atan2(2.0f * qz * qw, qw * qw - qz * qz);       
    return yaw_radian; // 返回偏航角（弧度）  
}

// 判断是否为毛刺  
bool isSpike(int index, const std::vector<float>& ranges, int num_ranges) {    
    int count = 0;    
    for (int i = -10; i <= 10; ++i) {    
        int check_index = (index + i + num_ranges) % num_ranges;  
        // 检查是否为inf或0，如果是，则跳过  
        if (std::isinf(ranges[check_index]) || ranges[check_index] == 0) {  
            continue;  
        }  
        float diff = std::abs(ranges[check_index] - ranges[index]);    
        if (diff > 0.3) {    
            count++;    
        }    
    }    
    return count <= 1; // 如果只有一个点跳变大于0.3，则认为是毛刺    
}
// 激光雷达数据回调函数  
void laserCallback(const boost::shared_ptr<sensor_msgs::LaserScan>& scan) {
    ROS_INFO("进入雷达回调函数！");
    if(laser_process==1){
    ROS_INFO("雷达回调函数开始");
    g_laser_scan = scan;  
    int num_ranges = g_laser_scan->ranges.size();
    //重置板子数据
    boards.clear();
    next_board_id = 1;

    std::vector<int> potential_right_edges; // 潜在的右边缘索引列表  
    std::map<int, float> right_edge_to_next_distance; // 用于跟踪右边缘后一个点的距离  
    
    used_edges.clear();  
    edges.clear(); // 清空边缘列表  
  
    // 处理毛刺点  
    for (int i = 0; i < num_ranges; ++i) {  
        if (!std::isinf(g_laser_scan->ranges[i]) && g_laser_scan->ranges[i] != 0) {  
            float current_diff = std::abs(g_laser_scan->ranges[i] - g_laser_scan->ranges[(i + num_ranges - 1) % num_ranges]);  
            float next_diff = std::abs(g_laser_scan->ranges[i] - g_laser_scan->ranges[(i + 1) % num_ranges]);  
            if ((current_diff > 0.3 || next_diff > 0.3) && isSpike(i, g_laser_scan->ranges, num_ranges)) {  
                // 向前搜索有效值  
                float replacement_dist = g_laser_scan->ranges[i];  
                for (int j = 1; j < num_ranges; ++j) {  
                    int index = (i - j + num_ranges) % num_ranges;  
                    if (!std::isinf(g_laser_scan->ranges[index]) && g_laser_scan->ranges[index] != 0) {  
                        replacement_dist = g_laser_scan->ranges[index];  
                        break;  
                    }  
                }  
  
                // 若未找到，则向后搜索有效值  
                for (int j = 1; j < num_ranges; ++j) {  
                    int index = (i + j) % num_ranges;  
                    if (!std::isinf(g_laser_scan->ranges[index]) && g_laser_scan->ranges[index] != 0) {  
                        replacement_dist = g_laser_scan->ranges[index];  
                        break;  
                    }  
                }  
  
                // 用找到的有效值替换当前值  
                g_laser_scan->ranges[i] = replacement_dist;  
            }  
        }  
    }  
  
    // 处理无穷大和零距离值  
    for (int i = 0; i < num_ranges; ++i) {  
        if (std::isinf(g_laser_scan->ranges[i]) || g_laser_scan->ranges[i] == 0) {  
            float replacement_dist = 0.0;  
            bool found_replacement = false;  
  
            // 向前搜索有效值  
            for (int j = 1; j < num_ranges; ++j) {  
                int index = (i - j + num_ranges) % num_ranges;  
                if (!std::isinf(g_laser_scan->ranges[index]) && g_laser_scan->ranges[index] != 0) {  
                    replacement_dist = g_laser_scan->ranges[index];  
                    found_replacement = true;  
                    break;  
                }  
            }  
  
            // 若未找到，则向后搜索有效值  
            if (!found_replacement) {  
                for (int j = 0; j < num_ranges; ++j) {  
                    int index = (i + j) % num_ranges;  
                    if (!std::isinf(g_laser_scan->ranges[index]) && g_laser_scan->ranges[index] != 0) {  
                        replacement_dist = g_laser_scan->ranges[index];  
                        found_replacement = true;  
                        break;  
                    }  
                }  
            }  
  
            // 用找到的有效值替换当前值  
            if (found_replacement) {  
                g_laser_scan->ranges[i] = replacement_dist;  
            }  
        }  
    }  
  
     // 检测边缘  
   for (int i = 140; i < 197; ++i) {  //110.35度到249.64度
    // 检查是否为右边缘（当前点距离小于前一个点距离，且差值大于0.2）  
    if (g_laser_scan->ranges[i] < g_laser_scan->ranges[i - 1] &&  
        (g_laser_scan->ranges[i - 1] - g_laser_scan->ranges[i]) > 0.02 && (g_laser_scan->ranges[i - 1] - g_laser_scan->ranges[i]) <1) {  
        edges.emplace_back(i, false); // 右边缘  
    }  
    // 检查是否为左边缘（当前点距离小于后一个点距离，且差值大于0.2）  
    if (i < num_ranges - 1 && // 确保不是最后一个点  
        g_laser_scan->ranges[i] < g_laser_scan->ranges[i + 1] &&  
        (g_laser_scan->ranges[i + 1] - g_laser_scan->ranges[i]) > 0.02 && (g_laser_scan->ranges[i + 1] - g_laser_scan->ranges[i]) < 1) {  
        edges.emplace_back(i, true); // 左边缘  
    }  
}
  
    // 输出检测到的边缘信息  
    for (const auto& edge : edges) {  
        float angle = edge.first * 360.0f / num_ranges;  
        float distance = g_laser_scan->ranges[edge.first];  
    }  
  
 
    // 匹配左右边缘以构成板子  
std::vector<std::pair<int, BoardEdge>> board_edges; // 存储边缘及其类型，用于配对  
for (const auto& edge : edges) {  
    BoardEdge board_edge;  
    board_edge.angle = edge.first * 360.0f / num_ranges;  
    board_edge.distance = g_laser_scan->ranges[edge.first];  
    board_edge.is_left = edge.second;  
    board_edges.push_back({edge.first, board_edge});  
}  
  
// 遍历边缘列表，尝试找到相邻的左右边缘对  
for (size_t i = 0; i < board_edges.size() - 1; ++i) {  
    if (!board_edges[i].second.is_left && // 当前是右边缘  
        board_edges[i + 1].second.is_left && // 下一个是左边缘  
        std::abs(board_edges[i].second.distance - board_edges[i + 1].second.distance) <= 0.5) {  
        // 匹配成功，记录板子信息  
        int board_id = next_board_id++;  
        boards[board_id] = {board_edges[i].second, board_edges[i + 1].second};  
  
        
    }  
}    
    //检测初始坐标
    float car_x = current_pose.position.x; //  
    float car_y = current_pose.position.y; //   
    float qx = 0;    // 小车朝向的四元数x分量（可能始终为0）  
    float qy = 0;    // 同上，y分量可能也始终为0  
    float qz = current_pose.orientation.z;    // 小车朝向的四元数z分量  
    float qw = current_pose.orientation.w;
    //计算小车角度
    float yaw_radian= quaternionToYawRadian(qx,qy,qz,qw);
    //获取序号最大的板子
    if (!boards.empty()) {  
    auto max_board = *std::max_element(boards.begin(), boards.end(),  
        [](const auto& a, const auto& b) { return a.first < b.first; });  
  
    // 提取最大序号的板子的边缘信息  
    float b = max_board.second.first.distance;  // 右边缘距离  
    float c = max_board.second.second.distance; // 左边缘距离  
    float a = 0.5;                              // 板子宽度（假设）  
  
    // 计算中心点距离  
    float right_side = 2 * (b * b + c * c) - a * a;  
    float center_distance = std::sqrt(right_side / 4);  
  
    // 用余弦定理计算与右边缘的夹角（注意：这里0.0625可能是板子一角到中心的距离的平方，需要确认）  
    float angle_rag = acos((pow(center_distance, 2) + pow(b, 2) - 0.0625) / (2 * center_distance * b));  
  
    // 计算中心线角度（雷达坐标系统中的绝对角度，假设board.second.first.angle是度数）  
    float center_angle_rad = max_board.second.first.angle * (M_PI / 180.0) + angle_rag;    
  
    // 计算板子相对于全局坐标系的角度  
    float global_board_angle_radian = yaw_radian + center_angle_rad - M_PI;  
  
    // 转换角度为度数以便输出（如果需要的话）  
    float center_angle_deg = center_angle_rad * (180 / M_PI);  
  
    // 计算板子中心点的全局坐标（假设car_x和car_y是小车当前的全局x和y坐标）  
    float car_x = current_pose.position.x;  
    float car_y = current_pose.position.y;  
    float x1 = car_x + (center_distance - 0.5) * cos(global_board_angle_radian);  
    float y1 = car_y + (center_distance - 0.5) * sin(global_board_angle_radian);  
  
    // 设置序号最大的板子的位姿  
    largest_board_pose.position.x = x1;  
    largest_board_pose.position.y = y1;  
    largest_board_pose.position.z = 0.0; // 假设板子在地面上  
  
    // 注意：下面设置的四元数只是表示了板子的朝向（绕Z轴），而不是板子中心点的完整姿态  
    // 如果需要完整的姿态信息  
    largest_board_pose.orientation.x = 0.0;  
    largest_board_pose.orientation.y = 0.0;  
    largest_board_pose.orientation.z = sinf(global_board_angle_radian / 2.0f);  
    largest_board_pose.orientation.w = cosf(global_board_angle_radian / 2.0f);  
  
 
    //可以省略
    ROS_INFO("板子 #%d 中心线角度: %.2f°, 全局位置: (%.2f, %.2f), 朝向四元数: (%.4f, %.4f, %.4f, %.4f)",  
             max_board.first, center_angle_deg, x1, y1,  
             largest_board_pose.orientation.x, largest_board_pose.orientation.y,  
             largest_board_pose.orientation.z, largest_board_pose.orientation.w);  
}
    lasergoal.target_pose.pose.position.x=largest_board_pose.position.x;
    lasergoal.target_pose.pose.position.y=largest_board_pose.position.y;
    lasergoal.target_pose.pose.position.z=largest_board_pose.position.z;
    lasergoal.target_pose.pose.orientation.x=largest_board_pose.orientation.x;
    lasergoal.target_pose.pose.orientation.y=largest_board_pose.orientation.y;
    lasergoal.target_pose.pose.orientation.z=largest_board_pose.orientation.z;
    lasergoal.target_pose.pose.orientation.w=largest_board_pose.orientation.w;
    ROS_INFO("laser_process = 0");
    laser_process = 0;
    }

}  


void mic_awake_cb(const std_msgs::Int8::ConstPtr& a_p)
{
    awake_flag = a_p->data;             //传递a_p值
    ROS_INFO("awake flag is set");
}

void doMsg(const std_msgs::Int8::ConstPtr& msg_p){
    int terrorist_count = msg_p->data;   
    if (terrorist_count != -1) {   
        switch (terrorist_count) {  
            case 1: sound_file =   "~/ucar_car/src/simple_navigation_goals/voice/c1.wav";
                    sound_file1=   "~/ucar_car/src/simple_navigation_goals/voice/j1.wav";
                    terrorist_data_received = true;
                    break;  
            case 2: sound_file = "~/ucar_car/src/simple_navigation_goals/voice/c2.wav";
                    sound_file1=   "~/ucar_car/src/simple_navigation_goals/voice/j2.wav";
                    terrorist_data_received = true;
                    break;  
            case 3: sound_file = "~/ucar_car/src/simple_navigation_goals/voice/c3.wav";
                    sound_file1=   "~/ucar_car/src/simple_navigation_goals/voice/j3.wav";
                    terrorist_data_received = true; 
                    break;  
            default: ROS_WARN("Unknown terrorist count: %d", terrorist_count); break;  
        }
          
        // 注意：这里不再调用 system，因为播放将在主循环中控制  
        // system(("aplay " + sound_file).c_str());  
    }  
}//处理恐怖分子识别时获取的数据

void laser_sub_cb(const std_msgs::Int8::ConstPtr& msg_p){
    laser_flag = msg_p->data;
}

void aid_kit_sub_cb(const std_msgs::Int8::ConstPtr& msg_p){
    ROS_INFO("aid_kit_flag");
    aid_kit_flag = msg_p->data;
}

void playAudio(const std::string& sound_file) {  
    system(("aplay " + sound_file).c_str());    
}  //处理音频

void mid(Mat &follow, const Mat &mask, int &error) {
    int halfWidth = follow.cols / 2;
    int half = halfWidth;
    int mid_output = half;

    for (int y = follow.rows - 1; y >= 0; --y) {
        int left = 0, right = follow.cols;
        
        Mat leftPart = mask.row(y).colRange(max(0, half - halfWidth), half);
        if (countNonZero(leftPart) == 0) {
            left = max(0, half - halfWidth);
        } else {
            vector<int> leftIndices;
            for (int x = 0; x < half; ++x) {
                if (mask.at<uchar>(y, x) == 255) {
                    leftIndices.push_back(x);
                }
            }
            if (!leftIndices.empty()) {
                left = std::accumulate(leftIndices.begin(), leftIndices.end(), 0.0) / leftIndices.size();
            }
        }

        Mat rightPart = mask.row(y).colRange(half, min(follow.cols, half + halfWidth));
        if (countNonZero(rightPart) == 0) {
            right = min(follow.cols, half + halfWidth);
        } else {
            vector<int> rightIndices;
            for (int x = half; x < follow.cols; ++x) {
                if (mask.at<uchar>(y, x) == 255) {
                    rightIndices.push_back(x);
                }
            }
            if (!rightIndices.empty()) {
                right = std::accumulate(rightIndices.begin(), rightIndices.end(), 0.0) / rightIndices.size();
            }
        }

        if (y == 255) {
            int WhiteCount = 0;
            for (int x = 0; x < mask.cols; ++x) {
                if (mask.at<uchar>(y, x) == 255) {  // Assuming mask is a single-channel uchar image
                    WhiteCount++;
                }
            }   
            if (WhiteCount >= 200) {
                {
                    if(right != 640 && left !=0 && !boundary_flag){
                        stop_flag = 1;                    
                    }else if(left != 0)
                    {
                        boundary_flag = 1;
                    }else if(right != 640)
                    {
                        boundary_flag = 2;
                    }
                }
                //cout<<"boundary_flag = "<<boundary_flag<<endl;
            }
        }
        if (y == 180)
        {
            if(boundary_flag == 2 || boundary_flag == 1){
                if(left >=20  && right <= 620)
                {
                    boundary_flag = 0;
                    cout<<"find straight road"<<endl;
                }
            }
            
        }
        int mid = (left + right) / 2;
        half = mid;
        follow.at<uchar>(y, mid) = 255;

        if (y == 245) {
            mid_output = mid;
        }

    }

    circle(follow, Point(mid_output, 245), 5, Scalar(255), -1);
    error = follow.cols / 2 - mid_output;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    if(xun_flag == 1){

    static double previous_error = 0.0;

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat img = cv_ptr->image;
    int y_start = 170;
    int y_end = y_start + 270;
    Mat cropped_img = img(Range(y_start, y_end), Range::all());

    Mat img_hsv;
    cvtColor(cropped_img, img_hsv, COLOR_BGR2HSV);
    Mat mask;
    inRange(img_hsv, Scalar(0, 0, 210), Scalar(179, 30, 255), mask);

    Mat follow = mask.clone();
    int error;
    mid(follow, mask, error);

    double derivative = error - previous_error;
    double control_signal = Kp * error + Kd * derivative;

    if (abs(error) <= 10) {
        error = 0;
    }

    previous_error = error;
    // cout<<error<<endl;
    geometry_msgs::Twist twist;
    if(boundary_flag == 1){
	    twist.linear.x = 0.1;
        twist.angular.z = 0.75;
        cout<<"boundary_flag = "<<boundary_flag<<endl;
    }else if(boundary_flag == 2){
        twist.linear.x = 0.1;
        twist.angular.z = -0.75;
        cout<<"boundary_flag = "<<boundary_flag<<endl;
    }else if(stop_flag){
        stop_flag++;
        if(stop_flag==10)
        {
            twist.linear.x = 0;
    	    twist.angular.z = 0;
            xun_flag = 0;
        }else{
            twist.linear.x = 0.2;
    	    twist.angular.z = -control_signal;
        }
        cout<<stop_flag<<endl;
    }else{
	    twist.linear.x = 0.2;
    	twist.angular.z = -control_signal;
        cout<<"straight"<<endl;
    }
    ROS_INFO("Publishing Twist: linear.x=%f, angular.z=%f", twist.linear.x, twist.angular.z);
    cmd_pub.publish(twist);

    // imshow("img", img);
    // imshow("mask", mask);
    //  imshow("follow", follow);
    //  waitKey(1);
    }
}

int main(int argc, char** argv) {

    ros::Time::init();
    ros::Rate r(0.25);
    ros::Rate loop_rate(30); // 10 Hz 
	setlocale(LC_ALL,"");
    ros::init(argc, argv, "send_goal_circle_final");
    
    
    ros::NodeHandle n;	

    ROS_INFO("GOOD");
    ROS_INFO("okkk!");
    ros::Publisher terrorist_pub = n.advertise<std_msgs::Int8>("start_detect",10);//开始识别恐怖分子信息发布
    ros::Publisher wuzi_pub = n.advertise<std_msgs::Int8>("start_detect1",10);//开始识别物资信息发布
    ros::Publisher aid_kit_pub = n.advertise<std_msgs::Int8>("start_detect2",10);//开始识别物资信息发布
    cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber awake_sub = n.subscribe<std_msgs::Int8>("awake_flag", 1, mic_awake_cb);  //语音唤醒订阅
    ros::Subscriber terrorist_sub = n.subscribe<std_msgs::Int8>("terrorist_data",10,doMsg);   //恐怖分子数量订阅
    ros::Subscriber laser_sub = n.subscribe<std_msgs::Int8>("laser_scan",10,laser_sub_cb);   //视觉找到目标物订阅
    ros::Subscriber aid_kit_sub = n.subscribe<std_msgs::Int8>("aid_kit_data",10,aid_kit_sub_cb);   //急救包识别订阅
    ros::Subscriber sub = n.subscribe("/scan", 10, laserCallback);   //雷达消息订阅
    ros::Subscriber subOdom = n.subscribe("odom", 1000, OdomCallback);
    ros::Subscriber sub_line = n.subscribe("/usb_cam/image_raw", 1, imageCallback);
    
    

    start_detect.data = 0;      //初始化恐怖分子识别标志位为0
    start_detect1.data = 0;      //初始化物资识别标志位为0
    start_detect2.data = 0;      //初始化急救包识别标志位为0
    // while (!awake_flag)                  //等待语音唤醒标志位
    // {
    //     ros::spinOnce();
    // }
    ROS_INFO("RUNNING!");
    
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    ros::Time start_time = ros::Time::now();   
    move_base_msgs::MoveBaseGoal goal[12];
    move_base_msgs::MoveBaseGoal point;

    //we'll send a goal to the robot to move 1 meter forward

    goal[0].target_pose.pose.position.x = 0.522;
    goal[0].target_pose.pose.position.y = 0.045;
    goal[0].target_pose.pose.position.z = 0.000;
    goal[0].target_pose.pose.orientation.x = 0.000;
    goal[0].target_pose.pose.orientation.y = 0.000;
    goal[0].target_pose.pose.orientation.z = 0.701;
    goal[0].target_pose.pose.orientation.w = 0.713;  //进恐怖分子识别区域

    goal[1].target_pose.pose.position.x = 0.989;
    goal[1].target_pose.pose.position.y = 1.304;
    goal[1].target_pose.pose.position.z = 0.000;
    goal[1].target_pose.pose.orientation.x = 0.000;
    goal[1].target_pose.pose.orientation.y = 0.000;
    goal[1].target_pose.pose.orientation.z = -0.003;
    goal[1].target_pose.pose.orientation.w = 1.000;  //读取恐怖分子数量

    goal[2].target_pose.pose.position.x = 0.542;
    goal[2].target_pose.pose.position.y = 1.325;
    goal[2].target_pose.pose.position.z = 0.000;
    goal[2].target_pose.pose.orientation.x = 0.000;
    goal[2].target_pose.pose.orientation.y = 0.000;
    goal[2].target_pose.pose.orientation.z = -0.711;
    goal[2].target_pose.pose.orientation.w = 0.703;  //恐怖分子后退点

    goal[3].target_pose.pose.position.x = 0.518;
    goal[3].target_pose.pose.position.y = 0.001;
    goal[3].target_pose.pose.position.z = 0.000;
    goal[3].target_pose.pose.orientation.x = 0.000;
    goal[3].target_pose.pose.orientation.y = 0.000;
    goal[3].target_pose.pose.orientation.z = -0.715;
    goal[3].target_pose.pose.orientation.w = 0.699; //出恐怖分子前的点

    goal[4].target_pose.pose.position.x = 0.016;
    goal[4].target_pose.pose.position.y = 0.047;
    goal[4].target_pose.pose.position.z = 0.000;
    goal[4].target_pose.pose.orientation.x = 0.000;
    goal[4].target_pose.pose.orientation.y = 0.000;
    goal[4].target_pose.pose.orientation.z = -1.000;
    goal[4].target_pose.pose.orientation.w = 0.014; //过坡前的点


    goal[5].target_pose.pose.position.x = -1.950;
    goal[5].target_pose.pose.position.y =  -0.043;
    goal[5].target_pose.pose.position.z = 0.000;
    goal[5].target_pose.pose.orientation.x = 0.000;
    goal[5].target_pose.pose.orientation.y = 0.000;
    goal[5].target_pose.pose.orientation.z = 1.000;
    goal[5].target_pose.pose.orientation.w = 0.004;  //过坡之后到的点

    goal[6].target_pose.pose.position.x = -2.353;
    goal[6].target_pose.pose.position.y = -1.670;
    goal[6].target_pose.pose.position.z = 0.000;
    goal[6].target_pose.pose.orientation.x = 0.000;
    goal[6].target_pose.pose.orientation.y = 0.000;
    goal[6].target_pose.pose.orientation.z = -0.715;
    goal[6].target_pose.pose.orientation.w = 0.699; //读取急救包位置

    goal[7].target_pose.pose.position.x = -1.752;
    goal[7].target_pose.pose.position.y = 0.004;
    goal[7].target_pose.pose.position.z = 0.000;
    goal[7].target_pose.pose.orientation.x = 0.000;
    goal[7].target_pose.pose.orientation.y = 0.000;
    goal[7].target_pose.pose.orientation.z = -0.003;
    goal[7].target_pose.pose.orientation.w = 1.000; //识别区域

    goal[8].target_pose.pose.position.x = -1.840;
    goal[8].target_pose.pose.position.y = 0.093;
    goal[8].target_pose.pose.position.z = 0.000;
    goal[8].target_pose.pose.orientation.x = 0.000;
    goal[8].target_pose.pose.orientation.y = 0.000;
    goal[8].target_pose.pose.orientation.z = -0.017;
    goal[8].target_pose.pose.orientation.w = 1.000; //坡道前停止区域

    goal[9].target_pose.pose.position.x = -0.003;
    goal[9].target_pose.pose.position.y = 0.049;
    goal[9].target_pose.pose.position.z = 0.000;
    goal[9].target_pose.pose.orientation.x = 0.000;
    goal[9].target_pose.pose.orientation.y = 0.000;
    goal[9].target_pose.pose.orientation.z = 0.000;
    goal[9].target_pose.pose.orientation.w = 1.000; //坡道后停止区域


    goal[10].target_pose.pose.position.x = 0.028;
    goal[10].target_pose.pose.position.y = -0.161;
    goal[10].target_pose.pose.position.z = 0.000;
    goal[10].target_pose.pose.orientation.x = 0.000;
    goal[10].target_pose.pose.orientation.y = 0.000;
    goal[10].target_pose.pose.orientation.z = -0.707;
    goal[10].target_pose.pose.orientation.w = 0.707; //巡线区域


    goal[11].target_pose.pose.position.x = 2.023;
    goal[11].target_pose.pose.position.y = -0.027;
    goal[11].target_pose.pose.position.z = 0.000;
    goal[11].target_pose.pose.orientation.x = 0.000;
    goal[11].target_pose.pose.orientation.y = 0.000;
    goal[11].target_pose.pose.orientation.z = 0.704;
    goal[11].target_pose.pose.orientation.w = 0.710; //到达终点


    point.target_pose.pose.position.x = -1.867;
    point.target_pose.pose.position.y = -0.975;
    point.target_pose.pose.position.z = 0.000;
    point.target_pose.pose.orientation.x = 0.000;
    point.target_pose.pose.orientation.y = 0.000;
    point.target_pose.pose.orientation.z = 0.379;
    point.target_pose.pose.orientation.w = 0.925; //识别左边区域



    int i;
    
    for (i = 0; i < 12;i++) {
        goal[i].target_pose.header.frame_id = "map";
        goal[i].target_pose.header.stamp = ros::Time::now();
        ROS_INFO("Sending goal");
        ac.sendGoal(goal[i]);
	
        ac.waitForResult();

        while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {

        }
       

	    if(i==1){
            start_detect.data = 1;                  //识别标志位设为1，则开始识别
            terrorist_pub.publish(start_detect);
            while (!terrorist_data_received) {  
                ros::Duration(0.1).sleep(); // 短暂休眠
                ros::spinOnce(); // 处理回调  
            }
            playAudio(sound_file);                                                     
        }
        // 识别急救包，并播报
        if (i == 6 ) { 
            start_detect2.data = 1;                  //识别标志位设为1，则开始识别
            aid_kit_pub.publish(start_detect2); 
            while(aid_kit_flag == 0){
                ros::Duration(0.1).sleep(); // 短暂休眠
                ros::spinOnce(); // 处理回调 
            }  
              
            ros::Duration(0.5).sleep();
            system("aplay ~/ucar_car/src/simple_navigation_goals/voice/j0.wav");

        }
        // 板前定位
        // if(i == 7){
        //     start_detect1.data = 1;                  //识别标志位设为1，则开始识别
        //     wuzi_pub.publish(start_detect1);
        //     ROS_INFO("start_detect1");
        //     while(laser_flag == 0)
        //     {
        //         if(cplus_flag ==1 ){
        //             break;
        //         }
        //         ros::Duration(0.1).sleep(); // 短暂休眠
        //         ros::spinOnce(); // 处理回调 
        //     }

        //     cplus_flag = 0;
            
        //     if(laser_flag == 0)
        //     {
        //             ROS_INFO("ac goal2");
        //         // ac到另外一个点
        //             point.target_pose.header.frame_id = "map";
        //             point.target_pose.header.stamp = ros::Time::now();
        //             ac.sendGoal(point);
        //             ac.waitForResult();
        //             while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        //         {
        //         }
        //         ROS_INFO("start_detect1_2");
        //         start_detect1.data = 1;                  //识别标志位设为1，则开始识别
        //         wuzi_pub.publish(start_detect1);
        //         while(laser_flag == 0)
        //         {
        //             if(cplus_flag ==1 ){
        //                 break;
        //             }
        //             ros::Duration(0.1).sleep(); // 短暂休眠
        //             ros::spinOnce(); // 处理回调 
        //         }

        //         if(laser_flag == 1)
        //         {
        //         //开启雷达识别
        //         ROS_INFO("雷达开启识别标志位为%d",laser_flag);
        //         laser_process = 1;
        //         while(laser_process == 1 ){
        //             ros::Duration(0.1).sleep(); // 短暂休眠
        //             ros::spinOnce(); // 处理回调 
        //         }
        //         lasergoal.target_pose.header.frame_id = "map";
        //         lasergoal.target_pose.header.stamp = ros::Time::now();
        //         ac.sendGoal(lasergoal);
        //         ac.waitForResult();
        //         while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        //         {
        //         }
        //         playAudio(sound_file1);
        //         laser_flag = 0;
        //         }
        //     }

        //     if(laser_flag == 1)
        //     {
        //         //开启雷达识别
        //         ROS_INFO("雷达开启识别标志位为%d",laser_flag);
        //         laser_process = 1;
        //         while(laser_process == 1 ){
        //             ros::Duration(0.1).sleep(); // 短暂休眠
        //             ros::spinOnce(); // 处理回调 
        //         }
        //         lasergoal.target_pose.header.frame_id = "map";
        //         lasergoal.target_pose.header.stamp = ros::Time::now();
        //         ac.sendGoal(lasergoal);
        //         ac.waitForResult();
        //         while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        //         {

        //         }
        //         playAudio(sound_file1);
        //     }
            

        // }
        //巡线
        if(i == 10){
            ROS_INFO("xunxian!");
            xun_flag = 1;
	     
        while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
         }
        }
        ROS_INFO("Arrive at!");

    }
    return 0;
}
