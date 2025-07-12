// #include <ros/ros.h>
// #include <sensor_msgs/LaserScan.h>
// #include <geometry_msgs/Point.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <vector>
// #include <cmath>
// #include <iostream>
// #include <algorithm>

// #include "ztestnav2025/lidar_process.h" // 服务消息头文件

// // 突变点结构体
// struct Mutationpoints {
//     size_t index;        // 激光点索引
//     geometry_msgs::Point position; // 位置
//     float range;         // 距离
//     float diff;          // 距离变化量
// };

// class LidarProcessor {
// private:
//     ros::NodeHandle nh_; 
//     ros::Subscriber sub_;
//     ros::ServiceServer server_;
//     sensor_msgs::LaserScan latest_scan_;
//     bool new_scan_received_;
    
//     // TF相关
//     tf2_ros::Buffer tf_buffer_;
//     tf2_ros::TransformListener tf_listener_;
    
//     // 配置参数
//     float distance_threshold_ = 0.03;  // 聚类距离阈值
//     float mutation_threshold_ = 0.15;  // 突变点阈值
//     float filter_x_min_ = -0.2;        // 过滤区域x最小值
//     float filter_x_max_ = 2.7;         // 过滤区域x最大值
//     float filter_y_min_ = -2.2;        // 过滤区域y最小值
//     float filter_y_max_ = 0.2;         // 过滤区域y最大值
//     double left_angle_limit_ = -3.0;   // 左侧角度限制（弧度）
//     double right_angle_limit_ = 2.9;   // 右侧角度限制（弧度）
    
//     // 板子检测结果
//     std::vector<std::vector<float>> boards_; // 存储检测到的板子信息 [x, y, width]

//     void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
//         latest_scan_ = *scan_msg;  // 更新最新扫描数据
//         new_scan_received_ = true;
//     }

//     // 将激光数据从雷达坐标系转换到地图坐标系
//     std::vector<geometry_msgs::Point> transformLaserToMap(const std::vector<float>& ranges) {
//         std::vector<geometry_msgs::Point> points;
//         geometry_msgs::Point p;
        
//         try {
//             // 获取从激光雷达坐标系到地图坐标系的变换
//             geometry_msgs::TransformStamped transform = 
//                 tf_buffer_.lookupTransform("map", latest_scan_.header.frame_id, ros::Time(0), ros::Duration(0.5));
            
//             // 遍历所有激光点，进行坐标转换
//             for (size_t i = 0; i < ranges.size(); ++i) {
//                 // 忽略无效点
//                 if (std::isinf(ranges[i]) || std::isnan(ranges[i]) || ranges[i] == 0.0f) {
//                     continue;
//                 }
                
//                 // 计算在激光雷达坐标系下的坐标
//                 float angle = latest_scan_.angle_min + i * latest_scan_.angle_increment;
//                 p.x = ranges[i] * cos(angle);
//                 p.y = ranges[i] * sin(angle);
//                 p.z = 0.0;
                
//                 // 转换到地图坐标系
//                 geometry_msgs::Point transformed_point;
//                 tf2::doTransform(p, transformed_point, transform);
//                 points.push_back(transformed_point);
//             }
//         } catch (tf2::TransformException& ex) {
//             ROS_WARN("Failed to transform from %s to map: %s", 
//                      latest_scan_.header.frame_id.c_str(), ex.what());
//             return std::vector<geometry_msgs::Point>(); // 返回空向量
//         }
        
//         return points;
//     }
    
//     // 检测突变点（物体边缘）
//     std::vector<Mutationpoints> detectMutationPoints(const std::vector<float>& ranges, 
//                                                     const std::vector<geometry_msgs::Point>& points) {
//         std::vector<Mutationpoints> mutations;
        
//         if (ranges.size() != points.size() || ranges.empty()) {
//             ROS_WARN("Invalid input data for mutation point detection");
//             return mutations;
//         }
        
//         // 计算角度限制对应的索引范围
//         size_t min_idx = std::max(0, static_cast<int>((left_angle_limit_ - latest_scan_.angle_min) / latest_scan_.angle_increment));
//         size_t max_idx = std::min(ranges.size() - 1, 
//                                  static_cast<size_t>((right_angle_limit_ - latest_scan_.angle_min) / latest_scan_.angle_increment));
        
//         // 寻找突变点
//         for (size_t i = min_idx + 1; i < max_idx; ++i) {
//             // 忽略无效点
//             if (std::isinf(ranges[i]) || std::isnan(ranges[i]) || 
//                 std::isinf(ranges[i-1]) || std::isnan(ranges[i-1])) {
//                 continue;
//             }
            
//             // 计算与前一个点的距离差
//             float diff = ranges[i] - ranges[i-1];
            
//             // 检查是否为突变点（正向或负向突变）
//             if (std::abs(diff) > mutation_threshold_) {
//                 Mutationpoints mp;
//                 mp.index = i;
//                 mp.position = points[i];
//                 mp.range = ranges[i];
//                 mp.diff = diff;
                
//                 mutations.push_back(mp);
                
//                 // 输出突变点信息（调试用）
//                 ROS_DEBUG("Mutation point detected at index %zu: distance=%.3f, diff=%.3f", 
//                          i, ranges[i], diff);
//             }
//         }
        
//         return mutations;
//     }
    
//     // 判断点是否在墙附近（简化版）
//     bool isNearWall(const geometry_msgs::Point& p) {
//         // 检查点是否接近过滤区域边界
//         return (p.x < filter_x_min_ + 0.1 || p.x > filter_x_max_ - 0.1 ||
//                 p.y < filter_y_min_ + 0.1 || p.y > filter_y_max_ - 0.1);
//     }

//     bool lidar_process(ztestnav2025::lidar_process::Request& req, 
//                       ztestnav2025::lidar_process::Response& resp) {
//         if (!new_scan_received_) {
//             ROS_WARN("No scan data available yet");
//             return false;
//         }
        
//         // 清空之前的检测结果
//         boards_.clear();
        
//         // 提取激光数据
//         std::vector<float> ranges = latest_scan_.ranges;
        
//         // 1. 将激光数据转换到地图坐标系
//         std::vector<geometry_msgs::Point> points_map = transformLaserToMap(ranges);
        
//         if (points_map.empty()) {
//             ROS_WARN("Failed to transform laser points to map frame");
//             return false;
//         }
        
//         // 2. 检测突变点
//         std::vector<Mutationpoints> mutations = detectMutationPoints(ranges, points_map);
        
//         // 3. 使用突变点进行聚类（改进版）
//         std::vector<int> clusters(ranges.size(), 0); // 0:未分类
//         int cluster_id = 1;
        
//         // 基于突变点分割点云
//         if (!mutations.empty()) {
//             // 对突变点按索引排序
//             std::sort(mutations.begin(), mutations.end(), 
//                      [](const Mutationpoints& a, const Mutationpoints& b) {
//                          return a.index < b.index;
//                      });
            
//             // 为每个聚类分配ID
//             for (size_t i = 0; i < mutations.size() - 1; ++i) {
//                 for (size_t j = mutations[i].index + 1; j < mutations[i+1].index; ++j) {
//                     clusters[j] = cluster_id;
//                 }
//                 cluster_id++;
//             }
            
//             // 处理最后一个突变点之后的点
//             for (size_t j = mutations.back().index + 1; j < ranges.size(); ++j) {
//                 clusters[j] = cluster_id;
//             }
            
//             // 处理第一个突变点之前的点
//             for (size_t j = 0; j < mutations[0].index; ++j) {
//                 clusters[j] = cluster_id; // 假设首尾相连（环形点云）
//             }
//         } else {
//             // 如果没有突变点，使用简单聚类
//             for (size_t i = 1; i < ranges.size(); ++i) {
//                 if (std::abs(ranges[i] - ranges[i-1]) < distance_threshold_) {
//                     clusters[i] = clusters[i-1];
//                 } else {
//                     clusters[i] = clusters[i-1] + 1;
//                 }
//             }
//             cluster_id = clusters.back() + 1;
//         }
        
//         // 4. 分析聚类结果，提取板子信息
//         std::vector<std::vector<int>> cluster_indices(cluster_id);
//         for (size_t i = 0; i < ranges.size(); ++i) {
//             if (clusters[i] > 0 && !isNearWall(points_map[i])) {
//                 cluster_indices[clusters[i]].push_back(i);
//             }
//         }
        
//         // 5. 计算每个聚类的中心点和宽度
//         for (const auto& indices : cluster_indices) {
//             if (indices.size() < 10) continue; // 忽略过小的聚类
            

//             double sum_x = 0.0;
//             double sum_y = 0.0;


//             double min_x = std::numeric_limits<double>::max();
//             double max_x = std::numeric_limits<double>::lowest();
//             double min_y = std::numeric_limits<double>::max();
//             double max_y = std::numeric_limits<double>::lowest();

            
//             for (int idx : indices) {
//                 sum_x += points_map[idx].x;
//                 sum_y += points_map[idx].y;
//                 min_x = std::min(min_x, points_map[idx].x);
//                 max_x = std::max(max_x, points_map[idx].x);
//                 min_y = std::min(min_y, points_map[idx].y);
//                 max_y = std::max(max_y, points_map[idx].y);
//             }
            
//             float center_x = sum_x / indices.size();
//             float center_y = sum_y / indices.size();
//             float width = max_x - min_x;
//             float height = max_y - min_y;
            
//             // 只保留符合板子尺寸的聚类
//             if ((width > 0.1 && width < 1.5) || (height > 0.1 && height < 1.5)) {
//                 boards_.push_back({center_x, center_y, std::max(width, height)});
//             }
//         }
        
//         // 构建服务响应
//         resp.lidar_results.clear();
//         resp.lidar_results.push_back(boards_.size()); // 板子数量
        
//         for (const auto& board : boards_) {
//             resp.lidar_results.push_back(board[0]); // x坐标
//             resp.lidar_results.push_back(board[1]); // y坐标
//             resp.lidar_results.push_back(board[2]); // 宽度
//         }
        
//         // 输出调试信息
//         ROS_INFO("检测到 %zu 个板子", boards_.size());
//         for (const auto& board : boards_) {
//             ROS_INFO("  位置: (%.2f, %.2f), 尺寸: %.2f 米", board[0], board[1], board[2]);
//         }
        
//         return true;
//     }
    
// public:
//     LidarProcessor() : nh_("~"), new_scan_received_(false), tf_listener_(tf_buffer_) {
//         // 订阅雷达数据
//         sub_ = nh_.subscribe("/scan", 10, &LidarProcessor::scanCallback, this);
//         // 注册服务
//         server_ = nh_.advertiseService("lidar_process", &LidarProcessor::lidar_process, this);
//         ROS_INFO("雷达处理器初始化完成");
        
//         // 从参数服务器加载配置
//         nh_.param<float>("distance_threshold", distance_threshold_, 0.03);
//         nh_.param<float>("mutation_threshold", mutation_threshold_, 0.15);
//         nh_.param<float>("filter_x_min", filter_x_min_, -0.2);
//         nh_.param<float>("filter_x_max", filter_x_max_, 2.7);
//         nh_.param<float>("filter_y_min", filter_y_min_, -2.2);
//         nh_.param<float>("filter_y_max", filter_y_max_, 0.2);
//         nh_.param<double>("left_angle_limit", left_angle_limit_, -3.0);
//         nh_.param<double>("right_angle_limit", right_angle_limit_, 2.9);
//     }
// };

// int main(int argc, char** argv) {
//     setlocale(LC_ALL,"");
//     ros::init(argc, argv, "lidar_test_zhn");
//     LidarProcessor processor;
//     ros::spin();
//     return 0;
// }
