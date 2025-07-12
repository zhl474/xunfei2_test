#include <ros/ros.h>  
#include <sensor_msgs/LaserScan.h>  
#include <vector>  
#include <cmath>  
#include <limits>  
#include <map>  
#include <set>  
  
// 用于存储板子的信息  
struct Board {  
    int id;  
    float left_angle, left_distance;  
    float right_angle, right_distance;  
    bool is_complete;  // 标记板子是否检测到了左右两个边缘  
};  
  
std::vector<Board> boards;  
int next_board_id = 1;  // 用于生成唯一的板子ID  
  
// 激光雷达数据回调函数  
void laserCallback(const sensor_msgs::LaserScanConstPtr& scan) {  
    int num_ranges = scan->ranges.size();  
  
    // 处理毛刺点和无效值（这里简化处理，只过滤inf和0）  
    std::vector<float> filtered_ranges(scan->ranges.begin(), scan->ranges.end());  
    for (int i = 0; i < num_ranges; ++i) {  
        if (std::isinf(filtered_ranges[i]) || filtered_ranges[i] == 0) {  
            filtered_ranges[i] = std::numeric_limits<float>::max();  // 使用一个大值代替无效值  
        }  
    }  
  
    // 检测边缘并尝试匹配板子  
    for (int i = 0; i < num_ranges - 1; ++i) {  
        float diff = std::abs(filtered_ranges[i] - filtered_ranges[i + 1]);  
        if (diff > 0.2) {  // 跳变超过0.2米视为边缘  
            bool is_left = filtered_ranges[i] < filtered_ranges[i + 1];  
            float angle = i * 360.0 / num_ranges;  
            float distance = scan->ranges[i];  
  
            // 尝试匹配现有板子的边缘  
            bool matched = false;  
            for (auto& board : boards) {  
                if (is_left && !board.is_complete && board.right_distance != 0.0 &&  
                    std::abs(angle - board.right_angle) < 10.0 &&  // 角度差小于10度  
                    std::abs(distance - board.right_distance) < 0.1) {  // 距离差小于0.1米  
                    board.left_angle = angle;  
                    board.left_distance = distance;  
                    board.is_complete = true;  
                    ROS_INFO("Matched left edge of board %d with angles %.2f, %.2f degrees",  
                             board.id, board.left_angle, board.right_angle);  
                    matched = true;  
                    break;  
                } else if (!is_left && !board.is_complete && board.left_distance != 0.0 &&  
                           std::abs(angle - board.left_angle) < 10.0 &&  
                           std::abs(distance - board.left_distance) < 0.1) {  
                    board.right_angle = angle;  
                    board.right_distance = distance;  
                    board.is_complete = true;  
                    ROS_INFO("Matched right edge of board %d with angles %.2f, %.2f degrees",  
                             board.id, board.left_angle, board.right_angle);  
                    matched = true;  
                    break;  
                }  
            }  
  
            // 如果没有匹配到现有板子，则创建新的板子（但只记录一个边缘）  
            if (!matched) {  
                Board new_board{next_board_id++, angle, distance, 0.0, 0.0, false};  
                if (is_left) {  
                    new_board.left_angle = angle;  
                    new_board.left_distance = distance;  
                } else {  
                    new_board.right_angle = angle;  
                    new_board.right_distance = distance;  
                }  
                boards.push_back(new_board);  
                ROS_INFO("Detected new board %d with edge at %.2f degrees", new_board.id, angle);  
            }  
        }  
    }  
  
    // 输出完整的板子信息  
    for (const auto& board : boards) {  
        if (board.is_complete) {  
            ROS_INFO("Board %d: Left edge at %.2f degrees, %.2f meters; Right edge at %.2f degrees, %.2f meters",  
                     board.id, board.left_angle, board.left_distance, board.right_angle, board.right_distance);  
        }  
    }  
  
    // 注意：此代码示例假设边缘是连续检测的，并且只考虑相邻点之间的差异来检测边缘。  
    // 在实际应用中，可能需要更复杂的逻辑来处理非相邻边缘、噪声和遮挡。  
}  
  
int main(int argc, char** argv) {  
    ros::init(argc, argv, "board_detector");  
    ros::NodeHandle nh;  
    ros::Subscriber sub = nh.subscribe("/scan", 10, laserCallback);  
    ros::spin();  
    return 0;  
}