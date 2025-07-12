#include <ros/ros.h>  
#include <sensor_msgs/LaserScan.h>  
#include <vector>  
#include <cmath>  
#include <limits>  
#include <map>  
#include <locale.h>
#include <cmath> 
#include <sensor_msgs/LaserScan.h>  
#include <geometry_msgs/PoseWithCovarianceStamped.h>  
#include <boost/shared_ptr.hpp>  



std::set<int> used_edges; 

struct BoardEdge {  
    float angle;  
    float distance;  
    bool is_left; // 标记为左边缘或右边缘  
};

std::map<int, std::pair<BoardEdge, BoardEdge>> boards; // 板子ID，右边缘和左边缘  
int next_board_id = 1;

boost::shared_ptr<sensor_msgs::LaserScan> g_laser_scan;  
std::vector<std::pair<int, bool>> edges; // 用于存储边缘的索引


// goal.target_pose.pose.position.x = -1.914;
// goal.target_pose.pose.position.y = -0.043;
// goal.target_pose.pose.position.z = 0.000;
// goal.target_pose.pose.orientation.x = 0.000;
// goal.target_pose.pose.orientation.y = 0.000;
// goal.target_pose.pose.orientation.z = -0.715;
// goal.target_pose.pose.orientation.w = 0.699; 

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
   for (int i = 165; i < 203; ++i) {  //145度到215度
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
        ROS_INFO("检测到边缘: 序号 %zu, 角度 %.2f°, 距离 %.2f 米, %s边缘",  
                 edges.size() - 1 - (&edge - &edges[0]), angle, distance,  
                 edge.second ? "左" : "右");  
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
    
    // 输出检测到的板子信息  
    for (const auto& board : boards) {  
        ROS_INFO("检测到的板子 #%d: 右边缘角度 %.2f°, 距离 %.2f 米; 左边缘角度 %.2f°, 距离 %.2f 米",  
                 board.first, board.second.first.angle, board.second.first.distance,  
                 board.second.second.angle, board.second.second.distance);  
    }  
    ROS_INFO("总共检测到 %zu 个板子", boards.size());
    //检测初始坐标
    float car_x = -1.90; //  
    float car_y = -0.02; //   
    float qx = 0;    // 小车朝向的四元数x分量（可能始终为0）  
    float qy = 0;    // 同上，y分量可能也始终为0  
    float qz = -0.753;    // 小车朝向的四元数z分量  
    float qw = 0.657;
    //计算小车角度
    float yaw_radian= quaternionToYawRadian(qx,qy,qz,qw);
    for (const auto& board : boards) {        
    //中心点距离
    float b = board.second.first.distance;  // 右边缘距离  
    float c = board.second.second.distance; // 左边缘距离  
    float a = 0.5;                          // 板子宽度
    float right_side = 2 * (b * b + c * c) - a * a;
    float center_distance = std::sqrt(right_side / 4);

    //用余弦定理计算与右边缘的夹角
    float angele_rag =acos((pow(center_distance,2)+pow(b,2)-0.0625)/(2*center_distance*b));
    // 计算中心线角度（雷达坐标系统中的绝对角度）  
    float center_angle_rad = board.second.first.angle* (M_PI / 180.0) + angele_rag;  

    //调整至全图坐标的偏向弧度
    float adjusted_board_angle_radian = center_angle_rad - M_PI;
    //板子相对于全局的弧度
    float global_board_angle_radian = yaw_radian + adjusted_board_angle_radian;

    float center_angle_deg = center_angle_rad * (180/M_PI);

    float x1 = car_x + (center_distance-0.5) * cos(global_board_angle_radian);  
    float y1 = car_y + (center_distance-0.5) * sin(global_board_angle_radian);  
    
    float x = 0.0f; // 绕Z轴旋转时，x分量为0  
    float y = 0.0f; // 绕Z轴旋转时，y分量为0  
    float z = sinf(global_board_angle_radian / 2.0f);
    float w = cosf(global_board_angle_radian / 2.0f); 

    // 输出板子信息，包括中心线角度和中心点距离  
    ROS_INFO("板子 #%d 中心线角度: %.2f°，全局旋转四元数分量: x=%.4f, y=%.4f, z=%.4f, w=%.4f,中心点距离雷达: %.2f 米，板子的坐标为（%.2f,%.2f)", 
             board.first, center_angle_deg,x,y,z,w,center_distance,x1,y1);  
    
}  
}  
  
int main(int argc, char **argv) {  
    ros::init(argc, argv, "board_detector");  
    setlocale(LC_ALL, "");  
    ros::NodeHandle nh;  
    ros::Subscriber sub = nh.subscribe("/scan", 10, laserCallback);  
    ros::spin();  
    return 0;  
}