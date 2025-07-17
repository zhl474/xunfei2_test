#ifndef MY_PLANNER_H_
#define MY_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h> // 保持使用旧的tf
#include <vector>
#include <string>

// 向前声明 tf2_ros::Buffer，因为 initialize 函数签名需要它
namespace tf2_ros {
    class Buffer;
}
namespace my_planner 
{
    class MyPlanner : public nav_core::BaseLocalPlanner 
    {
        public:
            MyPlanner();
            ~MyPlanner();

            void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
            bool isGoalReached();
        private:
            // 将所有变量移入类定义中，成为成员变量
            tf::TransformListener* tf_listener_;
            costmap_2d::Costmap2DROS* costmap_ros_;
            std::vector<geometry_msgs::PoseStamped> global_plan_;
    
            int target_index_;
            bool pose_adjusting_;
            bool goal_reached_;
            bool initial_rotation_done_;//用于判断初始旋转是否完成的标志
            // 新增：用于存储从参数服务器读取的值的成员变量
            double path_linear_x_gain_, path_linear_y_gain_, path_angular_gain_, lookahead_dist_;
            double goal_dist_threshold_, final_pose_linear_gain_, final_pose_angular_gain_, goal_yaw_tolerance_;
            int collision_check_lookahead_points_;
            int visualization_scale_factor_;
            bool visualize_costmap_;
            // ================= 曲率动态速度控制的成员变量 =================
            
            double curvature_damping_factor_;   // 曲率影响的阻尼因子
            double curvature_penalty_gain_;     // 新增：曲率惩罚增益，用于调节最大曲率对速度的影响

            double P_,I_,D_;
            double intergration = 0.0;
            double pre_error = 0.0;
    };
} // namespace my_planner
 
#endif // MY_PLANNER_H_