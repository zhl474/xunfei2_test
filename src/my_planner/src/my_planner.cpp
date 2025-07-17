#include "my_planner.h"


PLUGINLIB_EXPORT_CLASS( my_planner::MyPlanner, nav_core::BaseLocalPlanner)

namespace my_planner 
{
    // 构造函数：初始化指针成员为nullptr
    MyPlanner::MyPlanner() : tf_listener_(nullptr), costmap_ros_(nullptr)
    {
        setlocale(LC_ALL,"");
        out.open(output_file, fourcc, 7, cv::Size(640, 270));
        displayStream << std::fixed << std::setprecision(2);
    }
    MyPlanner::~MyPlanner()
    {
        // 检查指针是否有效，然后释放它
        if(tf_listener_ != nullptr)
        {
            delete tf_listener_;
            tf_listener_ = nullptr; // 释放后置空，防止悬挂指针
        }
    }

    // 移除了在这里的全局变量定义

    
    void MyPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_WARN("本地规划器，启动！");
        tf_listener_ = new tf::TransformListener();
        costmap_ros_ = costmap_ros;
        // 为此插件创建一个私有的节点句柄，用于访问其私有命名空间下的参数
        //私有命名空间是 /move_base/MyPlanner
        ros::NodeHandle private_nh("/move_base/MyPlanner" );

        ROS_INFO("为 %s 加载参数...", name.c_str());

        // 使用 .param() 方法读取参数。如果参数服务器上没有该参数，则使用第三个参数作为默认值。
        private_nh.param("path_linear_x_gain", path_linear_x_gain_, 2.0);
        private_nh.param("path_linear_y_gain", path_linear_y_gain_, 0.5);
        private_nh.param("path_angular_gain", path_angular_gain_, 6.8);
        private_nh.param("lookahead_dist", lookahead_dist_, 0.2);
        
        private_nh.param("goal_dist_threshold", goal_dist_threshold_, 0.10);
        private_nh.param("final_pose_linear_gain", final_pose_linear_gain_, 2.5);
        private_nh.param("final_pose_angular_gain", final_pose_angular_gain_, 1.5);
        private_nh.param("goal_yaw_tolerance", goal_yaw_tolerance_, 0.1);

        private_nh.param("collision_check_lookahead_points", collision_check_lookahead_points_, 10);
        private_nh.param("visualization_scale_factor", visualization_scale_factor_, 5);
        private_nh.param("visualize_costmap", visualize_costmap_, false);
        //------------------------------------用于动态速度控制的参数-----------------------------
        
        private_nh.param("curvature_damping_factor", curvature_damping_factor_, 0.5);
        private_nh.param("curvature_penalty_gain", curvature_penalty_gain_, 15.0); // 新增参数，需要调试一个合适的值
        ROS_INFO("curvature_penalty_gain_=%f",curvature_penalty_gain_);
        initial_rotation_done_ = false;

        private_nh.param("P", P_, 0.5);
        private_nh.param("I", I_, 0.5);
        private_nh.param("D", D_, 0.0);
        private_nh.param("point", point_,15);
        // ROS_INFO("加载参数path_linear_x_gain_=%f",path_linear_x_gain_);
        // ROS_INFO("加载参数path_linear_y_gain_=%f",path_linear_y_gain_);
    }
    //移除这里的全局变量声明，它们会覆盖类的成员变量
    // std::vector<geometry_msgs::PoseStamped> global_plan_;
    // int target_index_;
    // bool pose_adjusting_;
    // bool goal_reached_; 
    bool MyPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        // ROS_INFO("SETPLAN");
        target_index_ = 0;
        global_plan_ = plan;
        pose_adjusting_ = false;
        goal_reached_ = false;
        //每次设置新路径时，都将“初始旋转”标志重置为 false,确保下一次导航会执行初始旋转调整朝向
        // initial_rotation_done_ = false;
       
        return true;
    }

    bool MyPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {   
        int final_index = global_plan_.size()-1;
        geometry_msgs::PoseStamped pose_final;
        global_plan_[final_index].header.stamp = ros::Time(0);
        tf_listener_->transformPose("base_link",global_plan_[final_index],pose_final);
        if(pose_adjusting_ == false)//如果未进入姿态调整状态
        {
            double dx = pose_final.pose.position.x;
            double dy = pose_final.pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            if(dist < goal_dist_threshold_)//判定是否到达目标点附近的距离阈值
                pose_adjusting_ = true;
        }
        if(pose_adjusting_ == true)
        {
            double final_yaw = tf::getYaw(pose_final.pose.orientation);
            // ROS_WARN("调整最终姿态，final_yaw = %.2f",final_yaw);
            cmd_vel.linear.x = pose_final.pose.position.x * final_pose_linear_gain_;//到达目标点附近后调整位姿的速度比例系数
            if(final_yaw>0) cmd_vel.angular.z = std::max(std::min(final_yaw * final_pose_angular_gain_,2.5),0.6);
            else cmd_vel.angular.z = std::min(std::max(final_yaw * final_pose_angular_gain_,-2.5),-0.6);
            if(abs(final_yaw) < goal_yaw_tolerance_)
            {
                goal_reached_ = true;
                ROS_WARN("到达终点！");
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                initial_rotation_done_ = false;//路径结束判断：遇到障碍物或者完成最终姿态调整
            }
            return true;
        }

        geometry_msgs::PoseStamped target_pose;
        //-------------
        double error = 0.0;
        int count = 0;
        //----------------

        for(int i=target_index_;i<global_plan_.size();i++)
        {
            geometry_msgs::PoseStamped pose_base;
            global_plan_[i].header.stamp = ros::Time(0);
            tf_listener_->transformPose("base_link",global_plan_[i],pose_base);
            double dx = pose_base.pose.position.x;
            double dy = pose_base.pose.position.y;
            // double dist = std::sqrt(dx*dx + dy*dy);

            //-------------
            error += dy*exp(count/-30);
            //-------------
            count ++;
            if (count > point_) //选取的临时目标点的距离阈值
            {
                target_pose = pose_base;
                target_index_ = i;
                // ROS_WARN("选择第 %d 个路径点作为临时目标，距离=%.2f",target_index_,dist);
                break;
            }

            if(i == global_plan_.size()-1)//  如果最远的点也没超出阈值，就取最远的点
                target_pose = pose_base; 
        }
        error = error * (1 - exp(-1.0/30.0)) / (1 - exp(-(count+1.0)/30.0));
        ROS_INFO("归一化误差%f",error);

        if (!initial_rotation_done_) //如果还未进行过初始姿态调整，说明是第一个目标点
        {
            double angle_to_target = atan2(target_pose.pose.position.y, target_pose.pose.position.x);
            // ROS_INFO("开始进行初始姿态调整");
            if (std::abs(angle_to_target) < goal_yaw_tolerance_) {
                ROS_INFO("初始姿态已对准，设置标志位并开始正常行驶。");
                initial_rotation_done_ = true;
             
                // 对准后，这个周期就可以开始前进了，直接执行下面的正常行驶逻辑
            } else {//旋转朝向第一个导航点
                
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                if(angle_to_target>0) cmd_vel.angular.z = std::min(std::max(angle_to_target * final_pose_angular_gain_,0.6),2.5); // 与最终姿态调整共用参数
                else cmd_vel.angular.z = std::max(std::min(angle_to_target * final_pose_angular_gain_,-0.6),-2.5);
                
                return true; 
            }
        }

        //-----------------新增通过计算当前全局路径到target_index的最大曲率控制转弯速度--------------------
        
        // // double max_curvature = 0.0;//最大曲率
        // double total_curvature = 0.0;
        // double dynamic_x_gain = path_linear_x_gain_; // 默认使用参数文件中的基础x增益

        // // 在 [0, target_index_] 区间内计算最大曲率
        // // 需要至少3个点(i-1, i, i+1)来计算曲率，所以 target_index_ 必须至少为2
        // if (target_index_ >= 2) 
        // {
        //     // 循环遍历从路径起点到 target_index 的前一个点
        //     // 计算点 i 处的曲率，需要 p_i-1, p_i, p_i+1
        //     for (size_t i = 1; i < target_index_; ++i)
        //     {
        //         const auto& p0 = global_plan_[i-1].pose.position;
        //         const auto& p1 = global_plan_[i].pose.position;
        //         const auto& p2 = global_plan_[i+1].pose.position;
                
        //         // 计算向量 u (p0->p1) 和 v (p1->p2)
        //         double ux = p1.x - p0.x, uy = p1.y - p0.y;
        //         double vx = p2.x - p1.x, vy = p2.y - p1.y;

        //         double norm_u = std::sqrt(ux*ux + uy*uy);
        //         double norm_v = std::sqrt(vx*vx + vy*vy);
        //         double w_norm = std::sqrt(std::pow(p2.x - p0.x, 2) + std::pow(p2.y - p0.y, 2));

        //         // 使用Menger曲率公式: K = 2 * |cross(u, v)| / (|u| * |v| * |u+v|)
        //         double cross_product_mag = std::abs(ux * vy - uy * vx);

        //         if (norm_u > 1e-6 && norm_v > 1e-6 && w_norm > 1e-6) {
        //             double curvature = (2.0 * cross_product_mag) / (norm_u * norm_v * w_norm);
        //             // if (curvature > max_curvature) {最大曲率
        //             //     max_curvature = curvature;
        //             // }
        //             total_curvature += curvature;
        //         }
        //     }
        // }

        // // 根据最大曲率调整速度增益
        // // if (max_curvature > 10.0) // 设置一个阈值，避免在近似直线上过度反应，曲率超过阈值才会减速
        // // {
        // //     //速度增益只与该路径段的最大曲率有关
        // //     double penalty_term = max_curvature * curvature_penalty_gain_;
        // //     dynamic_x_gain = path_linear_x_gain_ / (curvature_damping_factor_ + penalty_term);
        // // }

        // // 对动态增益进行限幅，保证安全和稳定
        // // dynamic_x_gain = std::max(1.5, dynamic_x_gain); // 最小增益，防止失速
        // // dynamic_x_gain = std::min(dynamic_x_gain, path_linear_x_gain_); // 最大增益，防止飙车

        // dynamic_x_gain = dynamic_x_gain*std::min(std::max(exp((total_curvature/target_index_-4)/-25),0.5),2.0);

        // ROS_INFO("当前dynamic_x_gain=%f",dynamic_x_gain);
        // ROS_INFO("当前max_curvature=%f",max_curvature);
        
        intergration += error*0.03;
        intergration = std::min(std::max(intergration,-1.0),1.0);
        cmd_vel.linear.x = path_linear_x_gain_;
        cmd_vel.angular.z = error*P_;
        ROS_INFO("P:%f",error*P_);
        ROS_INFO("I:%f",intergration*I_);
        // ROS_INFO("P:%f",error*path_angular_gain_);


        //--------------------------全局路径显示，省去节省算力---------------------------------------------
        cv::Mat plan_image(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));        
        for(int i=target_index_;i<global_plan_.size();i++)
        {
            geometry_msgs::PoseStamped pose_base;
            global_plan_[i].header.stamp = ros::Time(0);
            tf_listener_->transformPose("base_link",global_plan_[i],pose_base);
            int cv_x = 300 - pose_base.pose.position.y*100;
            int cv_y = 300 - pose_base.pose.position.x*100;
            cv::circle(plan_image, cv::Point(cv_x,cv_y), 1, cv::Scalar(255,0,255)); 
            if(i-target_index_>point_) break;
        }
        cv::circle(plan_image, cv::Point(300, 300), 15, cv::Scalar(0, 255, 0));
        cv::line(plan_image, cv::Point(65, 300), cv::Point(510, 300), cv::Scalar(0, 255, 0), 1);
        cv::line(plan_image, cv::Point(300, 45), cv::Point(300, 555), cv::Scalar(0, 255, 0), 1);

        cv::namedWindow("Plan");
        cv::imshow("Plan", plan_image);
        cv::waitKey(1);
        // out.write(plan_image);
        
        return true;
    }
    bool MyPlanner::isGoalReached()
    {
        return goal_reached_;
    }
} // namespace my_planner