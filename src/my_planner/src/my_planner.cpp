#include "my_planner.h"
#include <tf/tf.h>//沟槽的move_base源码用的TF1，这里用TF2就会导致重启时odom,amcl,globalplan出现一堆莫名其妙的错误，甚至与启动位置有关，原理不详，疑似机魂不悦
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

PLUGINLIB_EXPORT_CLASS( my_planner::MyPlanner, nav_core::BaseLocalPlanner)

namespace my_planner 
{
    // 构造函数：初始化指针成员为nullptr
    MyPlanner::MyPlanner() : tf_listener_(nullptr), costmap_ros_(nullptr), target_index_(0), pose_adjusting_(false), goal_reached_(false), initial_rotation_done_(false)//初始化指针和变量，防止在某些情况下触发exit code = -11的段错误
    {
        setlocale(LC_ALL,"");
    }
    MyPlanner::~MyPlanner()
    {
        ROS_WARN(">>> 析构函数被调用 <<<");//
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
        if (!tf || !costmap_ros) {
            ROS_FATAL("致命错误: TF Buffer 或 CostmapROS 的指针为空!");
            // 抛出异常是更健壮的做法，能让 move_base 知道插件初始化失败
            throw std::runtime_error("TF Buffer 或 CostmapROS 的指针为空!");
        }
        
        if(tf_listener_ == nullptr)
        {
            tf_listener_ = new tf::TransformListener();
        }

        costmap_ros_ = costmap_ros;
        ROS_INFO("指针检查通过，tf_buffer 和 costmap_ros 已接收。");

        costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
        if (!costmap) {
            ROS_FATAL("致命错误: 代价地图尚未初始化!");
            throw std::runtime_error("代价地图尚未初始化!");
        }
        ROS_INFO("代价地图功能检查通过，尺寸: %d x %d。",
             costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
        
        

        // 为此插件创建一个私有的节点句柄，用于访问其私有命名空间下的参数
        //私有命名空间是 /move_base/MyPlanner
        ros::NodeHandle private_nh("/move_base/MyPlanner" );

        ROS_INFO("为 %s 加载参数...", name.c_str());

        // 使用 .param() 方法读取参数。如果参数服务器上没有该参数，则使用第三个参数作为默认值。
        private_nh.param("path_linear_x_gain", path_linear_x_gain_, 2.0);
        private_nh.param("path_linear_y_gain", path_linear_y_gain_, 0.5);
        private_nh.param("path_angular_gain", path_angular_gain_, 6.8);
        private_nh.param("angular_limit", angular_limit_, 0.08); 
        private_nh.param("lookahead_dist", lookahead_dist_, 0.2);
        
        private_nh.param("goal_dist_threshold", goal_dist_threshold_, 0.10);
        private_nh.param("final_pose_linear_gain", final_pose_linear_gain_, 2.5);
        private_nh.param("final_pose_angular_gain", final_pose_angular_gain_, 1.5);
        private_nh.param("goal_yaw_tolerance", goal_yaw_tolerance_, 0.1);

        private_nh.param("collision_check_lookahead_points", collision_check_lookahead_points_, 10);
        private_nh.param("visualization_scale_factor", visualization_scale_factor_, 5);
        private_nh.param("visualize_costmap", visualize_costmap_, false);
        //------------------------------------用于动态速度控制的参数-----------------------------

        private_nh.param("a", a_, 7.0);
        private_nh.param("k", k_, -25.0); 
        initial_rotation_done_ = false;
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
        // std::cout << "Press [Enter] to continue...";
        // std::cin.ignore(); // 清除缓冲区
        // std::cin.get();    // 等待回车
        // ROS_INFO("查看全局路径规划");
        // ROS_INFO("起点%f,%f",global_plan_[0].pose.position.x,global_plan_[0].pose.position.y);
        // geometry_msgs::PoseStamped pose;
        // tf::StampedTransform transform;
        // try {
        //     tf_listener_->lookupTransform("map", "base_link", ros::Time(0), transform);
        //     pose.header.frame_id = "map";
        //     pose.header.stamp = ros::Time::now();
        //     pose.pose.position.x = transform.getOrigin().x();
        //     pose.pose.position.y = transform.getOrigin().y();
        //     pose.pose.orientation = tf::createQuaternionMsgFromYaw(
        //         tf::getYaw(transform.getRotation()));
        // } catch (tf::TransformException &ex) {
        //     ROS_ERROR("获取机器人位姿失败: %s", ex.what());
        // }
        // ROS_INFO("实际机器人位置: (%.3f, %.3f)", pose.pose.position.x,pose.pose.position.y);
        // size_t number = global_plan_.size();
        // for(int i=0;i<20;i++){
        //     ROS_INFO("全局规划特殊点x%f,y%f",global_plan_[number*i/20].pose.position.x,global_plan_[number*i/20].pose.position.y);
        // }
        // ROS_INFO("目的地%f,%f,%f",global_plan_[global_plan_.size()-1].pose.position.x,global_plan_[global_plan_.size()-1].pose.position.y,tf2::getYaw(global_plan_[global_plan_.size()-1].pose.orientation));
        return true;
    }

    bool MyPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        //可以通过planner_frequency定期重新全局规划实现动态避障，这部分就不需要了
        // 获取代价地图的数据
        costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
        unsigned char* map_data = costmap->getCharMap();
        unsigned int size_x = costmap->getSizeInCellsX();
        unsigned int size_y = costmap->getSizeInCellsY();

        // 使用 OpenCV 绘制代价地图
        cv::Mat map_image;
        
        map_image.create(size_y, size_x, CV_8UC3);
        for (unsigned int y = 0; y < size_y; y++)
        {
            for (unsigned int x = 0; x < size_x; x++)
            {
                int map_index = y * size_x + x;
                unsigned char cost = map_data[map_index];               // 从代价地图数据取值
                cv::Vec3b& pixel = map_image.at<cv::Vec3b>(map_index);  // 获取彩图对应像素地址
            
                if (cost == 0)          // 可通行区域
                    pixel = cv::Vec3b(128, 128, 128); // 灰色
                else if (cost == 254)   // 障碍物
                    pixel = cv::Vec3b(0, 0, 0);       // 黑色
                else if (cost == 253)   // 禁行区域 
                    pixel = cv::Vec3b(255, 255, 0);   // 浅蓝色
                else
                {
                    // 根据灰度值显示从红色到蓝色的渐变
                    unsigned char blue = 255 - cost;
                    unsigned char red = cost;
                    pixel = cv::Vec3b(blue, 0, red);
                }
            }
        }
        

        // 在代价地图上遍历导航路径点
        cv::Mat flipped_image(size_x, size_y, CV_8UC3, cv::Scalar(128, 128, 128));
        for(int i=0;i<global_plan_.size();i++)
        {
            geometry_msgs::PoseStamped pose_in_map;
            global_plan_[i].header.stamp = ros::Time(0);
            tf_listener_->transformPose("map",global_plan_[i],pose_in_map);
            double map_x = pose_in_map.pose.position.x;
            double map_y = pose_in_map.pose.position.y;

            double origin_x = costmap->getOriginX();
            double origin_y = costmap->getOriginY();
            double local_x = map_x - origin_x;
            double local_y = map_y - origin_y;
            int x = local_x / costmap->getResolution();
            int y = local_y / costmap->getResolution();
            cv::circle(map_image, cv::Point(x,y), 0, cv::Scalar(255,0,255));    // 导航路径点

            // 检测前方路径点是否在禁行区域或者障碍物里
            if(i >= target_index_ && i < target_index_ + collision_check_lookahead_points_)
            {
                cv::circle(map_image, cv::Point(x,y), 0, cv::Scalar(0,255,255));// 检测路径点
                int map_index = y * size_x + x;
                unsigned char cost = map_data[map_index];
                if(cost >= 253)
                {
                    ROS_INFO("重新规划路径");
                    initial_rotation_done_ = false;//路径结束判断：遇到障碍物或者完成最终姿态调整
                    return false;//重新规划全局路径，实现动态避障的效果
                }
            }
        }

        map_image.at<cv::Vec3b>(size_y/2, size_x/2) = cv::Vec3b(0, 255, 0); // 机器人位置
        
        
        
        // 翻转地图
        flipped_image = cv::Mat(size_x, size_y, CV_8UC3, cv::Scalar(128, 128, 128));
        cv::flip(map_image, map_image, 0);//使用cvflip代替原本的for循环，运行速度更快
        if(visualize_costmap_)
        {
            // 显示代价地图
            cv::namedWindow("Map");
            cv::resize(map_image, map_image, cv::Size(size_y*5, size_x*5), 0, 0, cv::INTER_NEAREST);
            cv::resizeWindow("Map", size_y*5, size_x*5);
            cv::imshow("Map", map_image);
        }
        
        


        if (global_plan_.empty()) {
            ROS_ERROR("MyPlanner: Received an empty global plan. Cannot compute velocity commands.");
            return false; // 返回 false 表示规划失败，这可能会触发 move_base 的恢复行为
        }



        // ROS_INFO("进入代码确认");
        int final_index = global_plan_.size()-1;
        geometry_msgs::PoseStamped pose_final;
        // global_plan_[final_index].header.stamp = ros::Time(0);
        // tf_listener_->transformPose("base_link",global_plan_[final_index],pose_final);
        
        global_plan_[final_index].header.stamp = ros::Time(0);
        tf_listener_->transformPose("base_link",global_plan_[final_index],pose_final);



        if(pose_adjusting_ == false)//如果未进入姿态调整状态
        {
            double dx = pose_final.pose.position.x;
            double dy = pose_final.pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            // ROS_INFO("终点距离%f",dist);
            if(dist < goal_dist_threshold_)//判定是否到达目标点附近的距离阈值
                pose_adjusting_ = true;
        }
        if(pose_adjusting_ == true)
        {
            double final_yaw = tf::getYaw(pose_final.pose.orientation);
            // double final_yaw = tf2::getYaw(pose_final.pose.orientation);

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
        global_plan_[target_index_].header.stamp = ros::Time(0);
        tf_listener_->transformPose("base_link",global_plan_[target_index_],target_pose);
        double dxt = target_pose.pose.position.x;
        double dyt = target_pose.pose.position.y;
        double distt = std::sqrt(dxt*dxt + dyt*dyt);
        if(distt<0.7)
        {//小于10cm才会触发重新规划
            for(int i=target_index_;i<global_plan_.size();i++)
            {
                geometry_msgs::PoseStamped pose_base;
                global_plan_[i].header.stamp = ros::Time(0);
                tf_listener_->transformPose("base_link",global_plan_[i],pose_base);
                // ROS_WARN("检查地图坐标x%f,y%f",global_plan_[i].pose.position.x,global_plan_[i].pose.position.y);
                // ROS_INFO("检查baselink坐标x%f,y%f",pose_base.pose.position.x,pose_base.pose.position.y);


                double dx = pose_base.pose.position.x;
                double dy = pose_base.pose.position.y;
                double dist = std::sqrt(dx*dx + dy*dy);

                if (dist > lookahead_dist_) //选取的临时目标点的距离阈值
                {
                    target_pose = pose_base;
                    target_index_ = i;
                    // ROS_WARN("选择第 %d 个路径点作为临时目标，坐标x%f,y%f",target_index_,global_plan_[i].pose.position.x,global_plan_[i].pose.position.y);
                    break;
                }

                if(i == global_plan_.size()-1){//  如果最远的点也没超出阈值，就取最远的点
                    target_pose = pose_base; 
                }
            }
        }

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
        
        double avrage_curvature = 0.0001;//平均曲率
        double dynamic_x_gain = path_linear_x_gain_; // 默认使用参数文件中的基础x增益

        // 在 [0, target_index_] 区间内计算最大曲率
        // 需要至少3个点(i-1, i, i+1)来计算曲率，所以 target_index_ 必须至少为2
        int cur_final = std::min(target_index_+15,final_index);//多看10cm
        int cur_start = std::max(target_index_ - 15 , 0 );
        if (target_index_ >= 2) 
        {
            // 循环遍历从路径起点到 target_index 的前一个点
            // 计算点 i 处的曲率，需要 p_i-1, p_i, p_i+1
            for (int i = cur_start; i < cur_final; ++i)
            {
                const auto& p0 = global_plan_[i-1].pose.position;
                const auto& p1 = global_plan_[i].pose.position;
                const auto& p2 = global_plan_[i+1].pose.position;
                
                // 计算向量 u (p0->p1) 和 v (p1->p2)
                double ux = p1.x - p0.x, uy = p1.y - p0.y;
                double vx = p2.x - p1.x, vy = p2.y - p1.y;

                double norm_u = std::sqrt(ux*ux + uy*uy);
                double norm_v = std::sqrt(vx*vx + vy*vy);
                double w_norm = std::sqrt(std::pow(p2.x - p0.x, 2) + std::pow(p2.y - p0.y, 2));

                // 使用Menger曲率公式: K = 2 * |cross(u, v)| / (|u| * |v| * |u+v|)
                double cross_product_mag = std::abs(ux * vy - uy * vx);

                if (norm_u > 1e-6 && norm_v > 1e-6 && w_norm > 1e-6) {
                    double curvature = (2.0 * cross_product_mag) / (norm_u * norm_v * w_norm);
                    // if (curvature > max_curvature) {
                    //     max_curvature = curvature;
                    // }
                    avrage_curvature += curvature*(1.5-i/cur_final);//曲率加平均，近处权重更大
                }
            }
        }
        avrage_curvature = avrage_curvature/(cur_final - cur_start );
        // 根据最大曲率调整速度增益
        // if (max_curvature > 10.0) // 设置一个阈值，避免在近似直线上过度反应，曲率超过阈值才会减速
        // {
        //     //速度增益只与该路径段的最大曲率有关
        //     double penalty_term = max_curvature * curvature_penalty_gain_;
        //     dynamic_x_gain = path_linear_x_gain_ / (curvature_damping_factor_ + penalty_term);
        // }
        // ROS_INFO("基础速度%f",path_linear_x_gain_);
        dynamic_x_gain = path_linear_x_gain_  * exp((avrage_curvature-a_)/k_);

        // 对动态增益进行限幅，保证安全和稳定
        dynamic_x_gain = std::max(2.0, dynamic_x_gain); // 最小增益，防止失速
        // dynamic_x_gain = std::min(dynamic_x_gain, path_linear_x_gain_); // 最大增益，防止飙车不需要了，指数本来就限幅
        // ROS_INFO("当前dynamic_x_gain=%f",dynamic_x_gain);
        // ROS_INFO("当前平均curvature=%f",avrage_curvature);



        cmd_vel.linear.x = target_pose.pose.position.x * dynamic_x_gain;//小车运动速度比例系数
        cmd_vel.linear.y = target_pose.pose.position.y * path_linear_y_gain_;
        if(avrage_curvature < 5)
        {
            cmd_vel.angular.z = target_pose.pose.position.y * path_angular_gain_*(angular_limit_ * avrage_curvature+0.6);   //限制角速度，防止前进时超调摆头
        }
        else
        {
            cmd_vel.angular.z = target_pose.pose.position.y * path_angular_gain_;
        }
        // ROS_INFO("速度:x%f,y%f,z%f",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.angular.z);
    
        //--------------------------全局路径显示，省去节省算力---------------------------------------------
        // cv::Mat plan_image(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));        
        // for(int i=0;i<global_plan_.size();i++)
        // {
        //     geometry_msgs::PoseStamped pose_base;
        //     global_plan_[i].header.stamp = ros::Time(0);
        //     tf_listener_->transformPose("base_link",global_plan_[i],pose_base);
        //     int cv_x = 300 - pose_base.pose.position.y*100;
        //     int cv_y = 300 - pose_base.pose.position.x*100;
        //     cv::circle(plan_image, cv::Point(cv_x,cv_y), 1, cv::Scalar(255,0,255)); 
        // }
        // cv::circle(plan_image, cv::Point(300, 300), 15, cv::Scalar(0, 255, 0));
        // cv::line(plan_image, cv::Point(65, 300), cv::Point(510, 300), cv::Scalar(0, 255, 0), 1);
        // cv::line(plan_image, cv::Point(300, 45), cv::Point(300, 555), cv::Scalar(0, 255, 0), 1);

        // // cv::namedWindow("Plan");
        // // cv::imshow("Plan", plan_image);
        // cv::waitKey(1);
        
        return true;
    }

    bool MyPlanner::isGoalReached()
    {
        return goal_reached_;
    }
}
 // namespace my_planner