#include "my_planner.h"
#include <pluginlib/class_list_macros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

PLUGINLIB_EXPORT_CLASS( my_planner::MyPlanner, nav_core::BaseLocalPlanner)

namespace my_planner 
{
    MyPlanner::MyPlanner()
    {
        setlocale(LC_ALL,"");
    }
    MyPlanner::~MyPlanner()
    {}

    tf::TransformListener* tf_listener_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    void MyPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_WARN("本地规划器，启动！");
        tf_listener_ = new tf::TransformListener();
        costmap_ros_ = costmap_ros;
    }

    std::vector<geometry_msgs::PoseStamped> global_plan_;
    int target_index_;
    bool pose_adjusting_;
    bool goal_reached_; 
    bool MyPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        target_index_ = 0;
        global_plan_ = plan;
        pose_adjusting_ = false;
        goal_reached_ = false;
        return true;
    }

    bool MyPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        // 获取代价地图的数据
        costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
        unsigned char* map_data = costmap->getCharMap();
        unsigned int size_x = costmap->getSizeInCellsX();
        unsigned int size_y = costmap->getSizeInCellsY();

        // 使用 OpenCV 绘制代价地图
        cv::Mat map_image(size_y, size_x, CV_8UC3, cv::Scalar(128, 128, 128));
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
            if(i >= target_index_ && i < target_index_ + 10)
            {
                cv::circle(map_image, cv::Point(x,y), 0, cv::Scalar(0,255,255));// 检测路径点
                int map_index = y * size_x + x;
                unsigned char cost = map_data[map_index];
                if(cost >= 253)
                    return false;
            }
        }

        map_image.at<cv::Vec3b>(size_y/2, size_x/2) = cv::Vec3b(0, 255, 0); // 机器人位置

        // 翻转地图
        cv::Mat flipped_image(size_x, size_y, CV_8UC3, cv::Scalar(128, 128, 128));
        for (unsigned int y = 0; y < size_y; ++y)
        {
            for (unsigned int x = 0; x < size_x; ++x)
            {
                cv::Vec3b& pixel = map_image.at<cv::Vec3b>(y, x);
                flipped_image.at<cv::Vec3b>((size_x - 1 - x), (size_y - 1 - y)) = pixel;
            }
        }
        map_image = flipped_image;

        // 显示代价地图
        cv::namedWindow("Map");
        cv::resize(map_image, map_image, cv::Size(size_y*5, size_x*5), 0, 0, cv::INTER_NEAREST);
        cv::resizeWindow("Map", size_y*5, size_x*5);
        cv::imshow("Map", map_image);

        int final_index = global_plan_.size()-1;
        geometry_msgs::PoseStamped pose_final;
        global_plan_[final_index].header.stamp = ros::Time(0);
        tf_listener_->transformPose("base_link",global_plan_[final_index],pose_final);
        if(pose_adjusting_ == false)//如果未进入姿态调整状态
        {
            double dx = pose_final.pose.position.x;
            double dy = pose_final.pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            if(dist < 0.10)//判定是否到达目标点附近的距离阈值
                pose_adjusting_ = true;
        }
        if(pose_adjusting_ == true)
        {
            double final_yaw = tf::getYaw(pose_final.pose.orientation);
            ROS_WARN("调整最终姿态，final_yaw = %.2f",final_yaw);
            cmd_vel.linear.x = pose_final.pose.position.x * 2.5;//到达目标点附近后调整位姿的速度比例系数
            cmd_vel.angular.z = final_yaw * 1.5;
            if(abs(final_yaw) < 0.1)
            {
                goal_reached_ = true;
                ROS_WARN("到达终点！");
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
            }
            return true;
        }

        geometry_msgs::PoseStamped target_pose;
        for(int i=target_index_;i<global_plan_.size();i++)
        {
            geometry_msgs::PoseStamped pose_base;
            global_plan_[i].header.stamp = ros::Time(0);
            tf_listener_->transformPose("base_link",global_plan_[i],pose_base);
            double dx = pose_base.pose.position.x;
            double dy = pose_base.pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);

            if (dist > 0.2) //选取的临时目标点的距离阈值
            {
                target_pose = pose_base;
                target_index_ = i;
                ROS_WARN("选择第 %d 个路径点作为临时目标，距离=%.2f",target_index_,dist);
                break;
            }

            if(i == global_plan_.size()-1)
                target_pose = pose_base; 
        }
        cmd_vel.linear.x = target_pose.pose.position.x * 2.0;//小车运动速度比例系数
        cmd_vel.angular.z = target_pose.pose.position.y * 6.8;

        cv::Mat plan_image(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));        
        for(int i=0;i<global_plan_.size();i++)
        {
            geometry_msgs::PoseStamped pose_base;
            global_plan_[i].header.stamp = ros::Time(0);
            tf_listener_->transformPose("base_link",global_plan_[i],pose_base);
            int cv_x = 300 - pose_base.pose.position.y*100;
            int cv_y = 300 - pose_base.pose.position.x*100;
            cv::circle(plan_image, cv::Point(cv_x,cv_y), 1, cv::Scalar(255,0,255)); 
        }
        cv::circle(plan_image, cv::Point(300, 300), 15, cv::Scalar(0, 255, 0));
        cv::line(plan_image, cv::Point(65, 300), cv::Point(510, 300), cv::Scalar(0, 255, 0), 1);
        cv::line(plan_image, cv::Point(300, 45), cv::Point(300, 555), cv::Scalar(0, 255, 0), 1);

        // cv::namedWindow("Plan");
        // cv::imshow("Plan", plan_image);
        cv::waitKey(1);
        
        return true;
    }
    bool MyPlanner::isGoalReached()
    {
        return goal_reached_;
    }
} // namespace my_planner