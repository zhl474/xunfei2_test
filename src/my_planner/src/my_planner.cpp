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
    void MyPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_WARN("本地规划器，启动！");
        tf_listener_ = new tf::TransformListener();
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
        int final_index = global_plan_.size()-1;
        geometry_msgs::PoseStamped pose_final;
        global_plan_[final_index].header.stamp = ros::Time(0);
        tf_listener_->transformPose("base_link",global_plan_[final_index],pose_final);
        if(pose_adjusting_ == false)
        {
            double dx = pose_final.pose.position.x;
            double dy = pose_final.pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            if(dist < 0.1)//判定是否到达目标点附近的距离阈值
                pose_adjusting_ = true;
        }
        if(pose_adjusting_ == true)
        {
            double final_yaw = tf::getYaw(pose_final.pose.orientation);
            ROS_WARN("调整最终姿态,final_yaw = %.2f",final_yaw);
            cmd_vel.linear.x = pose_final.pose.position.x * 3.5;//到达目标点附近后调整位姿的速度比例系数
            cmd_vel.angular.z = final_yaw * 1.5;
            if(abs(final_yaw) < 0.1)
            {
                goal_reached_ = true;
                ROS_WARN("到达终点！");
                cmd_vel.linear.x = 0;//刹车
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
        cmd_vel.linear.x = target_pose.pose.position.x * 5;//小车运动速度比例系数
        cmd_vel.angular.z = target_pose.pose.position.y * 15.0;

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

        // cv::namedWindow("Plan");
        // cv::imshow("Plan", plan_image);
        // cv::waitKey(1);
        
        return true;
    }
    bool MyPlanner::isGoalReached()
    {
        return goal_reached_;
    }
} // namespace my_planner
