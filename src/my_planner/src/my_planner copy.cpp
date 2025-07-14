#include "my_planner.h"
#include <pluginlib/class_list_macros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
//为了处理tf2异常，包含下面的头文件
#include <tf2_ros/transform_listener.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


// PLUGINLIB_EXPORT_CLASS( my_planner::MyPlanner, nav_core::BaseLocalPlanner)

// namespace my_planner 
// {
//     MyPlanner::MyPlanner()
//     {
//         setlocale(LC_ALL,"");
//     }
//     MyPlanner::~MyPlanner()
//     {}

//     // 全局变量的定义移动到了类的声明中，这里不再需要
//     // tf::TransformListener* tf_listener_;
//     // costmap_2d::Costmap2DROS* costmap_ros_;
//     void MyPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
//     {
//         ROS_WARN("本地规划器，启动！");
//         //不再创建新的 listener，而是直接使用传入的 tf buffer
//         // tf_listener_ = new tf::TransformListener(); // 移除此行，解决内存泄漏
//         tf_buffer_ = tf; //添加此行
//         costmap_ros_ = costmap_ros;
//     }

//     // 全局变量的定义移动到了类的声明中，这里不再需要
//     // std::vector<geometry_msgs::PoseStamped> global_plan_;
//     // int target_index_;
//     // bool pose_adjusting_;
//     // bool goal_reached_;
//     bool MyPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
//     {
//         target_index_ = 0;
//         global_plan_ = plan;
//         pose_adjusting_ = false;
//         goal_reached_ = false;
//         return true;
//     }

//     bool MyPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
//     {
//         // 获取代价地图的数据
//         costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
//         unsigned char* map_data = costmap->getCharMap();
//         unsigned int size_x = costmap->getSizeInCellsX();
//         unsigned int size_y = costmap->getSizeInCellsY();

//         // 使用 OpenCV 绘制代价地图
//         cv::Mat map_image(size_y, size_x, CV_8UC3, cv::Scalar(128, 128, 128));
//         for (unsigned int y = 0; y < size_y; y++)
//         {
//             for (unsigned int x = 0; x < size_x; x++)
//             {
//                 int map_index = y * size_x + x;
//                 unsigned char cost = map_data[map_index];               // 从代价地图数据取值
//                 cv::Vec3b& pixel = map_image.at<cv::Vec3b>(map_index);  // 获取彩图对应像素地址
                
//                 if (cost == 0)          // 可通行区域
//                     pixel = cv::Vec3b(128, 128, 128); // 灰色
//                 else if (cost == 254)   // 障碍物
//                     pixel = cv::Vec3b(0, 0, 0);       // 黑色
//                 else if (cost == 253)   // 禁行区域 
//                     pixel = cv::Vec3b(255, 255, 0);   // 浅蓝色
//                 else
//                 {
//                     // 根据灰度值显示从红色到蓝色的渐变
//                     unsigned char blue = 255 - cost;
//                     unsigned char red = cost;
//                     pixel = cv::Vec3b(blue, 0, red);
//                 }
//             }
//         }

// 	//使用 try-catch 结构来安全地调用坐标变换
//         try
//         {
//             // 在代价地图上遍历导航路径点
//             for(int i=0; i < global_plan_.size(); i++)
//             {
//                 geometry_msgs::PoseStamped pose_in_map;
//                 //使用 tf2 的接口进行坐标变换
//                 tf_buffer_->transform(global_plan_[i], pose_in_map, "map", ros::Duration(0.1));
                
//                 double map_x = pose_in_map.pose.position.x;
//                 double map_y = pose_in_map.pose.position.y;

//                 double origin_x = costmap->getOriginX();
//                 double origin_y = costmap->getOriginY();
//                 double local_x = map_x - origin_x;
//                 double local_y = map_y - origin_y;
//                 int x = local_x / costmap->getResolution();
//                 int y = local_y / costmap->getResolution();
//                 cv::circle(map_image, cv::Point(x,y), 0, cv::Scalar(255,0,255));

//                 if(i >= target_index_ && i < target_index_ + 10)
//                 {
//                     cv::circle(map_image, cv::Point(x,y), 0, cv::Scalar(0,255,255));
//                     int map_index = y * size_x + x;
//                     unsigned char cost = map_data[map_index];
//                     if(cost >= 253)
//                         return false;
//                 }
//             }
            
//             map_image.at<cv::Vec3b>(size_y/2, size_x/2) = cv::Vec3b(0, 255, 0);
//             cv::flip(map_image, map_image, 0); // 使用更高效的cv::flip进行翻转
//             cv::namedWindow("Map");
//             cv::imshow("Map", map_image);

//             int final_index = global_plan_.size()-1;
//             geometry_msgs::PoseStamped pose_final;
//             //使用 tf2 的接口进行坐标变换
//             tf_buffer_->transform(global_plan_[final_index], pose_final, "base_link", ros::Duration(0.1));

//             if(pose_adjusting_ == false)
//             {
//                 double dx = pose_final.pose.position.x;
//                 double dy = pose_final.pose.position.y;
//                 double dist = std::sqrt(dx*dx + dy*dy);
//                 if(dist < 0.10)
//                     pose_adjusting_ = true;
//             }

//             if(pose_adjusting_ == true)
//             {
//                 double final_yaw = tf::getYaw(pose_final.pose.orientation);
//                 ROS_WARN("调整最终姿态，final_yaw = %.2f",final_yaw);
//                 cmd_vel.linear.x = pose_final.pose.position.x * 2.5;
//                 cmd_vel.angular.z = final_yaw * 1.5;
//                 if(abs(final_yaw) < 0.1)
//                 {
//                     goal_reached_ = true;
//                     ROS_WARN("到达终点！");
//                     cmd_vel.linear.x = 0;
//                     cmd_vel.angular.z = 0;
//                 }
//                 return true;
//             }

            geometry_msgs::PoseStamped target_pose;//计算前方40cm最大曲率，控制前进速度
            double ux = global_plan_[1].pose.position.x-global_plan_[0].pose.position.x;
            double uy = global_plan_[1].pose.position.y-global_plan_[0].pose.position.y;
            double last_norm = std::sqrt(ux*ux + uy*uy);//第一个点要提前算
            double xspeed = 2.0;//给个默认值以防万一
            for(int i = target_index_; i < global_plan_.size(); i++)
            {
                // 存储最大曲率信息
                double max_curvature = 0.0;
                int max_curvature_index = -1;

                geometry_msgs::PoseStamped pose_base;
                //使用 tf2 的接口进行坐标变换
                tf_buffer_->transform(global_plan_[i], pose_base, "base_link", ros::Duration(0.1));
                bool flag = 1;//由于计算曲率和找目标点用到的循环不同，所以加个标志
                
                double dx = pose_base.pose.position.x;
                double dy = pose_base.pose.position.y;
                double dist = std::sqrt(dx*dx + dy*dy);

                if(i!=0&&i!=global_plan_.size()){
                    double x0 = global_plan_[i-1].pose.position.x;
                    double y0 = global_plan_[i-1].pose.position.y;
                    double x1 = global_plan_[i].pose.position.x;
                    double x1 = global_plan_[i].pose.position.y;
                    double x2 = global_plan_[i+1].pose.position.x;
                    double y2 = global_plan_[i+1].pose.position.y;
                    // 计算相邻两点向量
                    // double ux = x1 - x0, uy = y1 - y0上一个点的next就是这个点的last
                    double vx = x2 - x1, vy = y2 - y1;
                    double cross = std::abs(ux * vy - uy * vx);
                    double norm_u = std::sqrt(ux*ux + uy*uy);
                    double norm_v = std::sqrt(vx*vx + vy*vy);
                    double norm_uv = std::sqrt((ux+vx)*(ux+vx) + (uy+vy)*(uy+vy));
                    // 避免除以零
                    if(norm_u > 1e-5 && norm_v > 1e-5 && norm_uv > 1e-5) {
                        // 计算曲率绝对值
                        double curvature = (2.0 * cross) / (norm_u * norm_v * norm_uv);
                        // 更新最大曲率
                        if(curvature > max_curvature) {
                            max_curvature = curvature;
                            max_curvature_index = i-target_index_;//说明最大曲率点的距离，用来平滑减速，免得有一个大曲率点就急刹车
                        }
                    }else {
                        ROS_DEBUG("曲率过大,不予理会");
                    }
                }
                if(dist>0.4){
                    break;//计算前方40cm曲率
                }
                ROS_INFO("前方40cm最大曲率%f",max_curvature);
                xspeed = 4.0/(0.5+max_curvature/max_curvature_index*4);//曲率越大速度越慢,距离越远影响越小

                if (flag && dist > 0.2)
                {
                    target_pose = pose_base;
                    target_index_ = i;
                    ROS_WARN("选择第 %d 个路径点作为临时目标，距离=%.2f",target_index_,dist);
                    flag = 0;
                }
                if(i == global_plan_.size()-1)
                    target_pose = pose_base; 
            }
            cmd_vel.linear.x = target_pose.pose.position.x * xspeed;
            cmd_vel.angular.z = target_pose.pose.position.y * 6.8;

//             cv::Mat plan_image(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));        
//             for(int i=0;i<global_plan_.size();i++)
//             {
//             	geometry_msgs::PoseStamped pose_base;
//             	global_plan_[i].header.stamp = ros::Time(0);
//             	tf_buffer_->transformPose("base_link",global_plan_[i],pose_base);
//             	int cv_x = 300 - pose_base.pose.position.y*100;
//             	int cv_y = 300 - pose_base.pose.position.x*100;
//             	cv::circle(plan_image, cv::Point(cv_x,cv_y), 1, cv::Scalar(255,0,255)); 
//         	}
//         	cv::circle(plan_image, cv::Point(300, 300), 15, cv::Scalar(0, 255, 0));
//         	cv::line(plan_image, cv::Point(65, 300), cv::Point(510, 300), cv::Scalar(0, 255, 0), 1);
//         	cv::line(plan_image, cv::Point(300, 45), cv::Point(300, 555), cv::Scalar(0, 255, 0), 1);
	
//         	// cv::namedWindow("Plan");
//         	// cv::imshow("Plan", plan_image);
//             	cv::waitKey(1);
//             }
//         catch (tf2::TransformException &ex)
//         {
//             //捕获并处理异常
//             ROS_WARN("坐标变换失败: %s", ex.what());
//             cmd_vel.linear.x = 0;
//             cmd_vel.angular.z = 0;
//            return false; // 返回 false，让 move_base 知道本地规划失败
//         }
        
//         return true;
//     }
//     bool MyPlanner::isGoalReached()
//     {
//         return goal_reached_;
//     }
// } // namespace my_planner