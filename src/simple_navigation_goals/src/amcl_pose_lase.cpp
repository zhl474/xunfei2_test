#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class PoseTransformer {
public:
    PoseTransformer() {
        // 订阅 /amcl_pose 话题
        pose_sub_ = nh_.subscribe("/amcl_pose", 10, &PoseTransformer::amclPoseCallback, this);
    }

    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        // 获取机器人在世界坐标系中的位姿
        geometry_msgs::Pose robot_pose = msg->pose.pose;

        // 创建一个TF监听器
        tf::TransformListener listener;
        
        try {
            // 等待静态变换，等待时间可以根据需要调整
            listener.waitForTransform("/base_link", "/laser_frame", ros::Time(0), ros::Duration(4.0));
            
            // 获取雷达相对于机器人的变换
            tf::StampedTransform laser_transform;
            listener.lookupTransform("/base_link", "/laser_frame", ros::Time(0), laser_transform);
            
            // 将机器人位姿和雷达相对于机器人的变换结合起来
            geometry_msgs::Pose laser_pose_world;
            laser_pose_world.position.x = robot_pose.position.x + laser_transform.getOrigin().x();
            laser_pose_world.position.y = robot_pose.position.y + laser_transform.getOrigin().y();
            laser_pose_world.position.z = robot_pose.position.z + laser_transform.getOrigin().z();
            
            // 旋转部分处理（使用tf变换工具）
            tf::Quaternion q_robot(robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w);
            tf::Quaternion q_laser(laser_transform.getRotation().x(), laser_transform.getRotation().y(), laser_transform.getRotation().z(), laser_transform.getRotation().w());
            tf::Quaternion q_combined = q_robot * q_laser;
            
            laser_pose_world.orientation.x = q_combined.x();
            laser_pose_world.orientation.y = q_combined.y();
            laser_pose_world.orientation.z = q_combined.z();
            laser_pose_world.orientation.w = q_combined.w();
            
            ROS_INFO("Laser pose in world frame: (%f, %f, %f)", laser_pose_world.position.x, laser_pose_world.position.y, laser_pose_world.position.z);
            ROS_INFO("Laser pose in world frame: (%f, %f, %f,%f)", laser_pose_world.orientation.x, laser_pose_world.orientation.y, laser_pose_world.orientation.z,laser_pose_world.orientation.w);
            
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_pose_in_world_frame");
    PoseTransformer pt;
    ros::spin();
    return 0;
}
