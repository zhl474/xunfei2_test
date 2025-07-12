#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

void rviz_Callback(const nav_msgs::Odometry::ConstPtr& odo)
{
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "map";
    static_transformStamped.child_frame_id = "base_link";
    static_transformStamped.transform.translation.x=1.0;
    static_transformStamped.transform.translation.y=1.0;
    static_transformStamped.transform.translation.z=odo->pose.pose.position.z;
    static_transformStamped.transform.rotation.w = odo->pose.pose.orientation.w;
    static_transformStamped.transform.rotation.x = odo->pose.pose.orientation.x;
    static_transformStamped.transform.rotation.y = odo->pose.pose.orientation.y;
    static_transformStamped.transform.rotation.z = odo->pose.pose.orientation.z;
    static_broadcaster.sendTransform(static_transformStamped);

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "pub_model_state");
    ros::NodeHandle nh;
    ros::Subscriber uwbSub = nh.subscribe<nav_msgs::Odometry>("topicname", 1000, rviz_Callback);
    ROS_INFO("RUNNING");

    ros::spin();

    return 0;
}

