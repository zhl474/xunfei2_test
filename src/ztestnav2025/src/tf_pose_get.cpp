#include "ros/ros.h"
#include "ros_nanodet/detect_result_srv.h"
#include "ztestnav2025/getpose_server.h"

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <boost/algorithm/string/join.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

#include "dynamic_reconfigure/server.h"
#include "ztestnav2025/drConfig.h"

//-------------------------调试的时候把43行和80行代码map改成odom就行-------------//

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class TF_position_get {
public:
    TF_position_get (ros::NodeHandle& nh) : 
        nh_(nh),
        tf_buffer_(),
        tf_listener_(tf_buffer_),
        getpose(nh_.advertiseService("getpose_server", &TF_position_get::getpose_server, this))
    {}

    bool waitForTransform() {//等待TF初始化完成
        ros::Time start_time = ros::Time::now();
        // 设置超时时间（例如10秒）
        ros::Duration timeout(10.0);
        
        while (ros::ok() && (ros::Time::now() - start_time < timeout)) {
            if (tf_buffer_.canTransform("map", "base_link", ros::Time(0))) {//这里map改odom即可调试
                return true;
            }
            ROS_WARN_THROTTLE(1, "等待坐标系变换中...");
            ros::Duration(0.1).sleep();  // 降低CPU占用
        }
        ROS_ERROR("等待超时:map到base_link的坐标变换未就绪");
        return false;
    }

    // 坐标变换查询（参考网页8的异常处理）
    bool getpose_server(ztestnav2025::getpose_server::Request &req,ztestnav2025::getpose_server::Response &resp) {
        auto pose = getCurrentPose();
        if (!pose) ROS_INFO("坐标获取失败");
        auto pose1 = *pose;
        double yaw = getYawFromTransform(pose1);
        resp.pose_at.resize(3);
        resp.pose_at[0] = pose1.transform.translation.x;
        resp.pose_at[1] = pose1.transform.translation.y;
        resp.pose_at[2] = yaw;
        return true;
    }


private:
    // TF坐标变换核心对象（参考网页6的TF生命周期管理）
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // ROS通信接口
    ros::NodeHandle nh_;
    ros::ServiceServer getpose;

    // 坐标变换查询（参考网页8的异常处理）
    boost::optional<geometry_msgs::TransformStamped> getCurrentPose() {
        try {
            waitForTransform();
            return tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));//这也要改
        } catch (tf2::TransformException &ex) {
            ROS_WARN("TF查询失败: %s", ex.what());
            if (!tf_buffer_._frameExists("map")) {
                ROS_ERROR("map坐标系未发布");
            } else if (!tf_buffer_._frameExists("base_link")) {
                ROS_ERROR("base_link坐标系未发布");
            }
            return boost::none;
        }
    }

    // 四元数转欧拉角（参考网页7的转换方法）
    double getYawFromTransform(const geometry_msgs::TransformStamped& tf) {
        return tf2::getYaw(tf.transform.rotation);
    }
};

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"tf_pose_get");
    ros::NodeHandle nh;
    TF_position_get tf_position_get(nh);

    ROS_INFO("tf坐标获取初始化完成");
    ros::spin();
}