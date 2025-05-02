#include "ros/ros.h"
#include "ros_nanodet/detect_result_srv.h"

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MecanumController {
public:
    MecanumController(ros::NodeHandle& nh) : 
        nh_(nh),
        tf_buffer_(),
        tf_listener_(tf_buffer_),
        cmd_pub_(nh.advertise<geometry_msgs::Twist>("cmd_vel", 10)) 
    {}

    void rotateFullCircle(double rotate) {//rotate是弧度
        geometry_msgs::Twist twist;
        ros::Rate rate(20);     // 控制频率20Hz
        auto start = getCurrentPose();
        if (!start) ROS_INFO("坐标获取失败");
        double target = getYawFromTransform(*start)+rotate;
        while (ros::ok()) {
            // 获取当前姿态（参考网页7的TF查询方法）
            auto now_yaw = getCurrentPose();
            if (!now_yaw) continue;

            // 计算旋转控制量（参考网页2的PWM控制原理）
            double yaw = getYawFromTransform(*now_yaw);
            if (fabs(yaw - target) <= angle_error_) {
                break;
            }

            // 发送运动指令（参考网页1的速度发布逻辑）
            twist.angular.z = angular_speed_;
            cmd_pub_.publish(twist);
            rate.sleep();
        }
    }

private:
    // TF坐标变换核心对象（参考网页6的TF生命周期管理）
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // ROS通信接口
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    
    // 运动学参数
    double angular_speed_ = 0.4;   // 默认角速度(rad/s)
    double angle_error_ = 0.087;   //容忍5度偏差

    // 坐标变换查询（参考网页8的异常处理）
    boost::optional<geometry_msgs::TransformStamped> getCurrentPose() {
        try {
            return tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("TF查询失败: %s", ex.what());
            return boost::none;
        }
    }

    // 四元数转欧拉角（参考网页7的转换方法）
    double getYawFromTransform(const geometry_msgs::TransformStamped& tf) {
        // tf2::Quaternion quat;
        // tf2::fromMsg(tf.transform.rotation, quat);
        // tf2::Matrix3x3 mat(quat);
        // double roll, pitch, yaw;
        // mat.getRPY(roll, pitch, yaw);
        // return yaw;
        return tf2::getYaw(tf.transform.rotation);
    }

    // 用户自定义操作接口
    void executeCustomOperation(double current_angle) {
        ROS_INFO("触发操作：当前角度 %.2f rad (%.1f°)", 
                current_angle, current_angle*180/M_PI);
        // 此处添加用户自定义逻辑（如数据采集、传感器触发等）
    }
};

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"zhltest");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Rate rate20(20);
    geometry_msgs::Twist twist;
    MecanumController controller(nh);

    //先转一圈看看能不能直接找到目标
    // twist.angular.z = 0.2;
    // pub.publish(twist);

    //去到最中间
    tf2::Quaternion q;
    move_base_msgs::MoveBaseGoal goal;
    q.setRPY(0, 0, 0);
    goal.target_pose.pose.position.x = 1.25;
    goal.target_pose.pose.position.y = 3.75;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = q.x();
    goal.target_pose.pose.orientation.y = q.y();
    goal.target_pose.pose.orientation.z = q.z();    
    goal.target_pose.pose.orientation.w = q.w();

    //转一圈看看能不能直接找到目标
    ros::ServiceClient client = nh.serviceClient<ros_nanodet::detect_result_srv>("detect_result");
    ros::service::waitForService("detect_result");
    ros_nanodet::detect_result_srv start_detect;
    start_detect.request.detect_start = 1;

    while(ros::ok()){
        // controller.rotateFullCircle(0.52);
        ros::Time start = ros::Time::now();
        bool flag = client.call(start_detect);
        ros::Time end = ros::Time::now();
        ros::Duration duration = end - start;
        ROS_INFO("运行耗时: %.3f 秒", duration.toSec());
        if (flag)
        {
            ROS_INFO("结果：%d,%d,%d,%d,%d",start_detect.response.class_name,start_detect.response.x0,start_detect.response.x1,start_detect.response.y0,start_detect.response.y1);
        }
        else
        {
            ROS_WARN("请求处理失败....");
            return 1;
        }
    }
}