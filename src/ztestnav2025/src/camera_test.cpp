#include "ros/ros.h"
#include "ros_nanodet/detect_result_srv.h"

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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MecanumController {
public:
    MecanumController(ros::NodeHandle& nh) : 
        nh_(nh),
        tf_buffer_(),
        tf_listener_(tf_buffer_),
        cmd_pub_(nh.advertise<geometry_msgs::Twist>("cmd_vel", 10)),
        client_(nh.serviceClient<ros_nanodet::detect_result_srv>("detect_result")) 
    {if (!client_.waitForExistence(ros::Duration(7.0))) {
        ROS_FATAL("检测服务 detect_result 不可用！");
        throw std::runtime_error("Service detect_result not found");
    }}

    bool waitForTransform() {//等待TF初始化完成
        ros::Time start_time = ros::Time::now();
        // 设置超时时间（例如10秒）
        ros::Duration timeout(10.0);
        
        while (ros::ok() && (ros::Time::now() - start_time < timeout)) {
            if (tf_buffer_.canTransform("map", "base_link", ros::Time(0))) {
                return true;
            }
            ROS_WARN_THROTTLE(1, "等待坐标系变换中...");
            ros::Duration(0.1).sleep();  // 降低CPU占用
        }
        ROS_ERROR("等待超时:map到base_link的坐标变换未就绪");
        return false;
    }

    void detect(std::vector<int>& result){//封装目标检测功能
        start_detect_.request.detect_start = 1;
        bool flag = client_.call(start_detect_);
        if (flag)
        {
            result[0] = start_detect_.response.x0;result[1] = start_detect_.response.y0;result[2] = start_detect_.response.x1;result[3] = start_detect_.response.y1;result[4] = start_detect_.response.class_name;
            ROS_INFO("结果：%d,%d,%d,%d,%d",start_detect_.response.class_name,start_detect_.response.x0,start_detect_.response.x1,start_detect_.response.y0,start_detect_.response.y1);
        }
        else
        {
            ROS_WARN("请求处理失败....");
            return ;
        }
    }

    void rotateCircle(double rotate,int direction) {//控制小车运动，rotate是弧度,direction逆时针是正向
        geometry_msgs::Twist twist;
        ros::Rate rate(20);     // 控制频率20Hz
        auto start_opt = getCurrentPose();
        if (!start_opt) ROS_INFO("坐标获取失败");
        double start = getYawFromTransform(*start_opt);
        double target = start+rotate;
        if(target>=3.14){
            target = 3.14 - target;//最大就3.14，超过3.14变成-3.14
        }
        ROS_INFO("起点和目标:%f,%f",start,target);
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
            twist.angular.z = angular_speed_ * direction;
            cmd_pub_.publish(twist);
            rate.sleep();
        }
    }

    void turn_and_find(double x,int y,int z){//原地旋转小车x度，执行y次目标检测,寻找z号目标
        std::vector<int> result = {-1,-1,-1,-1,-1};
        for(int i=0;i<y;i++){
            detect(result);
            if(result[4] == z){
                ROS_INFO("找到目标");
                break;
            }
        }
        if(result[4] == -1){
            ROS_INFO("这个位置看不到目标");
        }
        else{//让小车对准目标
            if(result[0]+result[2] == img_width){//如果一开始就是准的直接退出
                return;
            }
            else{
                double target = std::abs(result[0]+result[2]-img_width)*width_per_pixel/(2.0*std::sqrt(focal_distance*focal_distance+(result[1]+result[3]-img_height)*(result[1]+result[3]-img_height)*height_per_pixel*height_per_pixel/4.0));
                rotateCircle(std::atan(target),std::copysign(1, result[0]+result[2]-img_width));
            }
            
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

    ros::ServiceClient client_;
    ros_nanodet::detect_result_srv start_detect_;//目标检测客户端

    int img_width = 800;
    int img_height = 600;
    int focal_distance = 2.8;//单位毫米
    double width_per_pixel = 10.7118/img_width;
    double height_per_pixel = 3.7066/img_height;

    // 坐标变换查询（参考网页8的异常处理）
    boost::optional<geometry_msgs::TransformStamped> getCurrentPose() {
        try {
            waitForTransform();
            return tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
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

    controller.rotateCircle(3.1,1);

    while(ros::ok()){
        bool flag = client.call(start_detect);
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