#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "tensor_rt/Messages.h"
// 新增头文件
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <cmath>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// 全局变量存储识别结果
uint16_t room_a_class = -1, room_b_class = -1, room_c_class = -1;


//在机器人停止后，执行目标的精确定位和逼近导航
int Approach(
    int goal_num,
    MoveBaseClient& ac,
    ros::ServiceClient& detection_client,
    tf2_ros::Buffer& tf_buffer)
{
    ROS_INFO("已停止移动，开始精确探测目标板 (任务 %d)", goal_num);

    // 调用服务进行物体检测
    tensor_rt::Messages srv;
    if (!detection_client.call(srv) || srv.response.result.empty()) {
        ROS_ERROR("在探测点 goal_%d. 调用服务失败或者返回结果为空", goal_num);
        return -1;
    }

    // 从检测结果中找到置信度最高的目标
    int best_box_x = -1, best_box_w = -1;
    int detected_class = -1;
    int max_confidence = -1;
    for (size_t i = 0; i < srv.response.result.size(); i += 6) {
        if (srv.response.result[i + 5] > max_confidence) {
            max_confidence = srv.response.result[i + 5];
            best_box_x = srv.response.result[i];
            best_box_w = srv.response.result[i + 2];
            detected_class = srv.response.result[i + 4];    
        }
    }
    if (detected_class == -1) {
        ROS_ERROR("未检测到图案");
        return -1;
    }
    ROS_INFO("Detected class %d with confidence %d%%.", detected_class, max_confidence);

    // 获取最新的Lidar数据
    ROS_INFO("等待雷达服务");
    sensor_msgs::LaserScan::ConstPtr scan_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", ros::Duration(2.0));
    if (!scan_msg) {
        ROS_ERROR("无法接收到雷达数据");
        return -1;
    }

    // 计算目标在相机和雷达坐标系中的位置
    const double CAMERA_HFOV = 1.3962634; // 相机水平视场角，在xacro文件中
    const int IMAGE_WIDTH = 640; // 图像宽度，实际相机宽度是1920，但OpenCV绘制出的是640
    
    double center_x = best_box_x + best_box_w / 2.0;
    double angle_in_camera = (IMAGE_WIDTH / 2.0 - center_x) / (IMAGE_WIDTH / 2.0) * (CAMERA_HFOV / 2.0);
    //这里的数组range的范围是720，0.5度一个储存值，把目标的偏移角转化为0.5的整数倍

    int scan_index = (int)((angle_in_camera - scan_msg->angle_min) / scan_msg->angle_increment);
    ROS_INFO("range的索引为%d",scan_index);//判断一下求取索引值的算法对不对
    if (scan_index < 0 || scan_index >= scan_msg->ranges.size()) {
        ROS_ERROR("索引值超出range数组范围");// 检查计算出的索引是否在合法范围内
        return -1;
    }

    double distance = scan_msg->ranges[scan_index];
    if (std::isinf(distance) || std::isnan(distance)) {
        ROS_ERROR("雷达距离无效");
        return -1;
    }

    // 将目标点从雷达坐标系转换到map坐标系
    geometry_msgs::PointStamped target_point_laser_frame;// 创建一个带坐标系信息的点
    target_point_laser_frame.header.stamp = ros::Time::now();// 设置时间戳为当前时间
    target_point_laser_frame.header.frame_id = scan_msg->header.frame_id;// 设置坐标系为雷达的坐标系 ("laser_frame")
    target_point_laser_frame.point.x = distance * std::cos(angle_in_camera);
    target_point_laser_frame.point.y = distance * std::sin(angle_in_camera);
    target_point_laser_frame.point.z = 0;// 因为是2D雷达，z坐标为0

    geometry_msgs::PointStamped target_point_map_frame;// 创建一个用于存储转换后结果的点
    try {
        tf_buffer.transform(target_point_laser_frame, target_point_map_frame, "map", ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {// 如果变换失败（例如tf树中断）
        ROS_WARN("Could not transform point from %s to map: %s", scan_msg->header.frame_id.c_str(), ex.what());
        return -1;
    }
    ROS_INFO("Target board located at map coordinates: [x: %.2f, y: %.2f]", target_point_map_frame.point.x, target_point_map_frame.point.y);
    
    // 计算最终的逼近导航点(在目标前方0.5米)
    move_base_msgs::MoveBaseGoal approach_goal;
    approach_goal.target_pose.header.frame_id = "map";
    approach_goal.target_pose.header.stamp = ros::Time::now();
    // 获取机器人当前在"map"坐标系下的位姿，以便计算相对方向
    geometry_msgs::TransformStamped robot_pose_transform;
    try {// 查询从"map"到"base_link"的变换关系
        robot_pose_transform = tf_buffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("无法获得小车当前位置: %s", ex.what());
        return -1;
    }
    // 提取机器人和目标板的x, y坐标
    double robot_x = robot_pose_transform.transform.translation.x;
    double robot_y = robot_pose_transform.transform.translation.y;
    double target_x = target_point_map_frame.point.x;
    double target_y = target_point_map_frame.point.y;
    
    double angle_to_target = std::atan2(target_y - robot_y, target_x - robot_x);
    const double APPROACH_DISTANCE = 0.5;// 逼近距离 (米)
    // 计算逼近点的坐标：从目标板位置沿着“机器人->目标板”方向的反方向后退APPROACH_DISTANCE米
    approach_goal.target_pose.pose.position.x = target_x - APPROACH_DISTANCE * std::cos(angle_to_target);
    approach_goal.target_pose.pose.position.y = target_y - APPROACH_DISTANCE * std::sin(angle_to_target);
    // 设置逼近点的姿态，使其朝向正好指向目标板
    tf2::Quaternion q;
    q.setRPY(0, 0, angle_to_target);// 创建一个只绕Z轴旋转的四元数
    approach_goal.target_pose.pose.orientation = tf2::toMsg(q);// 将tf2四元数转换为geometry_msgs格式


    // 导航至逼近点
    ROS_INFO("发送目标板位置 target_%d", goal_num);
    ac.sendGoal(approach_goal);// 发送最终的逼近目标
    ac.waitForResult();// 返回本次识别到的物体类别ID

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("到达目标板位置 target_%d.", goal_num);
        return detected_class;
    } else {
        ROS_ERROR("无法到达目标板位置 target_%d.", goal_num);
        return -1;
    }
}


//导航至房间区域，并在途中持续检测目标
int driveAndDetect(
    move_base_msgs::MoveBaseGoal& room_goal,
    int goal_num,
    MoveBaseClient& ac,
    ros::ServiceClient& detection_client,
    tf2_ros::Buffer& tf_buffer,
    int avoid_class)
{
    ROS_INFO("开始前往房间 %d 的观测点...", goal_num);
    ac.sendGoal(room_goal);

    ros::Rate loop_rate(5); // 设置检测频率为5Hz
    tensor_rt::Messages srv;

    // 当机器人还在移动时，持续检测
    while (ros::ok() && (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE || ac.getState() == actionlib::SimpleClientGoalState::PENDING))
    {
        // 调用检测服务
        if (detection_client.call(srv) &&!srv.response.result.empty() && avoid_class != srv.response.result[4])//检测到的目标不能是上一个目标
        {
            ROS_INFO("在前往房间 %d 的途中检测到目标! 停止当前导航并开始逼近...", goal_num);
            ac.cancelAllGoals(); // 立刻停止机器人
            ros::Duration(0.3).sleep(); // 短暂等待，让机器人稳定下来。不等一小会雷达数据可能出错
            // 调用精确逼近函数
            return Approach(goal_num, ac, detection_client, tf_buffer);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    // 如果循环结束是因为成功到达了目的地,在途中没有检测到目标
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("已到达房间 %d 的最终观测点，途中未检测到目标，开始最后的探测...", goal_num);
        // 在目的地再进行一次检测（感觉用处不大）
        return Approach(goal_num, ac, detection_client, tf_buffer);
    }
    else // 如果循环结束是因为导航失败
    {
        ROS_ERROR("未能到达房间 %d 的观测点。状态: %s", goal_num, ac.getState().toString().c_str());
        return -1;
    }
}


int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "nav_client_approacher");
    ros::NodeHandle nh;

    MoveBaseClient ac("move_base", true);
    ros::ServiceClient client = nh.serviceClient<tensor_rt::Messages>("tensorRT_detect");
    
    tf2_ros::Buffer tf_buffer;// 创建一个TF缓冲区对象，用于存储坐标变换信息
    tf2_ros::TransformListener tf_listener(tf_buffer);// 创建一个TF监听器，并将其绑定到缓冲区

    ROS_INFO("等待move_base action server...");
    ac.waitForServer();
    ROS_INFO("等待 tensorRT_detect service...");
    client.waitForExistence();
    ROS_INFO("所有服务成功启动");

    move_base_msgs::MoveBaseGoal goal;// 创建一个可复用的导航目标对象
    goal.target_pose.header.frame_id = "map";// 所有目标都在"map"坐标系下



    // 前往房间 C 区域
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 3.735;
    goal.target_pose.pose.position.y = 1.055;
    goal.target_pose.pose.orientation.z = 0.7071;
    goal.target_pose.pose.orientation.w = 0.7071;
    room_c_class = driveAndDetect(goal, 1, ac, client, tf_buffer, 9);//第一次没有需要避免的对象
    ROS_INFO("_room_c_class = %d",room_c_class);
    //房间最内侧的板有可能因为速度过快导致错过，错过时重新来一次
    if(room_c_class == 65535)//返回值-1不在范围内，最终会变成65535
    {
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = 3.735;
        goal.target_pose.pose.position.y = 1.555;
        goal.target_pose.pose.orientation.z = 0.7071;
        goal.target_pose.pose.orientation.w = 0.7071;
        room_c_class = driveAndDetect(goal, 1, ac, client, tf_buffer, 9);
        ROS_INFO("_room_c_class = %d",room_c_class);
    }
    //前往房间 B 区域
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 2.727;
    goal.target_pose.pose.position.y = 1.214;
    goal.target_pose.pose.orientation.z = 0.866;
    goal.target_pose.pose.orientation.w = 0.5;
    ac.sendGoal(goal);//先到目标点，再持续检测，防止被上一块板截停
    ac.waitForResult();
    goal.target_pose.pose.position.x = 2.727;//房间B特殊处理：转向过程中持续检测，不容易错过卡在最右边的板
    goal.target_pose.pose.position.y = 1.214;
    goal.target_pose.pose.orientation.z = 0.7071;
    goal.target_pose.pose.orientation.w = 0.7071;
    room_b_class = driveAndDetect(goal, 2, ac, client, tf_buffer, room_c_class);

    // 前往房间 A 区域
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 0.486;
    goal.target_pose.pose.position.y = 0.839;
    goal.target_pose.pose.orientation.z = 1;
    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.w = 0;
    ac.sendGoal(goal);
    ac.waitForResult();
    goal.target_pose.pose.position.x = 0.486;//房间C特殊处理：转向过程中持续检测，不容易错过卡在最左边的板
    goal.target_pose.pose.position.y = 0.839;
    goal.target_pose.pose.orientation.z = 0.5;
    goal.target_pose.pose.orientation.w = 0.866;
    room_a_class = driveAndDetect(goal, 3, ac, client, tf_buffer, room_b_class);

    // 返回原点
    ROS_INFO("返回起点");
    goal.target_pose.header.stamp = ros::Time::now();//在返回起点的中途设一个点，防止因为角速度过快导致膨胀层漂移把路封死
    goal.target_pose.pose.position.x = 3;
    goal.target_pose.pose.position.y = 0;
    goal.target_pose.pose.orientation.z = 1;
    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.w = 0;
    ac.sendGoal(goal);
    ac.waitForResult();
    
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 0.1;//x设为零的时候距离墙过近，在原地旋转时会卡墙
    goal.target_pose.pose.position.y = 0;
    goal.target_pose.pose.orientation.z = 0;
    goal.target_pose.pose.orientation.w = 1;
    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    ac.sendGoal(goal);
    ac.waitForResult();

    ROS_INFO("Room C -> Class %d, Room B -> Class %d, Room A -> Class %d",
             room_c_class, room_b_class, room_a_class);
    return 0;
}
