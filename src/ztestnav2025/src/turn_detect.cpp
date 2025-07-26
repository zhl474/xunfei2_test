#include "ztestnav2025/turn_detect.h"

template<typename T>
T clamp(T value, T low, T high) {
    return (value < low) ? low : (value > high) ? high : value;
}


MecanumController::MecanumController(ros::NodeHandle& nh) : 
    nh_(nh),
    cmd_pub_(nh.advertise<geometry_msgs::Twist>("cmd_vel", 10)),
    detect_client_(nh.serviceClient<ros_nanodet::detect_result_srv>("nanodet_detect")),
    getpose_client_(nh.serviceClient<ztestnav2025::getpose_server>("getpose_server")),
    set_speed_client_(nh.serviceClient<ztestnav2025::set_speed>("set_speed")),
    adjust_client_(nh.serviceClient<ztestnav2025::lidar_process>("/lidar_process/lidar_process"))
    // timer(nh.createTimer(ros::Duration(17.0), &MecanumController::timerCallback, this, false, false))
{
    if (!detect_client_.waitForExistence()) {
        ROS_FATAL("检测服务 nanodet_detect 不可用！");
        throw std::runtime_error("Service detect_result not found");
    }
    if (!getpose_client_.waitForExistence()) {
        ROS_FATAL("获取坐标服务 getpose_result 不可用！");
        throw std::runtime_error("Service getpose_result not found");
    }
    if (!set_speed_client_.waitForExistence()) {
        ROS_FATAL("运动控制服务 setspeed 不可用！");
        throw std::runtime_error("Service setspeed not found");
    }
    // server_.setCallback(boost::bind(&MecanumController::PID_change, this, _1, _2));
}

void MecanumController::detect(std::vector<std::vector<int>>& result, int object_num){//封装目标检测功能
    start_detect_.request.detect_start = object_num;//要先传个-1把摄像头打开
    ros::Time test = ros::Time::now();
    bool flag = detect_client_.call(start_detect_);
    if ((ros::Time::now()-test).toSec()>0.2){
        ROS_WARN("目标检测超时%f",(ros::Time::now()-test).toSec());
    }
    if (flag){
        result[0] = start_detect_.response.x0;result[1] = start_detect_.response.y0;result[2] = start_detect_.response.x1;result[3] = start_detect_.response.y1;result[4] = start_detect_.response.class_name;
        // ROS_INFO("结果：%d,%d,%d,%d,%d",start_detect_.response.class_name,start_detect_.response.x0,start_detect_.response.y0,start_detect_.response.x1,start_detect_.response.y1);
    }
    else{
        ROS_WARN("目标检测失败");
        return ;
    }
}

void MecanumController::cap_close(){
    start_detect_.request.detect_start = -2;
    bool flag = detect_client_.call(start_detect_);
    if (flag){
        ROS_INFO("目标检测摄像头已关闭");
    }
    else{
        ROS_WARN("请求处理失败....");
        return ;
    }
}

void MecanumController::rotateCircle(double rotate, double angular_speed) {//控制小车运动，rotate是弧度
    geometry_msgs::Twist twist;
    ros::Rate rate(20);     // 控制频率20Hz

    std::vector<float> position_array = getCurrentPose();
    double start = position_array[2];
    double target = start+rotate;
    if(target>=3.14){
        target = target - 6.28;//最大就3.14，超过3.14变成-3.14
    }
    // ROS_INFO("起点和目标:%f,%f",start,target);
    while (ros::ok()) {
        // 获取当前姿态
        std::vector<float> now_yaw = getCurrentPose();
        if (now_yaw.empty()) continue;

        // 计算旋转控制量（参考网页2的PWM控制原理）
        double yaw = now_yaw[2];
        double yaw_error = fabs(yaw - target);
        if (yaw_error <= angle_error_) {
            break;
        }

        // 发送运动指令（参考网页1的速度发布逻辑）
        if(angular_speed>0) twist.angular.z = std::min(std::max(angular_speed * (yaw_error+0.2),0.5),2.0);
        else twist.angular.z = std::max(std::min(angular_speed * (yaw_error+0.2),-0.5),-2.0);
        ROS_INFO("误差%f速度%f",yaw_error,twist.angular.z);
        cmd_pub_.publish(twist);
        rate.sleep();
    }
}

// int MecanumController::turn_and_find(double find_time,int z,double angular_speed){//原地旋转小车x度，执行y次目标检测,寻找z号目标
//     std::vector<std::vector<int>> result = {{-1},{-1},{-1},{-1},{-1},{-1}};
//     double integral = 0, prev_error = 0;
//     // ros::Rate rate(20);     // 控制频率20Hz
//     set_speed_.request.work = true;
//     start_time_ = ros::Time::now();
//     double Kp,Ki,Kd;
//     nh_.getParam("/myplanernav/turn_find_p",Kp);
//     nh_.getParam("/myplanernav/turn_find_i",Ki);
//     nh_.getParam("/myplanernav/turn_find_d",Kd);

//     bool exit_flag = false;
//     while(ros::ok()&&!exit_flag){
//         // ros::Time test_time = ros::Time::now();
//         detect(result, z);     // 持续检测目标
//         // ROS_INFO("目标检测结果%d",result[4]);
//         if(result[4][i] < (z-1)*3 || result[4][i] >= z*3){
//             set_speed_.request.target_twist.angular.z = angular_speed;
//             set_speed_client_.call(set_speed_);
//             integral = 0;
//             if ((ros::Time::now() - start_time_).toSec()>find_time){
//                 exit_flag = true;
//                 result[4][i] = -1;
//                 ROS_INFO("找板超时");
//             }
//             continue;
//         }  // 目标丢失则旋转寻找目标
//         start_time_ = ros::Time::now();//找到目标就刷新开始时间免得一帧没检测到板子又退出去了
//         // 计算中心点偏差（误差输入）
//         int center_x = (result[0][i]+result[2][i])/2;
//         // ROS_INFO("中心点偏差%d",center_x);
//         // 退出条件：误差<7像素
//         if(std::abs(center_x - img_width/2) < 7){
//             ROS_INFO("已经对准");
//             integral = 0;
//             set_speed_.request.target_twist.angular.z = 0;
//             set_speed_.request.work = false;
//             set_speed_client_.call(set_speed_);
//             exit_flag = false;
//             return result[4][i];
//         } 
//         double error = (img_width/2.0 - center_x)/100; 
        
//         // 离散PID计算
//         integral += error * 0.2;       // dt=1/20≈0.05
//         integral = clamp(integral, -1.0, 1.0);
//         double derivative = (error - prev_error)/0.2;
//         double output = Kp*error + Ki*integral + Kd*derivative;
//         output = clamp(output, -0.4, 0.4);
//         // ROS_INFO("error:%f",error);
//         // ROS_INFO("P:%f",Kp*error);
//         // ROS_INFO("I:%f",Ki*integral);
//         // ROS_INFO("D:%f",Kd*derivative);
//         ROS_INFO("速度发布:%f",output);
        
//         // 执行旋转（限制输出范围）
//         set_speed_.request.target_twist.angular.z = output;
//         set_speed_client_.call(set_speed_);
        
//         prev_error = error;
//         // ROS_INFO("耗时%f",(ros::Time::now()-test_time).toSec());
//     }
//     exit_flag = false;
//     set_speed_.request.target_twist.linear.z = 0;
//     set_speed_.request.work = false;
//     set_speed_client_.call(set_speed_);
//     return result[4][i];
// }

bool MecanumController::test_point(double yaw,double distance){
    if(yaw>-0.95 &&yaw<=0.785){
        if(distance>1.25/cos(yaw)-0.5){
            return false;
        }
        return true;
    }
    else if(yaw>0.785 && yaw<=2.355){
        if(distance>1.25/sin(yaw)-0.5){
            return false;
        }
        return true;
    }
    else if(yaw>-2.19 && yaw<=-0.95){
        if(distance>1.75/abs(sin(yaw))-0.5){
            return false;
        }
        return true;
    }
    else{
        if(distance>1.25/abs(cos(yaw))-0.5){
            return false;
        }
        return true;
    }
}

bool MecanumController::turn_and_find_plus(double find_time,int z,double angular_speed,double& targetx,double& targety,double& targetz,double& targetx2,double& targety2,double& targetz2){//板子相互遮挡时，直接识别错误板，开到板子后面去
    std::vector<std::vector<int>> result = {{-1},{-1},{-1},{-1},{-1},{-1}};
    double integral = 0, prev_error = 0;
    set_speed_.request.work = true;
    start_time_ = ros::Time::now();
    double Kp,Ki,Kd;
    nh_.getParam("/myplanernav/turn_find_p",Kp);
    nh_.getParam("/myplanernav/turn_find_i",Ki);
    nh_.getParam("/myplanernav/turn_find_d",Kd);

    bool find = false,the_first = false,center_time = true,exit_flag = false;//标志位判断找到目标没有,非目标板最多就两个

    set_speed_.request.target_twist.angular.z = angular_speed;
    set_speed_client_.call(set_speed_);
    while(ros::ok()&&!exit_flag  && center_time){
        // ros::Time test_time = ros::Time::now();
        detect(result, z);     // 持续检测目标
        // ROS_INFO("目标检测结果%d",result[4]);遍历视野内所有目标，如果有二维码匹配项，则直接进入对准逻辑，如果没有二维码匹配项，记录第一个超过画面中心的板子
        if(!find){//还没找到板子，先定速旋转
            if ((ros::Time::now() - start_time_).toSec()>find_time){
                center_time = false;
                start_time_ = ros::Time::now();
                ROS_INFO("中心找板超时");
            }
            for(size_t i=0;i<result[0].size();i++){
                if(result[4][i] >= (z-1)*3 && result[4][i] < z*3){//如果直接把二维码匹配项找到了，直接进入对准逻辑
                    find = true;
                    start_time_ = ros::Time::now();
                    break;
                }
                if((result[0][i]+result[2][i])/2>320){//如果非目标超过了画面中心，记录当前位置和前方雷达距离，一会要过来
                    board_slope.request.lidar_process_start = 1;
                    adjust_client_.call(board_slope);
                    std::vector<float> position = getCurrentPose();
                    if(test_point(position[2],board_slope.response.lidar_results[0])){
                        if(the_first){
                            targetx = (board_slope.response.lidar_results[0]-0.6)*cos(position[2])+position[0];
                            targety = (board_slope.response.lidar_results[0]-0.6)*sin(position[2])+position[1];
                            targetz = position[2];
                            the_first = false;
                        }
                        else{
                            targetx2 = (board_slope.response.lidar_results[0]-0.6)*cos(position[2])+position[0];
                            targety2 = (board_slope.response.lidar_results[0]-0.6)*sin(position[2])+position[1];
                            targetz2 = position[2];
                        }
                    }
                }
            }
        }

        if(find){
            int center_x = 0;
            for(size_t i=0;i<result[0].size();i++){
                if(result[4][i] >= (z-1)*3 && result[4][i] < z*3){
                    center_x = (result[0][i]+result[2][i])/2;
                    break;
                }  // 计算目标误差
            }
            start_time_ = ros::Time::now();//找到目标就刷新开始时间免得一帧没检测到板子又退出去了
            // ROS_INFO("中心点偏差%d",center_x);
            // 退出条件：误差<7像素
            if(std::abs(center_x - img_width/2) < 7){
                ROS_INFO("已经对准");
                integral = 0;
                set_speed_.request.target_twist.angular.z = 0;
                set_speed_.request.work = false;
                set_speed_client_.call(set_speed_);
                exit_flag = false;
                targetx2 = (board_slope.response.lidar_results[0]-0.6)*cos(position[2])+position[0];
                targety2 = (board_slope.response.lidar_results[0]-0.6)*sin(position[2])+position[1];
                targetz2 = position[2];
                return true;
            } 
            double error = (img_width/2.0 - center_x)/100; 
            
            // 离散PID计算
            integral += error * 0.2;       // dt=1/20≈0.05
            integral = clamp(integral, -1.0, 1.0);
            double derivative = (error - prev_error)/0.2;
            double output = Kp*error + Ki*integral + Kd*derivative;
            output = clamp(output, -0.4, 0.4);
            // ROS_INFO("error:%f",error);
            // ROS_INFO("P:%f",Kp*error);
            // ROS_INFO("I:%f",Ki*integral);
            // ROS_INFO("D:%f",Kd*derivative);
            ROS_INFO("速度发布:%f",output);
            
            // 执行旋转（限制输出范围）
            set_speed_.request.target_twist.angular.z = output;
            set_speed_client_.call(set_speed_);
            
            prev_error = error;
            // ROS_INFO("耗时%f",(ros::Time::now()-test_time).toSec());
        }
    }
    set_speed_.request.target_twist.linear.z = 0;
    set_speed_.request.work = false;
    set_speed_client_.call(set_speed_);
    return false;
}

bool MecanumController::forward(int z,double forward_speed){
    std::vector<std::vector<int>> result = {{-1},{-1},{-1},{-1},{-1},{-1}};
    board_slope.request.lidar_process_start = 1;
    double integral = 0, prev_error = 0;
    set_speed_.request.target_twist.linear.x = 0.15;
    set_speed_.request.work = true;
    while(ros::ok()){
        adjust_client_.call(board_slope);
        // ROS_INFO("%f",board_slope.response.lidar_results[0]);
        if(board_slope.response.lidar_results[0] < 0.4){
            set_speed_.request.target_twist.linear.x = 0;
            set_speed_.request.target_twist.angular.z = 0;
            set_speed_.request.work = false;
            set_speed_client_.call(set_speed_);
            return true;
        }  // 利用雷达判定已经接近目标
        set_speed_client_.call(set_speed_);
        
        // prev_error = error;
    }
    set_speed_.request.target_twist.linear.x = 0;
    set_speed_.request.target_twist.angular.z = 0;
    set_speed_.request.work = false;
    set_speed_client_.call(set_speed_);
    return false;
}

int MecanumController::adjust(int z,double adjust_speed){
    std::vector<std::vector<int>> result = {{-1},{-1},{-1},{-1},{-1},{-1}};
    double integral = 0, prev_error = 0;
    double lidar_integral = 0, lidar_prev_error = 0;
    set_speed_.request.target_twist.linear.x = 0;
    set_speed_.request.target_twist.angular.z = 0;
    set_speed_.request.work = true;
    board_slope.request.lidar_process_start = 2;
    
    int count = 0;//连续三帧目标都在中心，才认为对准
    int failed_conut = 0;//连续5帧找不到目标，判定为目标丢失
    double p,i,d,p1,i1,d1;
    nh_.getParam("/myplanernav/adjust_detecet_P",p);
    nh_.getParam("/myplanernav/adjust_detecet_I",i);
    nh_.getParam("/myplanernav/adjust_detecet_D",d);
    nh_.getParam("/myplanernav/adjust_lidar_P",p1);
    nh_.getParam("/myplanernav/adjust_lidar_I",i1);
    nh_.getParam("/myplanernav/adjust_lidar_D",d1);

    int target_board = -1,center_x;
    
    while(ros::ok()){
        bool find = false;
        start_time_ = ros::Time::now();
        detect(result, z);     // 持续检测目标
        for(size_t i=0;i<result[0].size();i++){
            if(result[4][i] >= (z-1)*3 && result[4][i] < z*3){//如果直接把二维码匹配项找到了，直接进入对准逻辑
                target_board = result[4][i];
                find = true;
                center_x = (result[0][i]+result[2][i])/2;
                break;
            }
        }
        if(!find) continue;//可能会有几帧识别不到
        if(std::abs(center_x - img_width/2) < 20){
            integral = 0;
            ROS_INFO("在视野中心");
            set_speed_.request.target_twist.linear.y = 0;
            set_speed_client_.call(set_speed_);
        } 
        double error = (img_width/2.0 - center_x)/100; 
        // ROS_INFO("error:%f",error);
        // 离散PID计算
        integral += error*0.4;      
        integral = clamp(integral, -1.5, 1.5);
        double derivative = (error - prev_error)/0.2;
        double output = p*error + i*integral + d*derivative;
        // ROS_INFO("error:%f",error);
        // ROS_INFO("P:%f",p*error);
        // ROS_INFO("I:%f",i*integral);
        // ROS_INFO("D:%f",d*derivative);
        output = clamp(output, -0.15, 0.15);
        prev_error = error;
        // ROS_INFO("速度发布:%f",output);
        // 执行（限制输出范围）
        set_speed_.request.target_twist.linear.y = output;

        double lidar_output;
        if(adjust_client_.call(board_slope)){
            if(std::abs(board_slope.response.lidar_results[0]) < 0.1){
                lidar_integral = 0;
                set_speed_.request.target_twist.angular.z = 0;
                set_speed_client_.call(set_speed_);
            } 
            lidar_integral += board_slope.response.lidar_results[0] * 0.2;
            lidar_integral = clamp(lidar_integral, -1.0, 1.0);
            double lidar_derivative = (board_slope.response.lidar_results[0] - lidar_prev_error)/0.2;
            lidar_output = p1*board_slope.response.lidar_results[0] + lidar_integral*i1 + lidar_derivative*d1;
            // ROS_INFO("error:%f",board_slope.response.lidar_results[0]);
            // ROS_INFO("P:%f",p1*board_slope.response.lidar_results[0]);
            // ROS_INFO("I:%f",lidar_integral*i1);
            // ROS_INFO("D:%f",d1*lidar_derivative);
            lidar_output = clamp(lidar_output, -0.13, 0.13);
            // ROS_INFO("速度发布:%f",lidar_output);
            lidar_prev_error = board_slope.response.lidar_results[0];
        }
        if(std::abs(center_x - img_width/2) < 20 && std::abs(board_slope.response.lidar_results[0]) < 0.1){
            count++;
            set_speed_.request.target_twist.linear.y = 0;
            set_speed_.request.target_twist.angular.z = 0;
            set_speed_client_.call(set_speed_);
            ROS_INFO("满足退出条件第%d次",count);
            if (count>3){
                set_speed_.request.work = false;
                set_speed_client_.call(set_speed_);
                return target_board;//连续三帧都合格才退出
            }
            continue;
        }  // 已经接近目标退出循环
        else{
            count = 0;
        }
        set_speed_.request.target_twist.angular.z = lidar_output;
        set_speed_client_.call(set_speed_);
        // ROS_INFO("控制一次耗时%f",(ros::Time::now()-start_time_).toSec());
    }
    set_speed_.request.target_twist.linear.x = 0;
    set_speed_.request.target_twist.linear.y = 0;
    set_speed_.request.target_twist.angular.z = 0;
    set_speed_.request.work = false;
    set_speed_client_.call(set_speed_);
    
    return -1;
}


std::vector<float> MecanumController::getCurrentPose(){
    start_get_pose_.request.getpose_start= 1;
    if (getpose_client_.call(start_get_pose_)){
        // ROS_INFO("请求正常处理,响应结果长度%zu",start_get_pose_.response.pose_at.size());
        std::vector<float> pose_array = start_get_pose_.response.pose_at;
        return pose_array;
    }
    else{
        ROS_ERROR("请求处理失败....");
        return {};
    }
}

