#include <ros/ros.h>
#include <ros/master.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Config.h>
#include <tf/transform_listener.h>
using namespace std;

int awake_flag;
uint8_t room_index = 3;
uint8_t past_room_index = 3;
uint8_t goal_number = 0;
float theta = 0;
uint8_t success_detect = 0; //判断识别准确率，若大于      ，返回1，到下一房间。反之，到下一点。
int detect_flag=0;     //目标检测标志位
double x;
double y;
char *voice[4][4]={{"aplay ~/ucar_car/src/simple_navigation_goals/voice/B_corn.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/B_cucumber.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/B_rice.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/B_wheat.wav"},
                    {"aplay ~/ucar_car/src/simple_navigation_goals/voice/C_corn.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/C_cucumber.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/C_rice.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/C_wheat.wav"},
                    {"aplay ~/ucar_car/src/simple_navigation_goals/voice/D_corn.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/D_cucumber.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/D_rice.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/D_wheat.wav"},
                    {"aplay ~/ucar_car/src/simple_navigation_goals/voice/E_corn.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/E_cucumber.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/E_rice.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/E_wheat.wav"}};
char *num[7] = {"aplay ~/ucar_car/src/simple_navigation_goals/voice/1.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/2.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/3.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/4.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/5.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/6.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/7.wav"};
int count[4]={0};
int area_sort[4] = {4,4,4,4};   //各房间种物类型BCDE
int result;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    tf::TransformListener listener;
void mic_awake_cb(const std_msgs::Int8::ConstPtr& a_p)
{
    awake_flag = a_p->data;             //传递a_p值
    ROS_INFO("awake flag is set");
}

void area_msg_cb(const std_msgs::Int8::ConstPtr& r_p )
{
    result = r_p->data;
    detect_flag = 1;
    if (result == 10){//接受到10表示是空板 或者检测正确率不高 明天记得改已改
        success_detect = 0;
        ROS_INFO("detect result  : %d !",result); 
    }
    else{
        ROS_INFO("detect result : %d !",result);
        success_detect = 1;
    }
}

uint8_t all_detect(){
    if(area_sort[1] != 4 && area_sort[2] != 4 && area_sort[3] != 4)
        return 1;
    else 
        return 0;
}

uint8_t tf_weizhi(){
    tf::StampedTransform transform;
    try {
    listener.lookupTransform("map", "base_link", ros::Time(0), transform);
    } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    // ros::Duration(1.0).sleep();
    // continue;
    }
    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    if(x >= 2.25 && x <= 3.5 && y>= -2.15 && y<= 0){
        return 3;
    }
    else if (x >= 2.25 && x <= 3.5 && y>= -5.3 && y<= -3){
        return 2;
    }
    else if (x >= 3.7 && x <= 5.4 && y>= -2.15 && y<= 0){
        return 1;
    }
    else if (x >= 3.7 && x <= 5.4 && y>= -5.3 && y<= -3){
        return 0;
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "send_goals_circle");
    ros::NodeHandle n;
    ros::Publisher detect_pub = n.advertise<std_msgs::Int8>("/start_detect",1);//publisher
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    ros::Subscriber awake_sub = n.subscribe<std_msgs::Int8>("awake_flag",1,mic_awake_cb);  //语音唤醒订阅
    ros::Subscriber room_sub = n.subscribe<std_msgs::Int8>("/area_msg",3,area_msg_cb); //房间类型订阅
    // ros::Subscriber room_sub_f = n.subscribe<std_msgs::String>("/area_msgf",1,area_msg_cb_f);
    geometry_msgs::Twist twist;
    std_msgs::Int8 start_detect;           //控制识别启动，1时启动

    ros::Time before = ros::Time::now();
    ros::Time after = ros::Time::now();
    MoveBaseClient ac("move_base", true);
move_base_msgs::MoveBaseGoal goal[20];
//E guding
    goal[0].target_pose.pose.position.x =2.931;
goal[0].target_pose.pose.position.y = -0.368;
goal[0].target_pose.pose.orientation.z =0.365;  
goal[0].target_pose.pose.orientation.w = 0.931;
    //D jin
    goal[1].target_pose.pose.position.x =2.815;
goal[1].target_pose.pose.position.y = -3.300;
goal[1].target_pose.pose.orientation.z =-0.7;  
goal[1].target_pose.pose.orientation.w = 0.7;
    //C jin
    goal[2].target_pose.pose.position.x =4.943;
goal[2].target_pose.pose.position.y = -1.879;
goal[2].target_pose.pose.orientation.z =0.7;  
goal[2].target_pose.pose.orientation.w = 0.7;
    //C中
    goal[3].target_pose.pose.position.x =4.666;
goal[3].target_pose.pose.position.y = -0.182;
goal[3].target_pose.pose.orientation.z =-0.7;  
goal[3].target_pose.pose.orientation.w = 0.700;
    //B jin
    goal[4].target_pose.pose.position.x =4.819;
	goal[4].target_pose.pose.position.y = -3.259;
	goal[4].target_pose.pose.orientation.z =-0.7;  
	goal[4].target_pose.pose.orientation.w = 0.7;
    //B zhong
    goal[5].target_pose.pose.position.x =4.602;
	goal[5].target_pose.pose.position.y = -4.929;
	goal[5].target_pose.pose.orientation.z = 0.7;  
	goal[5].target_pose.pose.orientation.w =  0.7;
    //E suiji
    goal[6].target_pose.pose.position.x= 2.949;
    goal[6].target_pose.pose.position.y = -1.559;
    goal[6].target_pose.pose.orientation.z =0.947;  
    goal[6].target_pose.pose.orientation.w = -0.322;
    //E suiji 
    goal[7].target_pose.pose.position.x =2.931;
    goal[7].target_pose.pose.position.y = -0.368;
    goal[7].target_pose.pose.orientation.z =0.365;  
    goal[7].target_pose.pose.orientation.w = 0.931;
    //E suiji
    goal[8].target_pose.pose.position.x =0.917;
    goal[8].target_pose.pose.position.y = -0.117;
    goal[8].target_pose.pose.orientation.z =-0.725;  
    goal[8].target_pose.pose.orientation.w = 0.689;
    //E suiji
    goal[9].target_pose.pose.position.x =0.891;
	goal[9].target_pose.pose.position.y =-3.005;
	goal[9].target_pose.pose.orientation.z =-0.700;  
	goal[9].target_pose.pose.orientation.w = 0.700;
    //F jin
    goal[10].target_pose.pose.position.x =0.941;
	goal[10].target_pose.pose.position.y = -4.803;
	goal[10].target_pose.pose.orientation.z =-0.273;  
	goal[10].target_pose.pose.orientation.w = 0.962;
    //Fzuo
    goal[11].target_pose.pose.position.x =0.941;
	goal[11].target_pose.pose.position.y = -4.803;
	goal[11].target_pose.pose.orientation.z =0.962;  
	goal[11].target_pose.pose.orientation.w = -0.273;
    //Fyou
    goal[11].target_pose.pose.position.x =0.941;
	goal[11].target_pose.pose.position.y = -4.803;
	goal[11].target_pose.pose.orientation.z =0.962;  
	goal[11].target_pose.pose.orientation.w = -0.273;
    //F中1
    goal[12].target_pose.pose.position.x =0.910;
	goal[12].target_pose.pose.position.y = -3.949;
	goal[12].target_pose.pose.orientation.z = 0.211;  
	goal[12].target_pose.pose.orientation.w =0.978;
    //FB
    goal[16].target_pose.pose.position.x =0.910;
	goal[16].target_pose.pose.position.y = -2.558;
	goal[16].target_pose.pose.orientation.z = 0.449;  
	goal[16].target_pose.pose.orientation.w = 0.894;
    //FC
    goal[17].target_pose.pose.position.x =0.910;
	goal[17].target_pose.pose.position.y = -2.558;
	goal[17].target_pose.pose.orientation.z = 0.890;  
	goal[17].target_pose.pose.orientation.w =  0.455;
    //Fchu
    goal[18].target_pose.pose.position.x =0.937;
	goal[18].target_pose.pose.position.y = -0.345;
	goal[18].target_pose.pose.orientation.z =  0.700;  
	goal[18].target_pose.pose.orientation.w = 0.700;

    //0.019, 0.035, 0.000), Orientation(0.000, 0.000, 1.000, 0.005)
    goal[19].target_pose.pose.position.x = 0.084;
	goal[19].target_pose.pose.position.y = 0.068;
	goal[19].target_pose.pose.orientation.z =  0.100;  
	goal[19].target_pose.pose.orientation.w = 0.000;

    start_detect.data = 1;                                           //到达目标点后，识别位置1
    detect_pub.publish(start_detect);                //发布开启识别
         
    ROS_INFO(" Init success!!!");
    while(!awake_flag)                  //等待语音唤醒标志位
    {
        ros::spinOnce();           
    }
    ROS_INFO("RUNNING!");
    //等待action回应
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    goal[goal_number].target_pose.header.frame_id = "map";
    goal[goal_number].target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal[goal_number]);
    ROS_INFO("Send NO. %d Goal !!! room %d", goal_number,room_index );
    ac.waitForResult();
    start_detect.data = 2;                           //到达目标点后，识别位置1
    detect_pub.publish(start_detect);                //发布开启识别
    ROS_INFO("successfully send!here in room_index=1");
    before = ros::Time::now();
    after = ros::Time::now();
    while(theta< 0.7){
        ros::spinOnce();
        twist.angular.z = 0.2;
        twist.linear.x = 0.2;
        twist.linear.y = 0.2;
        cmd_pub.publish(twist);
        after = ros::Time::now();
        theta = 0.2 *((after - before).toSec());
        if(success_detect){
           area_sort[room_index] = result;
            goal_number ++;
            success_detect = 0;
            break;
        }
    }

    goal[goal_number].target_pose.header.frame_id = "map";
    goal[goal_number].target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal[goal_number]);
    ROS_INFO("Send NO. %d Goal !!! room %d", goal_number,room_index );
    while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
        ros::spinOnce();
        if(success_detect){
            room_index = tf_weizhi();
            area_sort[room_index] = result;
            if(past_room_index != room_index){
                goal_number ++;
                break;
               }
           }
     }
    if(success_detect && room_index == 3){
        goal_number = 2;
        success_detect =0;
    }
    else if(success_detect && room_index == 2){
        goal_number = 2;
        success_detect = 0;
    }
    else{
        ROS_INFO("The NO. %d Goal achieved success !!!", goal_number);
        ros::Duration(0.1).sleep();
        start_detect.data = 4;                           //到达目标点后，识别位置1
        detect_pub.publish(start_detect);                //发布开启识别
        ROS_INFO("successfully send!here in room_index<=1");
        ros::Duration(0.1).sleep();
        while(detect_flag == 0){
            twist.linear.x = 0.2;
            twist.angular.z = 0;
            cmd_pub.publish(twist);   
            ros::spinOnce();
        } 
        detect_flag = 0;
        if(success_detect){
            area_sort[2] = result;
            success_detect = 0;
        }
        goal_number ++;
    }
    goal[goal_number].target_pose.header.frame_id = "map";
    goal[goal_number].target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal[goal_number]);
    ros::Duration(3.0).sleep();
    start_detect.data = 2;                           //到达目标点后，识别位置1
    detect_pub.publish(start_detect);                //发布开启识别
    ROS_INFO("successfully send!here in room_index<=1");
    while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
        ros::spinOnce();
        if(success_detect){
            room_index = tf_weizhi();
            area_sort[room_index] = result;
            goal_number = 4;
            break;
        }
     }
    if(success_detect == 0){
        goal_number = 3;
        goal[goal_number].target_pose.header.frame_id = "map";
        goal[goal_number].target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(goal[goal_number]);
        while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
            ros::spinOnce();
            if(success_detect){
                room_index = tf_weizhi();
                area_sort[room_index] = result;
                goal_number  = 4;
                //success_detect = 0;
                break;
            }
        }
        if(success_detect == 0){
            ROS_INFO("The NO. %d Goal achieved success !!!", goal_number);
            ros::Duration(0.1).sleep();
            start_detect.data = 4;                           //到达目标点后，识别位置1
            detect_pub.publish(start_detect);                //发布开启识别
            ROS_INFO("successfully send!here in room_index<=1");
            ros::Duration(0.1).sleep();
            while(detect_flag == 0){
                twist.linear.x = 0.2;
                twist.angular.z = 0;
                cmd_pub.publish(twist);   
                ros::spinOnce();
            } 
            detect_flag = 0;
            if(success_detect){
                area_sort[1] = result;
                success_detect = 0;
            }
        }
    }
    success_detect = 0;
    if(all_detect()){
         before = ros::Time::now();
        after = ros::Time::now();
        while(theta< 1.54){
            ros::spinOnce();
            twist.angular.z = 1.5;
            cmd_pub.publish(twist);
            after = ros::Time::now();
            theta = 1.5 *((after - before).toSec());
        }
        goal_number = 13;
    }
    else{
        goal[goal_number].target_pose.header.frame_id = "map";
        goal[goal_number].target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(goal[goal_number]);
        while(tf_weizhi() != 0);
        start_detect.data = 2;                           //到达目标点后，识别位置1
        detect_pub.publish(start_detect);                //发布开启识别
        while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
            ros::spinOnce();
            if(success_detect){
                room_index = tf_weizhi();
                area_sort[room_index] = result;
                goal_number = 13;
                break;
            }
        }
        if(success_detect == 0){
            goal_number = 3;
            goal[goal_number].target_pose.header.frame_id = "map";
            goal[goal_number].target_pose.header.stamp = ros::Time::now();
            ac.sendGoal(goal[goal_number]);
            while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
                ros::spinOnce();
                if(success_detect){
                    room_index = tf_weizhi();
                    area_sort[room_index] = result;
                    goal_number  = 13;
                    //success_detect = 0;
                    break;
                }
            }
            if(success_detect == 0){
                ROS_INFO("The NO. %d Goal achieved success !!!", goal_number);
                ros::Duration(0.1).sleep();
                start_detect.data = 4;                           //到达目标点后，识别位置1
                detect_pub.publish(start_detect);                //发布开启识别
                ROS_INFO("successfully send!here in room_index<=1");
                ros::Duration(0.1).sleep();
                while(detect_flag == 0){
                    twist.linear.x = 0.2;
                    twist.angular.z = 0;
                    cmd_pub.publish(twist);   
                    ros::spinOnce();
                } 
                detect_flag = 0;
                if(success_detect){
                    area_sort[0] = result;
                    success_detect = 0;
                }
            }
        }
    }
    success_detect = 0;

    return 0;
}