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
#include <dynamic_reconfigure/Config.h>
#include <tf/transform_listener.h>
using namespace std;

//int num = 0;
int send_num = 1;
bool a;
uint8_t pic_sum = 0;
uint8_t expect_time = 60;
int sum = 0;
int start_flag = 0;
int room_index=0;       //记录当前房间号
int awake_flag=0;      //语音唤醒标志位
int detect_flag=0;     //目标检测标志位
int middle_detect = 0;
int state = 0;
int area_sort[4] = {0,0,0,0};   //各房间种物类型BCDE
uint8_t success_detect = 0; //判断识别准确率，若大于      ，返回1，到下一房间。反之，到下一点。
uint8_t goal_number = 0;
int corn_num  = 0;
int cucumber_num = 0;
int watermelon_num = 0;

double x;
double y;
uint8_t flag = 1;
uint8_t flag_send = 1;

bool is_start = true;
double distance_17 = 0;

char *voice[4][4]={{"aplay ~/ucar_car/src/simple_navigation_goals/voice/B_corn.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/B_cucumber.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/B_rice.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/B_wheat.wav"},
                    {"aplay ~/ucar_car/src/simple_navigation_goals/voice/C_corn.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/C_cucumber.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/C_rice.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/C_wheat.wav"},
                    {"aplay ~/ucar_car/src/simple_navigation_goals/voice/D_corn.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/D_cucumber.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/D_rice.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/D_wheat.wav"},
                    {"aplay ~/ucar_car/src/simple_navigation_goals/voice/E_corn.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/E_cucumber.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/E_rice.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/E_wheat.wav"}};
char *num[7] = {"aplay ~/ucar_car/src/simple_navigation_goals/voice/1.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/2.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/3.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/4.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/5.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/6.wav","aplay ~/ucar_car/src/simple_navigation_goals/voice/7.wav"};
int count[4]={0};

double dis_long = 0.9;
double ix,px;

void poseCallback(const nav_msgs::Odometry &p_msg){
    px = p_msg.pose.pose.position.x;
}

void area_msg_cb(const std_msgs::Int8::ConstPtr& r_p )
{
    int result = r_p->data;
    detect_flag = 1;
    if (result == 10){//接受到10表示是空板 或者检测正确率不高 明天记得改已改
        success_detect = 0;
        ROS_INFO("detect result  : %d !",result); 
    }
    else{
        ROS_INFO("detect result : %d !",result);
        success_detect = 1;
        area_sort[room_index]=result;
        room_index++;
    }
}

void area_msg_cb_f(const std_msgs::String::ConstPtr& r_p ){
    string s = r_p->data.c_str();
    detect_flag = 1;
    ROS_INFO("F area hear  :  %c%c%c%c", s[0],s[1],s[2],s[3]);
    ROS_INFO("goal_number = %d",goal_number);
    ROS_INFO("state = %d",state);
    if(goal_number == 9){
        sum = 0;
        if(s == "10"){
            ROS_INFO("NONE DETECT");
            state = 5;
            goal_number = 10;
        }
        else{
            sum += s[3]-48;
            corn_num += (s[0]-48);
            cucumber_num += (s[1]-48);
            watermelon_num += (s[2]-48);
            if(sum == 3){
                middle_detect = 1;
                state = 5;
            }
            if(sum == 2){
                if(s[4]-48 == 1){
                    ROS_INFO("HAVE DETECTED THE FIXXED BOARD!!");
                    state = 2;
                    goal_number = 12;//jiance zhongxin de dian
                }
                else{
                    ROS_INFO("HAVE DETECTED ONE BOARD IN A AREA!");
                    state = 1;
                    goal_number = 10;//jiance gudingban de dian
                }
            }
            if(sum == 1){
                if(s[4]-48 == 1){
                    ROS_INFO("HAVE DETECTED THE FIXXED BOARD!!");
                    state = 4;
                    goal_number = 12;//jiance zhongxin de dian
                }
                else{
                    ROS_INFO("HAVE DETECTED ONE BOARD IN A AREA!");
                    state = 3;
                    goal_number = 10;//jiance gudingban de dian
                }
            }
        }
    }
    else{
        //ROS_INFO("goal_number = %d",goal_number);
        if(s != "10")
            sum += s[3]-48;
        if(goal_number<16){//jiance shangbanqu
            if(state == 1){
                if(sum != 3 && goal_number == 11){
                    sum = 3;
                }
                if(sum != 3 ){ //xiayige gudingban
                    goal_number = 11;//xia yi ge gu ding ban
                }
                else{
                    middle_detect = 1;
                }
            }
            if(state == 2){
                if(sum != 3 && goal_number == 15){//zai suo you suijiban zhong
                    sum =3 ;
                }
                if(sum != 3){//zai suo you suijiban zhong
                    goal_number ++;//xia yi ge suiji ban
                }
                else{
                    middle_detect = 1;
                }
            }
            if(state == 3 ){
                if(sum == 1 && goal_number == 11){ //liang ge gudingban dou meishi biedao
                    sum = 2;
                    goal_number = 12;                           //diyige suijidian 
                }
                else if(sum == 2 && goal_number == 15){//zai zui hou yige suijidian qie meiyou wanquanshibie
                    sum = 3 ;
                }
                else if(sum == 2 && goal_number <15 && goal_number>=12){//zai suiji ban
                    ROS_INFO("here!!!");
                    goal_number ++;//xia yige suijiban
                }
                else if(sum == 2 && goal_number <= 11){ //zai gudingban
                    ROS_INFO("here in sum == 2 && goal_number<=11");
                    goal_number = 12;//suijiban diyige weizhi
                    ROS_INFO("goal_number = %d",goal_number);
                }
                else if(sum == 1 ){
                    goal_number = 11;//gudingban xiayige weizhi
                }
                if(sum == 3){
                    middle_detect = 1;
                }
            }
            if (state == 4){
                if(sum <= 2 && goal_number == 15){//zai zui hou yige suijidian qie meiyou wanquanshibie
                    sum = 3;
                }
                else if(sum <=2){
                    goal_number ++;//buzaizuihouyige qie meiyouwanquanshibie
                }
                if(sum == 3){
                    middle_detect = 1;
                }
            }
            if(state == 5){
                if(sum == 0 && goal_number == 11){////liang ge gudingban dou meishi biedao
                    sum = 1;
                    goal_number = 12;//diyige suijiban
                }
                else if(sum <= 2 && goal_number == 15){//zai zui hou yige suijidian qie meiyou wanquanshibie
                    sum = 3;
                }
                else if(sum == 0){
                    goal_number = 11;//xia yi ge  gudingban
                }
                else if(sum == 1 && goal_number<=11){
                    goal_number = 12;
                }
                else if(sum<=2){
                    goal_number ++;//xiayigesuijiban
                }
                if(sum == 3){
                    middle_detect = 1;
                }
            }
        }
        ROS_INFO("sum = %d",sum);
        if(s != "10"){
            corn_num += (s[0]-48);
            cucumber_num += (s[1]-48);
            watermelon_num += (s[2]-48);
        }
    }
}

void mic_awake_cb(const std_msgs::Int8::ConstPtr& a_p)
{
    awake_flag = a_p->data;             //传递a_p值
    ROS_INFO("awake flag is set");
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int main(int argc, char** argv){
    ros::init(argc, argv, "send_goals_circle");
    ros::NodeHandle n;
    ros::Publisher detect_pub = n.advertise<std_msgs::Int8>("/start_detect",1);//publisher
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    ros::Subscriber awake_sub = n.subscribe<std_msgs::Int8>("awake_flag",1,mic_awake_cb);  //语音唤醒订阅
    ros::Subscriber room_sub = n.subscribe<std_msgs::Int8>("/area_msg",3,area_msg_cb); //房间类型订阅
    ros::Subscriber room_sub_f = n.subscribe<std_msgs::String>("/area_msgf",1,area_msg_cb_f);
    ros::Subscriber pose_sub = n.subscribe("/odom",10,poseCallback);
    ros::ServiceClient client ;
    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer( ros::Duration( 5.0 ) )){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal[20];
    std_msgs::Int8 start_detect;           //控制识别启动，1时启动
    geometry_msgs::Twist twist;

    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::Config config;
    dynamic_reconfigure::BoolParameter bool_param;

    tf::TransformListener listener;

    ros::Time before = ros::Time::now();
    ros::Time after = ros::Time::now();
    //B进
    goal[0].target_pose.pose.position.x =4.819;
	goal[0].target_pose.pose.position.y = -3.259;
	goal[0].target_pose.pose.orientation.z =-0.7;  
	goal[0].target_pose.pose.orientation.w = 0.7;
    //B中
    goal[1].target_pose.pose.position.x =4.602;
	goal[1].target_pose.pose.position.y = -4.929;
	goal[1].target_pose.pose.orientation.z = 0.7;  
	goal[1].target_pose.pose.orientation.w =  0.7;
    //C进
    goal[2].target_pose.pose.position.x =4.943;
	goal[2].target_pose.pose.position.y = -1.879;
	goal[2].target_pose.pose.orientation.z =0.7;  
	goal[2].target_pose.pose.orientation.w = 0.7;
    //C中
    goal[3].target_pose.pose.position.x =4.666;
	goal[3].target_pose.pose.position.y = -0.182;
	goal[3].target_pose.pose.orientation.z =-0.7;  
	goal[3].target_pose.pose.orientation.w = 0.700;
    //D进
    goal[4].target_pose.pose.position.x =2.885;
    goal[4].target_pose.pose.position.y = -3.337;
    goal[4].target_pose.pose.orientation.z =-0.7;  
    goal[4].target_pose.pose.orientation.w = 0.7;
    //58
    goal[5].target_pose.pose.position.x =2.999;
    goal[5].target_pose.pose.position.y = -1.339;
    goal[5].target_pose.pose.orientation.z =-0.457;  
    goal[5].target_pose.pose.orientation.w = 0.890;
    //78
    goal[6].target_pose.pose.position.x= 2.949;
    goal[6].target_pose.pose.position.y = -1.559;
    goal[6].target_pose.pose.orientation.z =0.947;  
    goal[6].target_pose.pose.orientation.w = -0.322;
    //E右
    goal[7].target_pose.pose.position.x =2.931;
    goal[7].target_pose.pose.position.y = -0.368;
    goal[7].target_pose.pose.orientation.z =0.365;  
    goal[7].target_pose.pose.orientation.w = 0.931;
    //F进去前
    goal[8].target_pose.pose.position.x =0.917;
    goal[8].target_pose.pose.position.y = -0.117;
    goal[8].target_pose.pose.orientation.z =-0.725;  
    goal[8].target_pose.pose.orientation.w = 0.689;
    //F进去后
    goal[9].target_pose.pose.position.x =0.891;
	goal[9].target_pose.pose.position.y =-3.005;
	goal[9].target_pose.pose.orientation.z =-0.700;  
	goal[9].target_pose.pose.orientation.w = 0.700;
    //F左
    goal[10].target_pose.pose.position.x =0.941;
	goal[10].target_pose.pose.position.y = -4.803;
	goal[10].target_pose.pose.orientation.z =-0.273;  
	goal[10].target_pose.pose.orientation.w = 0.962;
    //F右
    goal[11].target_pose.pose.position.x =0.941;
	goal[11].target_pose.pose.position.y = -4.803;
	goal[11].target_pose.pose.orientation.z =0.962;  
	goal[11].target_pose.pose.orientation.w = -0.273;
    //F中1
    goal[12].target_pose.pose.position.x =0.910;
	goal[12].target_pose.pose.position.y = -3.949;
	goal[12].target_pose.pose.orientation.z = 0.211;  
	goal[12].target_pose.pose.orientation.w =0.978;
    //F中2
    goal[13].target_pose.pose.position.x =0.973;
	goal[13].target_pose.pose.position.y = -4.045;
	goal[13].target_pose.pose.orientation.z = 0.860;  
	goal[13].target_pose.pose.orientation.w = 0.511;
    //F中3
    goal[14].target_pose.pose.position.x =0.973;
	goal[14].target_pose.pose.position.y =-4.045;
	goal[14].target_pose.pose.orientation.z =0.937;  
	goal[14].target_pose.pose.orientation.w = -0.350;
    //F中4
    goal[15].target_pose.pose.position.x =0.973;
	goal[15].target_pose.pose.position.y = -4.045;
	goal[15].target_pose.pose.orientation.z =-0.481;  
	goal[15].target_pose.pose.orientation.w =0.877;
    //FB
    goal[16].target_pose.pose.position.x =0.970;
	goal[16].target_pose.pose.position.y = -2.558;
	goal[16].target_pose.pose.orientation.z = 0.79;  
	goal[16].target_pose.pose.orientation.w = 0.74;
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

    ros::Time start_time = ros::Time::now();                 //初始化小车启动时间戳
    send_num++;
    detect_flag  = 0;
    
    while(room_index != 5)    // total is 12 goals
    {
        while(room_index<=1){
            detect_flag = 0;
            goal[goal_number].target_pose.header.frame_id = "map";
            goal[goal_number].target_pose.header.stamp = ros::Time::now();
            ac.sendGoal(goal[goal_number]);
            ROS_INFO("Send NO. %d Goal !!!!", goal_number );
            ac.waitForResult();
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("The base reach area -- %d  and  shot",room_index);
                ros::Duration(0.1).sleep();
                if(goal_number == 0 || goal_number ==2){
                    ROS_INFO("The NO. %d Goal achieved success !!!", goal_number);
                    ros::Duration(0.1).sleep();
                    start_detect.data = 4;                           //到达目标点后，识别位置1
                    detect_pub.publish(start_detect);                //发布开启识别
                    ROS_INFO("successfully send!here in room_index<=1");
                    ros::Duration(0.1).sleep();
                    while(detect_flag == 0){
                        ros::spinOnce();
                    } 
                    detect_flag = 0;
                    if(success_detect){
                        goal_number += 2;
                    }
                    else{
                        goal_number ++;
                    }
                }
                else if(goal_number == 1 || goal_number == 3){
                    ROS_INFO("The NO. %d Goal achieved success !!!", goal_number);
                    ros::Duration(0.1).sleep();
                    start_detect.data = 4;                           //到达目标点后，识别位置1
                    detect_pub.publish(start_detect);                //发布开启识别
                    ROS_INFO("successfully send!here in room_index<=1");
                    ros::Duration(0.1).sleep();
                    while(detect_flag == 0){
                        ros::spinOnce();
                    } 
                    detect_flag = 0;
                    if(success_detect){
                        goal_number += 1;
                    }
                    else{
                        start_detect.data = 4;                           //到达目标点后，识别位置1
                        detect_pub.publish(start_detect);                //发布开启识别
                        ROS_INFO("successfully send!here in room_index<=1");
                        ros::Duration(0.1).sleep();
                        before = ros::Time::now();
                        after = ros::Time::now();
                        while(!success_detect && (after.sec-before.sec) <= expect_time){
                            ros::spinOnce();
                            after = ros::Time::now();
                            twist.angular.z = 0.8;
                            cmd_pub.publish(twist);
                        }
                        if(success_detect){
                            goal_number ++;
                        }
                        else{
                            ROS_INFO("FAIL DETECT!!! %d",room_index);
                            room_index ++;
                            goal_number ++;
                        }
                    }
                }
                success_detect = 0;
            }
        }
        ROS_INFO("!!!!_____>%d",room_index);
        detect_flag = 0;
        goal[4].target_pose.header.frame_id = "map";
        goal[4].target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(goal[4]);
        while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
        ROS_INFO("here");
        ros::Duration(0.1).sleep();
        start_detect.data = 4;                                           //到达目标点后，识别位置1
        detect_pub.publish(start_detect);                //发布开启识别
        ROS_INFO("successfully send!!!!DDD"); 
        ros::Duration(0.1).sleep();           
        while(detect_flag == 0){
            ros::spinOnce();
        } 
        detect_flag = 0;
        if(success_detect){
            room_index = 4;
            sum = area_sort[0] + area_sort[1] + area_sort[2];
            area_sort[3] = 6 - sum;
            goal_number = 8;
        }
        else{
            room_index = 3;
            goal_number = 5;
        }
        while(room_index == 3){
            success_detect = 0;
            ROS_INFO("HERE IN WHILE(ROOM_INDEX==3)_");
            goal[goal_number].target_pose.header.frame_id = "map";
            goal[goal_number].target_pose.header.stamp = ros::Time::now();
            ac.sendGoal(goal[goal_number]);
            while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
            ros::Duration(0.1).sleep();
            start_detect.data =4;                                           //到达目标点后，识别位置1
            detect_pub.publish(start_detect);                //发布开启识别
            ros::Duration(0.1).sleep();
            ROS_INFO("successfully send!EEE");            
            while(detect_flag == 0){
                ros::spinOnce();
            } 
            detect_flag = 0;
            if(goal_number<7 && room_index == 3 && success_detect == 0){
                goal_number ++;
            }
            else if(room_index == 3 && goal_number == 7 && success_detect == 0 ){          //goal_number = 4并且此时仍未识别到
                room_index ++;
                ROS_INFO("here in else if %d",room_index);
            }
            else if(success_detect){
                ROS_INFO("successfully to the end of the four rooms!");
                sum = area_sort[0]+area_sort[1]+area_sort[3];            //三个房间总共有三种累加和，分别为3，4,5，6
                if(sum == 3){
                    area_sort[2]=3;
                } 
                else if(sum == 4){
                    area_sort[2]=2;
                }
                else if(sum == 5){
                    area_sort[2]=1;
                }
                else {
                    area_sort[2]=0;
                }
            }
        }
    flag = 1;
    goal[9].target_pose.header.frame_id = "map";
    goal[9].target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal[9]);
    while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
        tf::StampedTransform transform;
        try {
            listener.lookupTransform("map", "base_link", ros::Time(0), transform);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        x = transform.getOrigin().x();
        y = transform.getOrigin().y();
        if( x >= 0.4 && x<= 2.8 && y>= -1.0){                                                  //chufajieduan
            if(flag){
                dynamic_reconfigure::Reconfigure srv2;
                dynamic_reconfigure::Config config2;
                dynamic_reconfigure::DoubleParameter double_param2;
                client = n.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/TebLocalPlannerROS/set_parameters");
                double_param2.name = "max_global_plan_lookahead_dist";
                double_param2.value = 0.6;
                config2.doubles.push_back(double_param2);
                srv2.request.config = config2;
                if(client.call(srv2)){
                    ROS_INFO("SUCCESS!!0.6");
                }
                else{
                    ROS_INFO("UNSECCESS!!");
                }
                dynamic_reconfigure::Reconfigure srv4;
                dynamic_reconfigure::Config config4;
                dynamic_reconfigure::DoubleParameter double_param4;
                client = n.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/TebLocalPlannerROS/set_parameters");
                double_param2.name = "weight_optimaltime";
                double_param2.value = 12;
                config2.doubles.push_back(double_param4);
                srv4.request.config = config4;
                if(client.call(srv4)){
                    ROS_INFO("SUCCESS!!!112");
                }
                else{
                    ROS_INFO("UNSECCESS!!");
                }
                flag = 0;
            }      
        }
    }
    goal_number = 9;
    room_index = 4;
    while(room_index == 4){
            detect_flag = 0;
            if(goal_number>= 13 && goal_number<=15){
                before = ros::Time::now();
                after = ros::Time::now();
                while(distance_17< 1.54){
                    twist.angular.z = 1.5;
                    cmd_pub.publish(twist);
                    after = ros::Time::now();
                    distance_17 = 1.5 *((after - before).toSec());
                }  
                distance_17 = 0;
            }
            else if(goal_number != 9){
                goal[goal_number].target_pose.header.frame_id = "map";
                goal[goal_number].target_pose.header.stamp = ros::Time::now();
                ac.sendGoal(goal[goal_number]);
                while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
            }
            if(goal_number >= 16){
                ros::Duration(0.5).sleep();
                start_detect.data = 7;                                           //到达目标点后，识别
                detect_pub.publish(start_detect);                //发布开启识别
                ROS_INFO("successfully send!!!LAST"); 
                detect_flag = 0;
                room_index = 5;
                break;
            }
            else if(goal_number>=12 && goal_number<=15){
                ros::Duration(0.1).sleep();
                start_detect.data = 6;                                           //到达目标点后，识别
                detect_pub.publish(start_detect);                //发布开启识别
                ROS_INFO("successfully send!!!!MIDDLE"); 
                ros::Duration(0.1).sleep();
                while(detect_flag == 0){
                    ros::spinOnce();
                } 
                detect_flag = 0;
            }
            else if(goal_number<12){
                ros::Duration(0.1).sleep();
                start_detect.data = 5;                                           //到达目标点后，识别
                detect_pub.publish(start_detect);                //发布开启识别
                ROS_INFO("successfully send!!!!FFF"); 
                ros::Duration(0.1).sleep();
                while(detect_flag == 0){
                    ros::spinOnce();
                } 
                detect_flag = 0;
            }           
            if(middle_detect){
                ROS_INFO("SHANGBANQU DETECT ALL!!");
                goal_number = 16;
            }
        }
        }


    goal[18].target_pose.header.frame_id = "map";
    goal[18].target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal[18]);
    while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
        ros::Duration(2).sleep();
        dynamic_reconfigure::Reconfigure srv3;
        dynamic_reconfigure::Config config3;
        dynamic_reconfigure::BoolParameter bool_param;

        client = n.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/local_costmap/obstacle_layer/set_parameters");
        bool_param.name = "enabled";
        bool_param.value = false;
        config3.bools.push_back(bool_param);

        srv3.request.config = config3;
        if(client.call(srv3)){
            ROS_INFO("SUCCESS!!!!");
        }
        ROS_INFO("%d %d",send_num,a);

        goal[19].target_pose.header.frame_id = "map";
        goal[19].target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(goal[19]);
        while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
    }
    detect_flag = 0;
    start_detect.data = 8;                                           //到达目标点后，识别
    detect_pub.publish(start_detect);                //发布开启识别
    ROS_INFO("successfully send!!!LAST"); 
    while(detect_flag == 0){
        ros::spinOnce();
    }
    if(room_index==5)
    {
        ROS_INFO("area_sort->result : %d %d %d %d",area_sort[0],area_sort[1],area_sort[2],area_sort[3]);
        ROS_INFO("corn_num:%d , cucumber_num : %d , watermelon_num : %d",corn_num,cucumber_num,watermelon_num);
        for(int j=0;j<4;j++)
        {
            system(voice[j][area_sort[j]]);
        }
        if(corn_num>cucumber_num && corn_num > watermelon_num){
            system("aplay ~/ucar_car/src/simple_navigation_goals/voice/F_corn.wav");
            system(num[corn_num-1]);
        }
        else if(cucumber_num> corn_num && cucumber_num > watermelon_num){
             system("aplay ~/ucar_car/src/simple_navigation_goals/voice/F_cucumber.wav");
             system(num[cucumber_num-1]);
        }
        else{
             system("aplay ~/ucar_car/src/simple_navigation_goals/voice/F_watermelon.wav");
             system(num[watermelon_num-1]);
        }
    }

  return 0;
}


