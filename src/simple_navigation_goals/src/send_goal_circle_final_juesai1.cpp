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
int detect_num = 0;
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
int detect_sum_flag = 0;
int area_sort[4] = {4,4,4,4};   //各房间种物类型BCDE
int detect_s[4] = {0,0,0,0};
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

int detect_flag_(){
    if(detect_s[0]>=2 || detect_s[1]>=2 || detect_s[2]>= 2|| detect_s[3]>= 2){
        return 1;
    }
    else{
        return 0;
    }
}
void detect_clean(){
    detect_s[0] = 0;
    detect_s[1] = 0;
    detect_s[2] = 0;
    detect_s[3] = 0;
}
void poseCallback(const nav_msgs::Odometry &p_msg){
    px = p_msg.pose.pose.position.x;
}

void area_msg_cb(const std_msgs::Int8::ConstPtr& r_p )
{
    int result = r_p->data;
    detect_flag = 1;
    detect_sum_flag = detect_flag_();
    if (result == 10 || detect_sum_flag == 0){//接受到10表示是空板 或者检测正确率不高 明天记得改已改
        if(result != 10){
            detect_s[result] += 1;
        }
        success_detect = 0;
        ROS_INFO("detect result  : %d !",result); 
    }
    else{
        ROS_INFO("detect result success : %d !",result);
        //detect_sum[result] += 1;
        if(detect_s[0] >= 2){
            success_detect = 1;
            area_sort[room_index]=0;
            detect_clean();
        }
        else if(detect_s[1] >= 2){
            success_detect = 1;
            area_sort[room_index]=1;
            detect_clean();
        }
        else if(detect_s[2] >= 2){
            success_detect = 1;
            area_sort[room_index]=2;
            detect_clean();
        }
        if(detect_s[3] >= 2){
            success_detect = 1;
            area_sort[room_index]=3;
            detect_clean();
        }
        room_index++;
    }
}

void area_msg_cb_f(const std_msgs::String::ConstPtr& r_p ){
    string s = r_p->data.c_str();
    detect_flag = 1;
    ROS_INFO("F area hear :  %c%c%c%c", s[0],s[1],s[2],s[3]);
    ROS_INFO("goal_number = %d",goal_number);
    ROS_INFO("state = %d",state);
    if(goal_number == 10){
        sum = 0;
        if(s == "10"){
            ROS_INFO("NONE DETECT");
            state = 5;
            goal_number = 11;
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
                    goal_number = 13;//jiance zhongxin de dian
                }
                else{
                    ROS_INFO("HAVE DETECTED ONE BOARD IN A AREA!");
                    state = 1;
                    goal_number = 11;//jiance gudingban de dian
                }
            }
            if(sum == 1){
                if(s[4]-48 == 1){
                    ROS_INFO("HAVE DETECTED THE FIXXED BOARD!!");
                    state = 4;
                    goal_number = 13;//jiance zhongxin de dian
                }
                else{
                    ROS_INFO("HAVE DETECTED ONE BOARD IN A AREA!");
                    state = 3;
                    goal_number = 11;//jiance gudingban de dian
                }
            }
        }
    }
    else{
        //ROS_INFO("goal_number = %d",goal_number);
        if(s != "10")
            sum += s[3]-48;
        if(goal_number<17){//jiance shangbanqu
            if(state == 1){
                if(sum != 3 && goal_number == 12){
                    sum = 3;
                }
                if(sum != 3 ){ //xiayige gudingban
                    goal_number = 12;//xia yi ge gu ding ban
                }
                else{
                    middle_detect = 1;
                }
            }
            if(state == 2){
                if(sum != 3 && goal_number == 16){//zai suo you suijiban zhong
                    sum = 3 ;
                }
                if(sum != 3){//zai suo you suijiban zhong
                    goal_number ++;//xia yi ge suiji ban
                }
                else{
                    middle_detect = 1;
                }
            }
            if(state == 3 ){
                if(sum == 1 && goal_number == 12){ //liang ge gudingban dou meishi biedao
                    sum = 2;
                    goal_number = 13;                           //diyige suijidian 
                }
                else if(sum == 2 && goal_number == 16){//zai zui hou yige suijidian qie meiyou wanquanshibie
                    sum = 3 ;
                }
                else if(sum == 2 && goal_number <16 && goal_number>=13){//zai suiji ban
                    ROS_INFO("here!!!");
                    goal_number ++;//xia yige suijiban
                }
                else if(sum == 2 && goal_number <= 12){ //zai gudingban
                    ROS_INFO("here in sum == 2 && goal_number<=11");
                    goal_number = 13;//suijiban diyige weizhi
                    ROS_INFO("goal_number = %d",goal_number);
                }
                else if(sum == 1 ){
                    goal_number = 12;//gudingban xiayige weizhi
                }
                if(sum == 3){
                    middle_detect = 1;
                }
            }
            if (state == 4){
                if(sum <= 2 && goal_number == 16){//zai zui hou yige suijidian qie meiyou wanquanshibie
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
                if(sum == 0 && goal_number == 12){////liang ge gudingban dou meishi biedao
                    sum = 1;
                    goal_number = 12;//diyige suijiban
                }
                else if(sum <= 2 && goal_number == 16){//zai zui hou yige suijidian qie meiyou wanquanshibie
                    sum = 3;
                }
                else if(sum == 0){
                    goal_number = 12;//xia yi ge  gudingban
                }
                else if(sum == 1 && goal_number<=12){
                    goal_number = 13;
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

int shujuchuli(){
    if(detect_num == 3){
        sum = area_sort[0]+area_sort[1]+area_sort[2]+area_sort[3] - 4;
        if(area_sort[0]==4){
            area_sort[0] = 6 -sum;
        }
        else if(area_sort[1]==4){
            area_sort[1] = 6 -sum;
        }
        else if(area_sort[2]==4){
            area_sort[2] = 6 -sum;
        }
        else if(area_sort[3]==4){
            area_sort[3] = 6 -sum;
        }
        return 1;
    }
    else{
        return 0;
    }
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

    move_base_msgs::MoveBaseGoal goal[19];
    std_msgs::Int8 start_detect;           //控制识别启动，1时启动
    geometry_msgs::Twist twist;

    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::Config config;
    dynamic_reconfigure::BoolParameter bool_param;

    tf::TransformListener listener;

    ros::Time before = ros::Time::now();
    ros::Time after = ros::Time::now();
    //B进
goal[0].target_pose.pose.position.x =4.882;
	goal[0].target_pose.pose.position.y = -3.417;
	goal[0].target_pose.pose.orientation.z =-0.7;  
	goal[0].target_pose.pose.orientation.w = 0.7;
//B向右
goal[1].target_pose.pose.position.x =4.364;
	goal[1].target_pose.pose.position.y = -4.994;
	goal[1].target_pose.pose.orientation.z =0.950;  
	goal[1].target_pose.pose.orientation.w = -0.313;
//B右
goal[2].target_pose.pose.position.x =4.364;
	goal[2].target_pose.pose.position.y = -4.994;
	goal[2].target_pose.pose.orientation.z =0.950;  
	goal[2].target_pose.pose.orientation.w = -0.313;
//C进
goal[3].target_pose.pose.position.x =4.877;
	goal[3].target_pose.pose.position.y = -1.983;
	goal[3].target_pose.pose.orientation.z =0.7;  
	goal[3].target_pose.pose.orientation.w = 0.7;
//C中
goal[4].target_pose.pose.position.x =4.744;
	goal[4].target_pose.pose.position.y = -0.169;
	goal[4].target_pose.pose.orientation.z =-0.7;  
	goal[4].target_pose.pose.orientation.w = 0.700;
//C右
goal[5].target_pose.pose.position.x =4.538;
	goal[5].target_pose.pose.position.y = -0.438;
	goal[5].target_pose.pose.orientation.z =0.903;  
	goal[5].target_pose.pose.orientation.w = 0.430;
    //D进
    goal[6].target_pose.pose.position.x =2.877;
goal[6].target_pose.pose.position.y = -3.324;
goal[6].target_pose.pose.orientation.z =-0.7;  
goal[6].target_pose.pose.orientation.w = 0.7;
    //E进
    goal[7].target_pose.pose.position.x =2.829;
goal[7].target_pose.pose.position.y = -2.365;
goal[7].target_pose.pose.orientation.z =0.7;  
goal[7].target_pose.pose.orientation.w = 0.7;
    //78
    goal[8].target_pose.pose.position.x =2.842;
goal[8].target_pose.pose.position.y = -0.941;
goal[8].target_pose.pose.orientation.z =-0.441;  
goal[8].target_pose.pose.orientation.w = 0.897;
    //E固
    goal[9].target_pose.pose.position.x =2.829;
goal[9].target_pose.pose.position.y = -0.885;
goal[9].target_pose.pose.orientation.z = 0.525;  
goal[9].target_pose.pose.orientation.w = 0.851;
    
    //F进去后
    goal[10].target_pose.pose.position.x =0.925;
	goal[10].target_pose.pose.position.y = -2.823;
	goal[10].target_pose.pose.orientation.z =-0.7;  
	goal[10].target_pose.pose.orientation.w = 0.7;
    //F左
    goal[11].target_pose.pose.position.x =0.943;
	goal[11].target_pose.pose.position.y = -4.950;
	goal[11].target_pose.pose.orientation.z =-0.255;  
	goal[11].target_pose.pose.orientation.w = 0.967;
    //F右
    goal[12].target_pose.pose.position.x =0.943;
	goal[12].target_pose.pose.position.y = -4.950;
	goal[12].target_pose.pose.orientation.z =0.967;  
	goal[12].target_pose.pose.orientation.w = -0.255;
    //F中1
    goal[13].target_pose.pose.position.x =0.974;
	goal[13].target_pose.pose.position.y = -3.988;
	goal[13].target_pose.pose.orientation.z =0.250;  
	goal[13].target_pose.pose.orientation.w = 0.968;
    //F中2
    goal[14].target_pose.pose.position.x =0.973;
	goal[14].target_pose.pose.position.y =-4.045;
	goal[14].target_pose.pose.orientation.z =0.937;  
	goal[14].target_pose.pose.orientation.w = -0.350;
    //F中3
    goal[15].target_pose.pose.position.x =0.973;
	goal[15].target_pose.pose.position.y = -4.045;
	goal[15].target_pose.pose.orientation.z =-0.481;  
	goal[15].target_pose.pose.orientation.w =0.877;
    //F中4
    goal[16].target_pose.pose.position.x =0.973;
	goal[16].target_pose.pose.position.y = -4.045;
	goal[16].target_pose.pose.orientation.z =-0.481;  
	goal[16].target_pose.pose.orientation.w =0.877;
    //F下
    goal[17].target_pose.pose.position.x =0.939;
	goal[17].target_pose.pose.position.y = -3.123;///////
	goal[17].target_pose.pose.orientation.z =0.7;  
	goal[17].target_pose.pose.orientation.w = 0.7;

    //Fchu!
    goal[18].target_pose.pose.position.x =0.05;
	goal[18].target_pose.pose.position.y = 0.080;
	goal[18].target_pose.pose.orientation.z =1.0;  
	goal[18].target_pose.pose.orientation.w = 0;


    start_detect.data = 1;                                           //到达目标点后，识别位置1
    detect_pub.publish(start_detect);                //发布开启识别/
         
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
    detect_flag  = 0;
    
    while(room_index != 5)
    {
        while(room_index<=1){
            detect_flag = 0;
            goal[goal_number].target_pose.header.frame_id = "map";
            goal[goal_number].target_pose.header.stamp = ros::Time::now();
            ac.sendGoal(goal[goal_number]);
            ROS_INFO("Send NO. %d Goal !!!!", goal_number );
            ac.waitForResult();

            //若平移：参数 distance = 0.6    y_vel = 1.0    向左  可以多给点距离
            //若平移：参数 distance = 0.6    y_vel = -1.0   向右
            //若平移：参数 theta = 1.4   z_theta = -3.14    顺时针90
            //若平移：参数 theta = 1.4    y_vel = 3。14   逆时针90
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("The base reach area -- %d  and  shot",room_index);
                // ros::Duration(0.1).sleep();
                if(goal_number == 0 || goal_number ==3){
                    ROS_INFO("The NO. %d Goal achieved success !!!", goal_number);
                    ros::Duration(0.1).sleep();
                    start_detect.data = 4;                           //到达目标点后，识别位置1
                    detect_pub.publish(start_detect);                //发布开启识别
                    ROS_INFO("successfully send!here in room_index<=1");
                    ros::Duration(0.3).sleep();                          //%%%
                    while(detect_flag == 0){                          ////
                        twist.linear.x = 0.2;      
                        twist.angular.z = 0;
                        twist.linear.y = 0;
                        cmd_pub.publish(twist);   
                        ros::spinOnce();
                    } 
                    detect_flag = 0;

                    if(success_detect){
                        goal_number += 3;   
                        detect_num++;
                        before = ros::Time::now();
                        after = ros::Time::now();
                        while(distance_17 < 3.14){
                            twist.linear.x = 0;
                            twist.linear.y = 0;
                            twist.angular.z = 4.0;
                            cmd_pub.publish(twist);
                            after = ros::Time::now();
                            distance_17 = 4.0 *((after - before).toSec());
                        }  
                        distance_17 = 0;
                        success_detect = 0;
                    }
                    else{
                        goal_number ++;
                    }
                }
                else{
                    ros::Duration(0.1).sleep();
                    start_detect.data = 4;                           //到达目标点后，识别位置1
                    detect_pub.publish(start_detect);                //发布开启识别
                    ROS_INFO("successfully send!here in room_index<=1");
                    ros::Duration(0.3).sleep();                          //%%%
                    while(detect_flag == 0){
                        ros::spinOnce();
                    }
                    detect_flag = 0;
                     if(success_detect){
                        goal_number = room_index * 3;
                        detect_num ++;
                     }
                     else{
                        if(goal_number == 2 || goal_number == 5){
                            room_index ++;
                        }
                        goal_number ++;
                     }
                }
                success_detect = 0;
            }
        }
        ROS_INFO("!!!!_____>%d",room_index);
        detect_flag = 0;
        goal[6].target_pose.header.frame_id = "map";
        goal[6].target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(goal[6]);
        while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
        ROS_INFO("here");
        ros::Duration(0.1).sleep();
        start_detect.data = 4;                                           //到达目标点后，识别位置1
        detect_pub.publish(start_detect);                //发布开启识别
        ROS_INFO("successfully send!!!!DDD"); 
        ros::Duration(0.4).sleep();                           //%%%
        ROS_INFO("detect_flag = %d",detect_flag);
        while(detect_flag == 0){
             twist.linear.x = 0.2;
             twist.linear.y = 0;
            twist.angular.z = 0.0;
            cmd_pub.publish(twist);
            ros::spinOnce();
        } 
        detect_flag = 0;

        before = ros::Time::now();
        after = ros::Time::now();          
        while(distance_17 < 3.25){
            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.angular.z = 4.0;
            cmd_pub.publish(twist);
            after = ros::Time::now();
            distance_17 = 4.0 *((after - before).toSec());
        }  
        distance_17 = 0; 

        if(success_detect){
            detect_num ++;
            if(detect_num == 3){
                goal_number = 10;
                room_index = 4;
            }
            else{
                room_index = 3;
                goal_number = 7;
            }
        }
        else{
            room_index = 3;
            goal_number = 7;
        }
        while(room_index == 3){
            success_detect = 0;
            ROS_INFO("HERE IN WHILE(ROOM_INDEX==3)_");
            goal[goal_number].target_pose.header.frame_id = "map";
            goal[goal_number].target_pose.header.stamp = ros::Time::now();
            ac.sendGoal(goal[goal_number]);
            ac.waitForResult();
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ros::Duration(0.1).sleep();
                start_detect.data =4;                                           //到达目标点后，识别位置1
                detect_pub.publish(start_detect);                //发布开启识别
                ros::Duration(0.3).sleep();                          //%%%
                ROS_INFO("successfully send!EEE");            
                while(detect_flag == 0){
                    ros::spinOnce();
                } 
                detect_flag = 0;
                if(goal_number<9 && room_index == 3 && success_detect == 0){
                    goal_number ++;
                }
                else if(room_index == 3 && goal_number == 9 && success_detect == 0 ){          //goal_number = 4并且此时仍未识别到
                    room_index ++;
                    ROS_INFO("here in else if %d",room_index);
                }
                else if(success_detect){
                    ROS_INFO("successfully to the end of the four rooms!");
                    detect_num ++;
                }
            }
        }
    flag = 1;
    goal[10].target_pose.header.frame_id = "map";
    goal[10].target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal[10]);
    while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
    goal_number = 10;
    room_index = 4;
    while(room_index == 4){
            detect_flag = 0;
            if(goal_number>= 14 && goal_number<=16){
                before = ros::Time::now();
                after = ros::Time::now();
                while(distance_17< 1.40){
                    twist.linear.x = 0;
                    twist.linear.y = 0;
                    twist.angular.z = 3.14;
                    cmd_pub.publish(twist);
                    after = ros::Time::now();
                    distance_17 = 3.14 *((after - before).toSec());
                }  
                distance_17 = 0;
            }
            else if(goal_number == 12){
                before = ros::Time::now();
                after = ros::Time::now();
                while(distance_17< 2.25){
                    twist.linear.x = 0;
                    twist.linear.y = 0;
                    twist.angular.z = -3.14;
                    cmd_pub.publish(twist);
                    after = ros::Time::now();
                    distance_17 = 3.14 *((after - before).toSec());
                }  
                distance_17 = 0;
            }
            else if(goal_number != 10){
                goal[goal_number].target_pose.header.frame_id = "map";
                goal[goal_number].target_pose.header.stamp = ros::Time::now();
                ac.sendGoal(goal[goal_number]);
                while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
            }
            if(goal_number >= 17){
                ros::Duration(0.1).sleep();
                start_detect.data = 7;                                           //到达目标点后，识别
                detect_pub.publish(start_detect);                //发布开启识别
                ros::Duration(0.68).sleep();
                ROS_INFO("successfully send!!!LAST"); 
                detect_flag = 0;
                room_index = 5;
                break;
            }
            else if(goal_number>=13 && goal_number<=16){
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
            else if(goal_number<13){
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
                if(goal_number == 10){
                    ROS_INFO("%f",distance_17);
                    distance_17 = 0;
                    before = ros::Time::now();
                    after = ros::Time::now();
                    while(distance_17< 3.14){
                        twist.linear.x = 0;
                        twist.linear.y = 0;
                        twist.angular.z = 3.14;
                        cmd_pub.publish(twist);
                        after = ros::Time::now();
                        distance_17 = 3.14 *((after - before).toSec());
                    }  
                    distance_17 = 0;
                }
                ROS_INFO("SHANGBANQU DETECT ALL!!");
                goal_number = 17;
            }
        }
    }


    goal[18].target_pose.header.frame_id = "map";
    goal[18].target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal[18]);
    while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
    before = ros::Time::now();
    after = ros::Time::now();
    while(distance_17< 0.06){
        twist.linear.x = 0.6;
        twist.linear.y = 0;
        twist.angular.z = 0;
        cmd_pub.publish(twist);
        after = ros::Time::now();
        distance_17 = 0.6 *((after - before).toSec());
    }  
    distance_17 = 0;
    //     ros::Duration(2).sleep();
    //     dynamic_reconfigure::Reconfigure srv3;
    //     dynamic_reconfigure::Config config3;
    //     dynamic_reconfigure::BoolParameter bool_param;

    //     client = n.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/local_costmap/obstacle_layer/set_parameters");
    //     bool_param.name = "enabled";
    //     bool_param.value = false;
    //     config3.bools.push_back(bool_param);

    //     srv3.request.config = config3;
    //     if(client.call(srv3)){
    //         ROS_INFO("SUCCESS!!!!");
    //     }

    //     goal[20]target_pose.header.frame_id = "map";
    //     goal[20].target_pose.header.stamp = ros::Time::now();
    //     ac.sendGoal(goal[20]);
    //     while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);
    // }
    detect_flag = 0;
    ROS_INFO("HERE");
    start_detect.data = 8;                                           //到达目标点后，识别
    detect_pub.publish(start_detect);                //发布开启识别
    ROS_INFO("successfully send!!!LAST"); 
    while(detect_flag == 0){
        ros::spinOnce();
    }
    if(room_index==5)
    {
        flag = shujuchuli();
        if(flag){
            ROS_INFO("area_sort->result : %d %d %d %d",area_sort[0],area_sort[1],area_sort[2],area_sort[3]);
        }
        else{
            ROS_INFO("FAIL DETECT!!!");
        }
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

