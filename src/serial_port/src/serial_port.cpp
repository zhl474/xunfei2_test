//http://wjwwood.io/serial/doc/1.1.0/index.html
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <serial_port/rcmsg.h>

#define IBUS    1       // 等于1定义的是富斯afhds2a IBUS协议
#define SBUS    0       // 等于2定义的是扶他爸的SBUS协议

int main(int argc, char** argv)
{
    uint16_t RC_Data[10] = {0};
    uint16_t con=0;
    serial_port::rcmsg RC_Info;
    ros::init(argc, argv, "serial_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;
    ros::Publisher rcmsg_pub = n.advertise<serial_port::rcmsg>("/RC", 1000);
    //创建一个serial类
    serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/rc_port");
    
#if IBUS
    ROS_INFO("USE IBUS");
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    sp.setStopbits(serial::stopbits_two);
#elif SBUS
    ROS_INFO("USE SBUS");
    //设置串口通信的波特率
    sp.setBaudrate(100000);
    sp.setStopbits(serial::stopbits_two);
    sp.setParity(serial::parity_even);
#endif
    //串口设置timeout
    sp.setTimeout(to);
 
    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/rc_port is opened.");
    }
    else
    {
        return -1;
    }
    
    ros::Rate loop_rate(1000);
    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t n = sp.available();
        
        if(n!=0)
        {
            uint8_t ucRxBuffer[1024];
            //读出数据，它会自动检测断点，这个很实用，我们只需要知道每次拿出来都是完整的一帧协议就行
            n = sp.read(ucRxBuffer, n);
            
            /*for(int i=0; i<n; i++)
            {
                //16进制的方式打印到屏幕
                std::cout << std::hex << (buffer[i] & 0xff) << " ";
                
            }
            std::cout << std::endl;
            //把数据发送回去
            sp.write(buffer, n);*/
#if IBUS
            if(ucRxBuffer[0]==0x20 && ucRxBuffer[1]==0x40){
                RC_Info.rssi=1;
                RC_Data[0] = ((ucRxBuffer[ 3] & 0x0f) << 8) + ucRxBuffer[ 2];
                RC_Data[1] = ((ucRxBuffer[ 5] & 0x0f) << 8) + ucRxBuffer[ 4];
                RC_Data[2] = ((ucRxBuffer[ 7] & 0x0f) << 8) + ucRxBuffer[ 6];
                RC_Data[3] = ((ucRxBuffer[ 9] & 0x0f) << 8) + ucRxBuffer[ 8];
                RC_Data[4] = ((ucRxBuffer[11] & 0x0f) << 8) + ucRxBuffer[10];
                RC_Data[5] = ((ucRxBuffer[13] & 0x0f) << 8) + ucRxBuffer[12];
                RC_Data[6] = ((ucRxBuffer[15] & 0x0f) << 8) + ucRxBuffer[14];
                RC_Data[7] = ((ucRxBuffer[17] & 0x0f) << 8) + ucRxBuffer[16];
                RC_Data[8] = ((ucRxBuffer[19] & 0x0f) << 8) + ucRxBuffer[18];
                RC_Data[9] = ((ucRxBuffer[21] & 0x0f) << 8) + ucRxBuffer[20];
            }
#elif SBUS
            if(ucRxBuffer[0]==0x0F && ucRxBuffer[24]==0x00){
                RC_Data[0] = ((   0x0000 + (ucRxBuffer[ 2] & 0x07)) << 8) + ucRxBuffer[ 1];
                RC_Data[1] = (((  0x0000 + (ucRxBuffer[ 3] & 0x3F)) << 8) + ucRxBuffer[ 2]) >> 3;
                RC_Data[2] = (((((0x0000 + (ucRxBuffer[ 5] & 0x01)) << 8) + ucRxBuffer[ 4]) << 8) + ucRxBuffer[5]) >> 6;
                RC_Data[3] = (((  0x0000 + (ucRxBuffer[ 6] & 0x0F)) << 8) + ucRxBuffer[ 5]) >> 1;
                RC_Data[4] = (((  0x0000 + (ucRxBuffer[ 7] & 0x7F)) << 8) + ucRxBuffer[ 6]) >> 4;
                RC_Data[5] = (((((0x0000 + (ucRxBuffer[ 9] & 0x03)) << 8) + ucRxBuffer[ 8]) << 8) + ucRxBuffer[7]) >> 7;
                RC_Data[6] = (((  0x0000 + (ucRxBuffer[10] & 0x1F)) << 8) + ucRxBuffer[ 6]) >> 2;
                RC_Data[7] = (((  0x0000 + (ucRxBuffer[11] & 0xFF)) << 8) + ucRxBuffer[10]) >> 5;
                RC_Data[8] = (((  0x0000 + (ucRxBuffer[13] & 0x07)) << 8) + ucRxBuffer[12]) >> 0;
                RC_Data[9] = (((  0x0000 + (ucRxBuffer[14] & 0x3F)) << 8) + ucRxBuffer[13]) >> 3;
            }
#endif
            //处理摇杆中位漂移，添加死区,ibus/sbus的数值为1000-2000之间
            for(int i = 0;i < 8;i++){
                if(RC_Data[i] > 1550){
                    RC_Data[i] = RC_Data[i] - 50;
                }else if(RC_Data[i] < 1450){
                    RC_Data[i] = RC_Data[i] + 50;
                }else{
                    RC_Data[i] = 1500;
                }
            }
            //将遥控器值转入需要变量
            RC_Info.ch1 = (int16_t)(RC_Data[0]  - 1500);
            RC_Info.ch2 = (int16_t)(RC_Data[1]  - 1500);
            RC_Info.ch3 = (int16_t)(RC_Data[2]  - 1500);
            RC_Info.ch4 = (int16_t)(RC_Data[3]  - 1500);
            
            RC_Info.swa = (RC_Data[4]>1500)?-500:500;
            RC_Info.swb = (RC_Data[5]>1500)?-500:500;
            RC_Info.swc = (RC_Data[6]>1500)?-500:((RC_Data[6]==1500)?0:500);//三段开关
            RC_Info.swd = (RC_Data[7]>1500)?-500:500;
            
            RC_Info.vra = RC_Data[8]-1000;
            RC_Info.vrb = RC_Data[9]-1000;
        
            /*ROS_INFO("----------------");
            ROS_INFO("ch1:%d",RC_Info.ch1);
            ROS_INFO("ch2:%d",RC_Info.ch2);
            ROS_INFO("ch3:%d",RC_Info.ch3);
            ROS_INFO("ch4:%d",RC_Info.ch4);
            ROS_INFO("swa:%d",RC_Info.swa);
            ROS_INFO("swb:%d",RC_Info.swb);
            ROS_INFO("swc:%d",RC_Info.swc);
            ROS_INFO("swd:%d",RC_Info.swd);
            ROS_INFO("vra:%d",RC_Info.vra);
            ROS_INFO("vrb:%d",RC_Info.vrb);*/
            con=0;
        }else{
            con++;
            if(con>100){
                //ROS_INFO("RC unconnected");
                con=0;
                RC_Info.rssi=0;
            }
            
        }
        rcmsg_pub.publish(RC_Info);
        loop_rate.sleep();
    }
    
    //关闭串口
    sp.close();
 
    return 0;
}
