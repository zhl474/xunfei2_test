## 改动说明

xf_mic_asr_offline语音识别包改动：采用平衡小车之家方案，边录音边检测

ydlidar激光雷达驱动包改动：ydlidar.launch将samp_rate改为4，即以4k频率采样（4K精度0.1m，9K精度为0.28m），同时tf变换的频率调高至100hz（频率太小会出现激光雷达信息歪着的情况）

增加serial_port包：串口读取遥控器接收机的isbus/sbus信息，发布遥控器信息/RC（信息类型为自定义rcmsg.msg）

增加radio包：遥控器控制

增加map包：gmapping建图，这份代码不是在jetsonnano上跑的，是在电脑上跑的

增加gazebo_nav包：导航功能包



## 编译时要注意：

几个串口的设备配置，将src/udev放到/etc/udev/rules.d里面，然后重启

先执行: `catkin_make -DCATKIN_WHITELIST_PACKAGES="ucar_controller"`
（因为在radio包里有掉用其服务，包含其服务头文件#include <ucar_controller/SetLEDMode.h>）

然后执行: `catkin_make -DCATKIN_WHITELIST_PACKAGES="serial_port"`
（因为在radio包里有掉用其消息类型，包含其消息头文件#include <serial_port/rcmsg.h>）

然后catkin_make -DCATKIN_WHITELIST_PACKAGES  命令结束后，再回到那种catkin_make 编译所有包的状态，执行：`catkin_make -DCATKIN_WHITELIST_PACKAGES=""`

第一次编译成功之后有小改动可以直接`catkin_make`编译所有包，不会报错



