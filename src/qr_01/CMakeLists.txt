cmake_minimum_required(VERSION 3.0.2)
project(qr_01)

# 查找 ROS 依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

# 使用 pkg-config 查找 ZBar
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBar REQUIRED zbar)

# 查找 OpenCV 并指定必要组件
find_package(OpenCV REQUIRED COMPONENTS
  core
  highgui
  imgproc
)

add_service_files(
  FILES
  qr_srv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# 配置 catkin 包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES qr_01
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZBar_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 生成可执行文件
add_executable(aruco src/aruco.cpp)
# add_executable(aruco_sub src/aruco_sub.cpp)

# 链接库（规范写法）
target_link_libraries(aruco
  ${catkin_LIBRARIES}
  ${ZBar_LIBRARIES}
  ${OpenCV_LIBS}  # 使用 OpenCV 的变量名 OpenCV_LIBS
)

# target_link_libraries(aruco_sub
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
# )