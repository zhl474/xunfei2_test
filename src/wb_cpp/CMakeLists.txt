cmake_minimum_required(VERSION 3.10)
project(wb_cpp)

find_package(OpenCV REQUIRED)

set(PYBIND11_SOURCE_DIR ${CMAKE_SOURCE_DIR}/pybind11-2.10.1)
set(PYTHON_INC "/usr/include/python3.6m")
add_library(whitebalance MODULE whitebalance.cpp)

# 设置模块属性
set_target_properties(whitebalance PROPERTIES
    PREFIX ""
    SUFFIX ".so"
    CXX_STANDARD 14
)

# 直接包含本地下载的头文件
target_include_directories(whitebalance PRIVATE
    ${PYBIND11_SOURCE_DIR}/include
    ${PYTHON_INC}
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(whitebalance PRIVATE ${OpenCV_LIBS})