cmake_minimum_required(VERSION 3.0.2)
project(wb_cpp)

find_package(pybind11 REQUIRED)
find_package(OpenCV REQUIRED)


pybind11_add_module(whitebalance whitebalance.cpp)
target_link_libraries(whitebalance PRIVATE ${OpenCV_LIBS})