#ifndef TRAFFIC_LIGHT_H
#define TRAFFIC_LIGHT_H

#include <opencv2/opencv.hpp>
#include <iostream>

int detectTrafficLightStatus(cv::Mat frame);

#endif