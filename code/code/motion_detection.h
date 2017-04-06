#ifndef MOTION_DETECTION_H
#define MOTION_DETECTION_H

#include <opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <iomanip>
#include <cxcore.h>
using namespace cv;
using namespace std;

Rect motion_detection(Mat &image,Mat &image_gray,Mat &background_gray);//运动检测
Rect get_track_selection(Mat &image);//获取追踪区域

#endif