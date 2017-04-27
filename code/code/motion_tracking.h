#ifndef MOTION_TRACKING_H
#define MOTION_TRACKING_H

#include <opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <iomanip>
#include <cxcore.h>
using namespace cv;
using namespace std;

extern int vmin;
extern int vmax;
extern int smin;
extern bool debug;
extern int track_num;//追踪的窗口数
extern mutex m;//互斥量

RotatedRect motion_tracking(Rect &track_window,Mat image);//运动追踪

#endif