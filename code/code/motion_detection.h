#ifndef MOTION_DETECTION_H
#define MOTION_DETECTION_H

#include <opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <iomanip>
#include <cxcore.h>
using namespace cv;
using namespace std;

Rect motion_detection(Mat &image,Mat &image_gray,Mat &background_gray);//�˶����
Rect get_track_selection(Mat &image);//��ȡ׷������

#endif