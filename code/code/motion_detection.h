#ifndef MOTION_DETECTION_H
#define MOTION_DETECTION_H

#include <opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <iomanip>
#include <cxcore.h>
#include <string>  
#include <list>  
#include <vector>  
#include <map>  
#include <stack>  
using namespace cv;
using namespace std;

Rect background_motion_detection(Mat &image_gray,Mat &background_gray);//运动检测
Rect get_track_selection(Mat &image);//获取追踪区域
Rect get_track_selection_all(Mat &image);
Rect rectA_intersect_rectB(Rect rectA, Rect rectB);//求两个区域的交叉部分
Mat frame3_diff_motion_detection(Mat image_gray_pre,Mat image_gray,Mat image_gray_next,Mat &background_gray_cv32f);//改进三帧差法检测运动物体
void mat_and(Mat src1,Mat src2,Mat &dst);
void mat_or(Mat src1,Mat src2,Mat &dst);

#endif