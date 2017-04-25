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

Rect background_motion_detection(Mat &image_gray,Mat &background_gray);//�˶����
Rect get_track_selection(Mat &image);//��ȡ׷������
Rect get_track_selection_all(Mat &image);
Rect rectA_intersect_rectB(Rect rectA, Rect rectB);//����������Ľ��沿��
Mat frame3_diff_motion_detection(Mat image_gray_pre,Mat image_gray,Mat image_gray_next,Mat &background_gray_cv32f);//�Ľ���֡�����˶�����
void mat_and(Mat src1,Mat src2,Mat &dst);
void mat_or(Mat src1,Mat src2,Mat &dst);

#endif