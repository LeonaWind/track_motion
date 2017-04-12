#ifndef MOTION_DETECTION_H
#define MOTION_DETECTION_H

#include <opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <iomanip>
#include <cxcore.h>
using namespace cv;
using namespace std;

Rect motion_detection(Mat &image_gray,Mat &background_gray);//�˶����
Rect get_track_selection(Mat &image);//��ȡ׷������
Rect rectA_intersect_rectB(Rect rectA, Rect rectB);//����������Ľ��沿��
Rect frame3_diff_motion_detection(Mat image_gray_pre,Mat image_gray,Mat image_gray_next,Mat &background_gray_cv32f);//�Ľ���֡�����˶�����
void mat_and(Mat src1,Mat src2,Mat &dst);
void mat_or(Mat src1,Mat src2,Mat &dst);

#endif