#ifndef MOTION_TRACKING_H
#define MOTION_TRACKING_H

#include <opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <iomanip>
#include <cxcore.h>
#include <thread>
#include <mutex>
#include <queue>
#include<sstream>
#include "kcftracker.h"
using namespace cv;
using namespace std;

extern bool debug;

RotatedRect meanshift_motion_tracking(Rect &track_window,Mat image);//运动追踪
float get_distance(Point result_point,Point old_point);

class trackThread {
private:
	queue<KCFTracker> list;
	queue<KCFTracker> result;
	int track_num;//需要追踪的窗口数
	mutex list_mutex;
	mutex result_mutex;
	mutex pic_mutex;
	Mat image;
	Mat origin;
	int rows;
	int cols;
	int rate;
	bool HOG;
	bool FIXEDWINDOW;
	bool MULTISCALE;
	bool LAB;
	double ratio;//转换比例
	int frame_num;
public:
	trackThread(int video_rate,double video_ratio,bool hog,bool fixedwindow,bool multiscale,bool lab){
		rate=video_rate;
		ratio=video_ratio;
		HOG=hog;
		FIXEDWINDOW=fixedwindow;
		MULTISCALE=multiscale;
		LAB=lab;
		frame_num=0;
	};
	~trackThread(){ };
	void thread_test();
	void update(vector<Rect> r,Mat pic);
	void update_image(Mat new_image,Mat new_origin,int new_frame_num);
	void set_image(Mat pic);
	queue<KCFTracker> get_result();
	queue<KCFTracker> get_list();
	void set_list();

};

void run_thread(trackThread& track_thread);

#endif