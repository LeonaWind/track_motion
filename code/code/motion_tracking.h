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
#include "kcftracker.h"
using namespace cv;
using namespace std;

extern int vmin;
extern int vmax;
extern int smin;
extern bool debug;

RotatedRect motion_tracking(Rect &track_window,Mat image);//运动追踪

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
	bool HOG;
	bool FIXEDWINDOW;
	bool MULTISCALE;
	bool LAB;
public:
	trackThread(int num,bool hog,bool fixedwindow,bool multiscale,bool lab){
		track_num=num;
		HOG=hog;
		FIXEDWINDOW=fixedwindow;
		MULTISCALE=multiscale;
		LAB=lab;
	}

	void thread_test(){
		KCFTracker a;
		if(!list.empty()){
			{
				unique_lock<mutex> lock(list_mutex);

				a=list.front();//从列表中获取一个数
				list.pop();

			}
			Rect result_rect=a.update(image);
			{
				unique_lock<mutex> lock(pic_mutex);
				rectangle(origin, Point(result_rect.x, result_rect.y), Point(result_rect.x + result_rect.width, result_rect.y + result_rect.height), Scalar(0, 255, 255), 1, 8);
			}
			{
				unique_lock<mutex> lock(result_mutex);
				result.push(a);//放入结果列表
			}
		}
	}

	void update(vector<Rect> r,Mat pic){
		//清空队列
		unique_lock<mutex> list_lock(list_mutex);
		int length=list.size();
		for(int i=0;i<length;i++){
			list.pop();
		}

		//重新放入队列
		for(vector<Rect>::iterator iter = r.begin(); iter != r.end(); iter++){
			KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
			if((*iter).width>0&&(*iter).height>0){
				tracker.init((*iter), pic);
				list.push(tracker);//放入队列
			}
		}
	}

	void update_image(Mat new_image,Mat new_origin){
		image=new_image;
		origin=new_origin;
	}

	void set_image(Mat pic){
		image=pic;
	}

	queue<KCFTracker> get_result(){
		return result;
	}

	queue<KCFTracker> get_list(){
		return list;
	}


	void set_list(){
		unique_lock<mutex> list_lock(list_mutex);
		unique_lock<mutex> result_lock(result_mutex);
		int length1=list.size();
		int length2=result.size();

		for(int i=0;i<length1;i++)
		{
			list.pop();
		}
		for(int i=0;i<length2;i++)
		{
			KCFTracker e=result.front();
			list.push(e);
			result.pop();
		}
		imshow("result",origin);
	}

};

void run_thread(trackThread& track_thread);

#endif