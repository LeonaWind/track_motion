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
	int rows;
	int cols;
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
		KCFTracker tracker;
		Rect old_rect;//旧的位置
		Rect result_rect;//新的位置
		int th=5;
		if(!list.empty()){
			{
				unique_lock<mutex> lock(list_mutex);

				tracker=list.front();//从列表中获取一个数
				list.pop();

			}
			old_rect=tracker.getRect();
			result_rect=tracker.update(image);
			{
				unique_lock<mutex> lock(pic_mutex);
				rectangle(origin, Point(result_rect.x, result_rect.y), Point(result_rect.x + result_rect.width, result_rect.y + result_rect.height), Scalar(0, 255, 255), 1, 8);
			}
			//加个判断，如果已经到达了边界，就不再跟踪了
			//判断方向
			int ns=(result_rect.y+result_rect.height)/2-(old_rect.y+old_rect.height)/2;
			int we=(result_rect.x+result_rect.width)/2-(old_rect.x+old_rect.width)/2;
			bool ns_flag=false;
			bool we_flag=false;
			//向下,且不到达边界
			if(ns>=0&&(result_rect.y+result_rect.height)<rows-th){
				ns_flag=true;
			}
			//向上
			if(ns<=0&&result_rect.y>th){
				ns_flag=true;
			}
			//向左
			if(we<=0&&result_rect.x>th){
				we_flag=true;
			}
			//向右
			if(we>=0&&(result_rect.x+result_rect.width)<cols-th){
				we_flag=true;
			}

			if(ns_flag==true&&we_flag==true)
			{
				unique_lock<mutex> lock(result_mutex);
				result.push(tracker);//放入结果列表
			}
		}
	}

	void update(vector<Rect> r,Mat pic){
		try{
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
		}catch(const exception& e){
			cout<<e.what()<<endl;
		}
	}

	void update_image(Mat new_image,Mat new_origin){
		image=new_image;
		origin=new_origin;
		rows=new_image.rows;
		cols=new_image.cols;
	}

	void set_image(Mat pic){
		image=pic;
		rows=pic.rows;
		cols=pic.cols;
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