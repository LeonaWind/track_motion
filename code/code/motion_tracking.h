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

extern int vmin;
extern int vmax;
extern int smin;
extern bool debug;

RotatedRect motion_tracking(Rect &track_window,Mat image);//运动追踪
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
	int per;
public:
	trackThread(int video_rate,double video_ratio,int video_per,bool hog,bool fixedwindow,bool multiscale,bool lab){
		rate=video_rate;
		ratio=video_ratio;
		HOG=hog;
		FIXEDWINDOW=fixedwindow;
		MULTISCALE=multiscale;
		LAB=lab;
		frame_num=0;
		per=video_per;
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
			//计算速度
			/*//按秒测
			old_rect=tracker.getOldRect();
			result_rect=tracker.update(image);
			int temp_x=(old_rect.x+old_rect.width)/2;
			int temp_y=(old_rect.y+old_rect.height)/2;
			Point old_point(temp_x,temp_y);
			temp_x=(result_rect.x+result_rect.width)/2;
			temp_y=(result_rect.y+result_rect.height)/2;
			Point result_point(temp_x,temp_y);
			float dis=get_distance(result_point,old_point);//移动的距离
			dis=dis/ratio;//转换比例
			int frame=frame_num%per;
			float v=dis*rate/frame;
			cout<<v<<endl;
			ostringstream oss;
			oss<<v;
			string x_string(oss.str());
			x_string+="m/s";
			*/
			//按帧测
			old_rect=tracker.getRect();
			result_rect=tracker.update(image);
			int temp_x=(old_rect.x+old_rect.width)/2;
			int temp_y=(old_rect.y+old_rect.height)/2;
			Point old_point(temp_x,temp_y);
			temp_x=(result_rect.x+result_rect.width)/2;
			temp_y=(result_rect.y+result_rect.height)/2;
			Point result_point(temp_x,temp_y);
			float dis=get_distance(result_point,old_point);//移动的距离
			dis=dis/ratio;//转换比例
			float v=dis*rate;
			cout<<v<<endl;
			ostringstream oss;
			oss<<v;
			string x_string(oss.str());
			x_string+="m/s";

			//减去摄像头的移动速度

			{
				unique_lock<mutex> lock(pic_mutex);
				rectangle(origin, Point(result_rect.x, result_rect.y), Point(result_rect.x + result_rect.width, result_rect.y + result_rect.height), Scalar(0, 255, 255), 1, 8);
				//显示速度
				putText(origin,x_string,Point(result_rect.x, result_rect.y),FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,0,0));
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

	void update_image(Mat new_image,Mat new_origin,int new_frame_num){
		image=new_image;
		origin=new_origin;
		rows=new_image.rows;
		cols=new_image.cols;
		frame_num=new_frame_num;
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