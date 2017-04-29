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
using namespace cv;
using namespace std;

extern int vmin;
extern int vmax;
extern int smin;
extern bool debug;

RotatedRect motion_tracking(Rect &track_window,Mat image);//运动追踪
typedef struct queuet      //队列的结构
{
 int *front;
 int *rear;
}queue_t;

class trackThread {
private:
	queue<int> list;
	queue<int> result;
	int track_num;//需要追踪的窗口数
	mutex list_mutex;
	mutex result_mutex;
public:
	trackThread(int num,queue<int> l){
		track_num=num;
		list=l;
	}

	void thread_test(){
		int a;
		{
			unique_lock<mutex> lock(list_mutex);
			a=list.front();//从列表中获取一个数
			list.pop();
		}
		a++;
		{
			unique_lock<mutex> lock(result_mutex);
			result.push(a);//放入结果列表
		}
	}
	queue<int> get_result(){
		return result;
	}

	queue<int> get_list(){
		return list;
	}

	void set_list(queue<int> l){
		int length1=list.size();
		int length2=l.size();
		for(int i=0;i<length1;i++)
		{
			list.pop();
		}
		for(int i=0;i<length2;i++)
		{
			int e=l.front();
			list.push(e);
			l.pop();
		}
	}

	void print(queue<int> Q,String s)  //打印队列
	{
		cout<<s;
		int length=Q.size();
		for(int i=0;i<length;i++)
		{
			int e=Q.front();
			cout<<e<<" ";
			Q.pop();
		}
		cout<<endl;
	}

};

void run_thread(trackThread& track_thread);

#endif