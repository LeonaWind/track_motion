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

RotatedRect motion_tracking(Rect &track_window,Mat image);//�˶�׷��
typedef struct queuet      //���еĽṹ
{
 int *front;
 int *rear;
}queue_t;

class trackThread {
private:
	queue<int> list;
	queue<int> result;
	int track_num;//��Ҫ׷�ٵĴ�����
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
			a=list.front();//���б��л�ȡһ����
			list.pop();
		}
		a++;
		{
			unique_lock<mutex> lock(result_mutex);
			result.push(a);//�������б�
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

	void print(queue<int> Q,String s)  //��ӡ����
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