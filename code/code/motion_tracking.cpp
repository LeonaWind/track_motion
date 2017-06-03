/*
文件名:motion_tracking.cpp
作者:甘晓蓉 
编写日期:2017-5
文件描述:运动检测与速度计算模块，包含对目标进行跟踪使用的函数及多线程编程
*/

#include"motion_tracking.h"

extern bool debug;

//-------------------------------------------------------------------------------------------------
// function:run_thread
// brief:线程中运行方法
// parameter:无
// return:无
//-------------------------------------------------------------------------------------------------
void run_thread(trackThread& track_thread){
	track_thread.thread_test();
}

//-------------------------------------------------------------------------------------------------
// function:get_distance
// brief:获取两个点之间的距离
// parameter:Point result_point,Point old_point
// return:距离float result
//-------------------------------------------------------------------------------------------------
float get_distance(Point result_point,Point old_point){
	float x=(result_point.x-old_point.x)*(result_point.x-old_point.x);
	float y=(result_point.y-old_point.y)*(result_point.y-old_point.y);
	float result=sqrt(x+y);
	return result;
}

//-------------------------------------------------------------------------------------------------
// function: thread_test
// brief:trackThread类中方法，线程从待跟踪队列list中
//       取一个KCFTracker进行跟踪，更新值后放入跟踪结果队列result
// parameter:无
// return:无
//-------------------------------------------------------------------------------------------------
void trackThread::thread_test(){
	KCFTracker tracker;
	Rect old_rect;//旧的位置
	Rect result_rect;//新的位置
	int th=5;
	//若待跟踪队列list中还有值，则取一个进行跟踪
	if(!list.empty()){
		{//从列表中获取一个值
			unique_lock<mutex> lock(list_mutex);
			tracker=list.front();
			list.pop();

		}

		//计算速度
		old_rect=tracker.getRect();//获取旧位置
		result_rect=tracker.update(image);//跟踪，并获取新位置
		int temp_x=(old_rect.x+old_rect.width)/2;
		int temp_y=(old_rect.y+old_rect.height)/2;
		Point old_point(temp_x,temp_y);
		temp_x=(result_rect.x+result_rect.width)/2;
		temp_y=(result_rect.y+result_rect.height)/2;
		Point result_point(temp_x,temp_y);
		float dis=get_distance(result_point,old_point);//移动的距离
		dis=dis/ratio;//转换比例
		float v=dis*rate;//计算速度
		if(debug) cout<<v<<endl;
		ostringstream oss;
		oss<<v;
		string x_string(oss.str());
		x_string+="m/s";

		{//在图中显示速度
			unique_lock<mutex> lock(pic_mutex);
			rectangle(origin, Point(result_rect.x, result_rect.y), Point(result_rect.x + result_rect.width, result_rect.y + result_rect.height), Scalar(0, 255, 255), 1, 8);
			putText(origin,x_string,Point(result_rect.x, result_rect.y),FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,0,0));
		}

		//判断，如果跟踪结果已经到达了边界，就不再跟踪了
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

		//若未到达画面边界，则将跟踪后的tracker放入跟踪结果队列
		if(ns_flag==true&&we_flag==true){
			unique_lock<mutex> lock(result_mutex);
			result.push(tracker);//放入结果列表
		}
	}
}

//-------------------------------------------------------------------------------------------------
// function:update
// brief:trackThread类中方法，每次进行新的运动检测后，将检测结果放入待跟踪队列list，并跟新图片
// parameter:待跟踪区域vector<Rect> r,新一帧图像Mat pic
// return:无
//-------------------------------------------------------------------------------------------------
void trackThread::update(vector<Rect> r,Mat pic){
	try{
		//清空队列list
		unique_lock<mutex> list_lock(list_mutex);
		int length=list.size();
		for(int i=0;i<length;i++){
			list.pop();
		}

		//将运动检测结果放入队列
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

//-------------------------------------------------------------------------------------------------
// function:update_image
// brief:trackThread类中方法，更新计算的图像
// parameter:旧图像Mat new_image,新一帧图像Mat new_origin,int new_frame_num
// return:无
//-------------------------------------------------------------------------------------------------
void trackThread::update_image(Mat new_image,Mat new_origin,int new_frame_num){
	image=new_image;
	origin=new_origin;
	rows=new_image.rows;
	cols=new_image.cols;
	frame_num=new_frame_num;
}

//-------------------------------------------------------------------------------------------------
// function:set_image
// brief:trackThread类中方法，设置跟踪时的图像
// parameter:新一帧图像Mat pic
// return:无
//-------------------------------------------------------------------------------------------------
void trackThread::set_image(Mat pic){
	image=pic;
	rows=pic.rows;
	cols=pic.cols;
}

//-------------------------------------------------------------------------------------------------
// function:get_result
// brief:trackThread类中方法，返回跟踪结果队列
// parameter:无
// return:跟踪结果队列queue<KCFTracker> result
//-------------------------------------------------------------------------------------------------
queue<KCFTracker> trackThread::get_result(){
	return result;
}

//-------------------------------------------------------------------------------------------------
// function:get_list
// brief:trackThread类中方法，返回待跟踪队列
// parameter:无
// return:待跟踪队列queue<KCFTracker> list
//-------------------------------------------------------------------------------------------------
queue<KCFTracker> trackThread::get_list(){
	return list;
}

//-------------------------------------------------------------------------------------------------
// function:set_list
// brief:trackThread类中方法，将跟踪结果队列result中的值放入待跟踪队列list
// parameter:无
// return:无
//-------------------------------------------------------------------------------------------------
void trackThread::set_list(){
	unique_lock<mutex> list_lock(list_mutex);
	unique_lock<mutex> result_lock(result_mutex);
	int length1=list.size();
	int length2=result.size();

	//清空待跟踪队列list
	for(int i=0;i<length1;i++){
		list.pop();
	}

	//将跟踪结果队列result中的值放入待跟踪队列list，清空result
	for(int i=0;i<length2;i++)
	{
		KCFTracker e=result.front();
		list.push(e);
		result.pop();
	}
	imshow("result",origin);//显示结果
}
