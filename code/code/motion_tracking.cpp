/*
�ļ���:motion_tracking.cpp
����:������ 
��д����:2017-5
�ļ�����:�˶�������ٶȼ���ģ�飬������Ŀ����и���ʹ�õĺ��������̱߳��
*/

#include"motion_tracking.h"

extern bool debug;

//-------------------------------------------------------------------------------------------------
// function:run_thread
// brief:�߳������з���
// parameter:��
// return:��
//-------------------------------------------------------------------------------------------------
void run_thread(trackThread& track_thread){
	track_thread.thread_test();
}

//-------------------------------------------------------------------------------------------------
// function:get_distance
// brief:��ȡ������֮��ľ���
// parameter:Point result_point,Point old_point
// return:����float result
//-------------------------------------------------------------------------------------------------
float get_distance(Point result_point,Point old_point){
	float x=(result_point.x-old_point.x)*(result_point.x-old_point.x);
	float y=(result_point.y-old_point.y)*(result_point.y-old_point.y);
	float result=sqrt(x+y);
	return result;
}

//-------------------------------------------------------------------------------------------------
// function: thread_test
// brief:trackThread���з������̴߳Ӵ����ٶ���list��
//       ȡһ��KCFTracker���и��٣�����ֵ�������ٽ������result
// parameter:��
// return:��
//-------------------------------------------------------------------------------------------------
void trackThread::thread_test(){
	KCFTracker tracker;
	Rect old_rect;//�ɵ�λ��
	Rect result_rect;//�µ�λ��
	int th=5;
	//�������ٶ���list�л���ֵ����ȡһ�����и���
	if(!list.empty()){
		{//���б��л�ȡһ��ֵ
			unique_lock<mutex> lock(list_mutex);
			tracker=list.front();
			list.pop();

		}

		//�����ٶ�
		old_rect=tracker.getRect();//��ȡ��λ��
		result_rect=tracker.update(image);//���٣�����ȡ��λ��
		int temp_x=(old_rect.x+old_rect.width)/2;
		int temp_y=(old_rect.y+old_rect.height)/2;
		Point old_point(temp_x,temp_y);
		temp_x=(result_rect.x+result_rect.width)/2;
		temp_y=(result_rect.y+result_rect.height)/2;
		Point result_point(temp_x,temp_y);
		float dis=get_distance(result_point,old_point);//�ƶ��ľ���
		dis=dis/ratio;//ת������
		float v=dis*rate;//�����ٶ�
		if(debug) cout<<v<<endl;
		ostringstream oss;
		oss<<v;
		string x_string(oss.str());
		x_string+="m/s";

		{//��ͼ����ʾ�ٶ�
			unique_lock<mutex> lock(pic_mutex);
			rectangle(origin, Point(result_rect.x, result_rect.y), Point(result_rect.x + result_rect.width, result_rect.y + result_rect.height), Scalar(0, 255, 255), 1, 8);
			putText(origin,x_string,Point(result_rect.x, result_rect.y),FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,0,0));
		}

		//�жϣ�������ٽ���Ѿ������˱߽磬�Ͳ��ٸ�����
		int ns=(result_rect.y+result_rect.height)/2-(old_rect.y+old_rect.height)/2;
		int we=(result_rect.x+result_rect.width)/2-(old_rect.x+old_rect.width)/2;
		bool ns_flag=false;
		bool we_flag=false;
		//����,�Ҳ�����߽�
		if(ns>=0&&(result_rect.y+result_rect.height)<rows-th){
			ns_flag=true;
		}
		//����
		if(ns<=0&&result_rect.y>th){
			ns_flag=true;
		}
		//����
		if(we<=0&&result_rect.x>th){
			we_flag=true;
		}
		//����
		if(we>=0&&(result_rect.x+result_rect.width)<cols-th){
			we_flag=true;
		}

		//��δ���ﻭ��߽磬�򽫸��ٺ��tracker������ٽ������
		if(ns_flag==true&&we_flag==true){
			unique_lock<mutex> lock(result_mutex);
			result.push(tracker);//�������б�
		}
	}
}

//-------------------------------------------------------------------------------------------------
// function:update
// brief:trackThread���з�����ÿ�ν����µ��˶����󣬽��������������ٶ���list��������ͼƬ
// parameter:����������vector<Rect> r,��һ֡ͼ��Mat pic
// return:��
//-------------------------------------------------------------------------------------------------
void trackThread::update(vector<Rect> r,Mat pic){
	try{
		//��ն���list
		unique_lock<mutex> list_lock(list_mutex);
		int length=list.size();
		for(int i=0;i<length;i++){
			list.pop();
		}

		//���˶�������������
		for(vector<Rect>::iterator iter = r.begin(); iter != r.end(); iter++){
			KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
			if((*iter).width>0&&(*iter).height>0){
				tracker.init((*iter), pic);
				list.push(tracker);//�������
			}
		}
	}catch(const exception& e){
		cout<<e.what()<<endl;
	}
}

//-------------------------------------------------------------------------------------------------
// function:update_image
// brief:trackThread���з��������¼����ͼ��
// parameter:��ͼ��Mat new_image,��һ֡ͼ��Mat new_origin,int new_frame_num
// return:��
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
// brief:trackThread���з��������ø���ʱ��ͼ��
// parameter:��һ֡ͼ��Mat pic
// return:��
//-------------------------------------------------------------------------------------------------
void trackThread::set_image(Mat pic){
	image=pic;
	rows=pic.rows;
	cols=pic.cols;
}

//-------------------------------------------------------------------------------------------------
// function:get_result
// brief:trackThread���з��������ظ��ٽ������
// parameter:��
// return:���ٽ������queue<KCFTracker> result
//-------------------------------------------------------------------------------------------------
queue<KCFTracker> trackThread::get_result(){
	return result;
}

//-------------------------------------------------------------------------------------------------
// function:get_list
// brief:trackThread���з��������ش����ٶ���
// parameter:��
// return:�����ٶ���queue<KCFTracker> list
//-------------------------------------------------------------------------------------------------
queue<KCFTracker> trackThread::get_list(){
	return list;
}

//-------------------------------------------------------------------------------------------------
// function:set_list
// brief:trackThread���з����������ٽ������result�е�ֵ��������ٶ���list
// parameter:��
// return:��
//-------------------------------------------------------------------------------------------------
void trackThread::set_list(){
	unique_lock<mutex> list_lock(list_mutex);
	unique_lock<mutex> result_lock(result_mutex);
	int length1=list.size();
	int length2=result.size();

	//��մ����ٶ���list
	for(int i=0;i<length1;i++){
		list.pop();
	}

	//�����ٽ������result�е�ֵ��������ٶ���list�����result
	for(int i=0;i<length2;i++)
	{
		KCFTracker e=result.front();
		list.push(e);
		result.pop();
	}
	imshow("result",origin);//��ʾ���
}
