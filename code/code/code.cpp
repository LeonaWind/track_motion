/*
�ļ���:code.cpp
����:������ 
��д����:2017-5
�ļ�����:�������е�����������ȡ��ƵԴ������˶����͸���
*/

#include <opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <iomanip>
#include <cxcore.h>
#include <thread>
#include <mutex>

#include"motion_detection.h"
#include"motion_tracking.h"
#include "kcftracker.h"
#include"motion_segment.h"

using namespace cv;
using namespace std;

bool debug=false;//���Կ���
int track_num=0;//׷�ٵĴ�����

int main( int argc, const char** argv )
{
	Rect track_window;//���ٵ�����
	char choice;
	int capture_flag=0,tracking_flag=0;
	int delay=0;
	int current_frame = 0;//���㵱ǰΪ�ڼ�֡
	clock_t start, finish,detection_time,tracking_time;//���ڼ���ʱ�临�Ӷ�
	double duration,detection_duration,tracking_duration,total_duration;
	double rate = 100;//֡��
	const char* keys ={"{1|  | 0 | camera number}"};
		Mat pre_frame, cur_frame;//��ȡ��֡
	Mat pre_gray,cur_gray;
	int nFrmNum = 0;//��ȡ��֡��
	bool paused = false;
	Mat image,background_image;//��ǰ����ͼ�񣬱���ͼ
	Mat image1, image2, image3;//�Ľ���֡�����˶�����ʱ�洢����֡ͼ��
	Mat image_gray,background_gray;//��ǰ����ͼ�񣬱���ͼ�ĻҶ�ͼ

	//KCFT����
	bool HOG = true;
	bool FIXEDWINDOW = false;
	bool MULTISCALE = true;
	bool SILENT = true;
	bool LAB = false;
	//KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
	// Tracker results
	Rect result;

	//�˶�����ٶ�
	int per=30;

	//����Ч��
	double total_track_duration,total_detection_duration;
	int cal_count=0;

	cout<<"***************����������ѡ��*****************"<<endl;
	cout<<"1--�򿪵�������ͷ"<<endl;
	cout<<"2--�򿪲�����Ƶ"<<endl;
	cout<<"������--�˳�"<<endl;
	cout<<"**********************************************"<<endl;
	choice=getchar();
	getchar();

	cout<<"******************�˶����ٷ���*****************"<<endl;
	cout<<"KCF"<<endl;
	cout<<"**********************************************"<<endl;

	if(choice == '1'){
		capture_flag=1;
	}else if(choice == '2'){
		capture_flag=2;
	}else if(choice == '3'){
		capture_flag=3;
	}else{
		cout<<"�˳�"<<endl;
		return -1;
	}

	/*****************************������*************************/
	//������ͷ
	VideoCapture capture;
	CommandLineParser parser(argc, argv, keys);
	int camNum = parser.get<int>("0");
	if(capture_flag == 1){
		capture.open(camNum);
		delay = 20;//��֡��ļ��ʱ��:
		if( !capture.isOpened() )
		{
			cout << "***Could not initialize capturing...***\n";
			cout << "Current parameter's value: \n";
			parser.printParams();
			return -1;
		}
	}

	//����Ƶ
	VideoCapture video_capture;
	long total_frame_num = 0;
	if(capture_flag == 2){
		video_capture.open("football1.mp4");
		total_frame_num = video_capture.get(CV_CAP_PROP_FRAME_COUNT);
		rate = video_capture.get(CV_CAP_PROP_FPS);//֡��
		delay = 1;//��֡��ļ��ʱ��:
		if( !video_capture.isOpened() )
		{
			cout << "***Could not initialize capturing...***\n";
			cout << "Current parameter's value: \n";
			return -1;
		}
		cout<<"******************������Ϣ********************"<<endl;
		cout<<"������Ƶ��"<<total_frame_num<<"֡"<<endl;
		cout<<"֡��Ϊ    "<<rate<<"֡/��"<<endl;
		cout<<"��Ϊ"<<video_capture.get(CV_CAP_PROP_FRAME_WIDTH)<<"     ��Ϊ"<<video_capture.get(CV_CAP_PROP_FRAME_HEIGHT)<<endl;
		cout<<"**********************************************"<<endl;
	}

	//���߳�
	int list_num=6;
	int thread_num=6;
	double ratio=150/9.35;
	trackThread track_thread(rate, ratio, HOG, FIXEDWINDOW, MULTISCALE, LAB);

	while(1)
	{
		start = clock();
		/*****************************��ȡһ֡*************************/
		if(capture_flag == 1){
			capture >> pre_frame;
			ratio=1;
		}else if(capture_flag == 2){
			video_capture.read(pre_frame);
		}

		if( !pre_frame.empty())
		{
			//���̫�󣬽�ͼƬ��С
			//resize(pre_frame, pre_frame, Size(), 0.3, 0.3);
			imshow("ԭ��Ƶ",pre_frame);
			cout<<"���ǵ�"<<current_frame<<"֡"<<endl;
			if(debug) {
				imshow("current_frame",pre_frame);
				cout<<"��"<<pre_frame.rows<<"��"<<pre_frame.cols<<endl;
			}

			pre_frame.copyTo(image);

			/*****************************ͼ��Ԥ����************************/
			GaussianBlur(image,image,cv::Size(0,0), 2, 0, 0);//��˹�˲�
			cvtColor(image, image_gray, CV_BGR2GRAY);//��ȡ�Ҷ�ͼimage_gray,��ͨ��ͼ��
			if(debug) imshow("Ԥ������",image_gray);
			/*****************************�˶����*************************/
			if(current_frame == 1){//����ǵ�һ֡����Ҫ�����ڴ棬����ʼ��    
				image_gray.convertTo(background_gray,CV_32F); //��һ֡��Ϊ����ͼ
			}

			//�Ľ���֡�����˶�����
			if(current_frame%per==1){
				image1=image_gray.clone();//��ȡ��һ��ͼ
			}
			if(current_frame%per==2){
				image2=image_gray.clone();//��ȡ�ڶ���ͼ
			}
			if(current_frame%per==3){
				if(debug) cout<<"֡����"<<endl;
				image3=image_gray.clone();//��ȡ������ͼ
				Mat detection_image;
				Mat segment_image;
				vector<Rect> track_rect;
				if(capture_flag == 1){
					detection_image = frame3_diff_motion_detection(image1,image2,image3,background_gray);//�˶����,���ظ�������
					track_rect=get_track_selection_all(detection_image);//���׷�ٵ�����
				}else {
					detection_image = frame3_diff_motion_detection(image1,image2,image3,background_gray);//�˶����,���ظ�������
					segment_image = motion_segment(pre_frame);
					track_rect= get_track_selection_many(detection_image,segment_image);
				}
				track_num=track_rect.size();
				if(track_num>0){
					track_thread.update(track_rect,image);//���¸�����
				}

				detection_time=clock();
			}

			/*****************************�˶�����*************************/
			//track_num����Ҫ׷�ٵ�������thread_num���߳�����
			int count_track=track_num;//�����Ѿ�׷�ٵĴ�����
			track_thread.update_image(image,pre_frame,current_frame);//����ͼƬ

			while(count_track>0){
				//ÿ��ִ��thread_num��
				thread threads[6];
				for(int t=0;t<thread_num;t++){
					threads[t]=thread(run_thread,ref(track_thread));
				}
				for(auto& t:threads){//�ȴ�ִ����
					t.join();
				}
				count_track-=thread_num;
			}

			track_thread.set_list();//����
			tracking_time=clock();

			/*****************************����Ч��*************************/
			duration = (double)(finish - start) / CLOCKS_PER_SEC;
			if(detection_time>start){
				if(current_frame%per==3){
					detection_duration=(double)(detection_time - start) / CLOCKS_PER_SEC;
					tracking_duration=(double)(tracking_time - detection_time) / CLOCKS_PER_SEC;
				}else{
					detection_duration=0;
					tracking_duration=(double)(tracking_time - start) / CLOCKS_PER_SEC;
				}

			}else{
				detection_duration=0;
				tracking_duration=(double)(tracking_time - start) / CLOCKS_PER_SEC;
			}

			//cout << "duration" << duration<<endl;
			cout << "detection_duration=" << detection_duration<<endl;
			cout << "tracking_duration=" << tracking_duration<<endl;

			total_detection_duration+=detection_duration;
			total_track_duration+=tracking_duration;
			++current_frame;//ͼƬ��+1

			/***************************��ȡ�����������***********************/
			char c = (char)waitKey(delay);
			if( c == 27 ||(current_frame >= total_frame_num-1&&total_frame_num>0))
				break;
			switch(c){
			case '1':
				break;
			}
			finish = clock();
		}
	}


	/*****************************�ͷ���Դ*************************/
	capture.release();
	video_capture.release();

	return 0;
}

