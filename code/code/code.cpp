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

int vmin = 10;
int vmax = 256;
int smin = 60;
bool debug=true;
int track_num=0;//׷�ٵĴ�����
mutex m;//������

const char* keys =
{
	"{1|  | 0 | camera number}"
};

int main( int argc, const char** argv )
{
	Rect track_window;//���ٵ�����
	char choice,tracking_choice;
	int capture_flag=0,tracking_flag=0;
	int delay=0;//֡��
	int current_frame = 0;//���㵱ǰΪ�ڼ�֡
	clock_t start, finish,detection_time,tracking_time,total_start,total_finish;//���ڼ���ʱ�临�Ӷ�
	double duration,detection_duration,tracking_duration,total_duration;

	//����
	//int sframe1=5,sframe2=90,sframe3=900;//�������
	//int sframe1=6,sframe2=80,sframe3=150;//��������
	//int sframe1=6,sframe2=72,sframe3=140;//�����г�

	cout<<"***************����������ѡ��*****************"<<endl;
	cout<<"1--�򿪵�������ͷ"<<endl;
	cout<<"2--�򿪲�����Ƶ"<<endl;
	cout<<"3--�򿪲�����Ƭ����"<<endl;
	cout<<"������--�˳�"<<endl;
	cout<<"**********************************************"<<endl;
	//choice=getchar();
	//getchar();
	choice='3';

	cout<<"***************��ѡ���˶����ٷ���*****************"<<endl;
	cout<<"1--CamShift"<<endl;
	cout<<"2--KCF"<<endl;
	cout<<"������--�˳�"<<endl;
	cout<<"**********************************************"<<endl;
	//tracking_choice=getchar();
	//getchar();
	tracking_choice='2';

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

	if(tracking_choice == '1'){
		tracking_flag=1;
	}else if(tracking_choice == '2'){
		tracking_flag=2;
	}else{
		cout<<"�˳�"<<endl;
		return -1;
	}

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
		video_capture.open("G:/����/data/test.mp4");
		total_frame_num = video_capture.get(CV_CAP_PROP_FRAME_COUNT);
		double rate = video_capture.get(CV_CAP_PROP_FPS);
		delay = 1;//��֡��ļ��ʱ��:
		if( !video_capture.isOpened() )
		{
			cout << "***Could not initialize capturing...***\n";
			cout << "Current parameter's value: \n";
			return -1;
		}
		cout<<"******************������Ϣ********************"<<endl;
		cout<<"������Ƶ��"<<total_frame_num<<"֡"<<endl;
		cout<<"**********************************************"<<endl;
	}

	//�򿪲�����Ƭ����
	string pic_path="G:\\����\\data\\Subway\\img\\";
	int max_pic=175;
	if(capture_flag == 3){
		delay = 1;//��֡��ļ��ʱ��:
		current_frame = 1;
	}

	Mat pre_frame, cur_frame;//��ȡ��֡
	Mat pre_gray,cur_gray;
	int nFrmNum = 0;//��ȡ��֡��
	bool paused = false;
	Mat image,background_image;//��ǰ����ͼ�񣬱���ͼ
	Mat image1, image2, image3;//�Ľ���֡�����˶�����ʱ�洢����֡ͼ��
	Mat image_gray,background_gray;//��ǰ����ͼ�񣬱���ͼ�ĻҶ�ͼ
	RotatedRect trackBox;//camshift׷�ٽ��

	//KCFT����
	bool HOG = true;
	bool FIXEDWINDOW = false;
	bool MULTISCALE = true;
	bool SILENT = true;
	bool LAB = false;
	KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
	// Tracker results
	Rect result;

	//���߳�
	queue<int> list;
	list.push(1);
	list.push(2);
	list.push(3);
	list.push(4);
	list.push(5);
	int list_num=5;
	int thread_num=5;
	trackThread track_thread(list_num,list);

	total_start = clock();
	while(1)
	{
		if(debug) start = clock();
		//����һ֡ͼ��
		if(capture_flag == 1){
			capture >> pre_frame;
		}else if(capture_flag == 2){
			video_capture.read(pre_frame);
		}else if(capture_flag == 3){
			if(current_frame>max_pic) return -1;
			char s[10];
			sprintf(s,"%0.4d",current_frame);
			String pic_path_temp=pic_path+s+".jpg";
			if(debug) cout<<"pic_path_temp:"<<pic_path_temp<<endl;
			pre_frame = imread(pic_path_temp);
		}

		if( !pre_frame.empty())
		{
			imshow("ԭ��Ƶ",pre_frame);
			if(debug) {
				imshow("current_frame",pre_frame);
				cout<<"���ǵ�"<<current_frame<<"֡"<<endl;
				cout<<"��"<<pre_frame.rows<<"��"<<pre_frame.cols<<endl;
			}

			pre_frame.copyTo(image);

			//ͼ��Ԥ����
			GaussianBlur(image,image,cv::Size(0,0), 3, 0, 0);//��˹�˲�
			cvtColor(image, image_gray, CV_BGR2GRAY);//��ȡ�Ҷ�ͼimage_gray,��ͨ��ͼ��
			//equalizeHist(image_gray,image_gray);//ֱ��ͼ���⻯
			//imshow("equalizeHist_gray",image_gray);

			//������������˶�����
			/*if((current_frame%50==0)||(current_frame%50==1)||(current_frame%50==2)){
			Rect track_window_temp = background_motion_detection(image_gray,background_gray);//�˶����,���ظ�������
			track_window=rectA_intersect_rectB(track_window_temp,track_window);//����������Ľ�������
			}*/

			if(current_frame == 0){//����ǵ�һ֡����Ҫ�����ڴ棬����ʼ��    
				image_gray.convertTo(background_gray,CV_32F); //��һ֡��Ϊ����ͼ
			}

			//�Ľ���֡�����˶�����
			if(current_frame%30==1){
				image1=image_gray.clone();//��ȡ��һ��ͼ
			}
			if(current_frame%30==2){
				image2=image_gray.clone();//��ȡ�ڶ���ͼ
			}
			if(current_frame%30==3){
				if(debug) cout<<"֡����"<<endl;
				image3=image_gray.clone();//��ȡ������ͼ
				Mat detection_image = frame3_diff_motion_detection(image1,image2,image3,background_gray);//�˶����,���ظ�������
				//Mat segment_image = motion_segment(pre_frame);
				//vector<Rect> track_rect=get_track_selection_many(detection_image,segment_image);
				vector<Rect> track_rect=get_track_selection_many_by_detection(detection_image);//���׷�ٵ�����
				track_num=track_rect.size();

				//��������
				int max_area=0;
				Rect track_window_temp;
				for (vector<Rect>::const_iterator iter = track_rect.begin(); iter != track_rect.end(); iter++){
					if((*iter).area()>max_area){
						max_area=(*iter).area();
						track_window_temp=(*iter);
					}
				}

				if(track_window_temp.width>0&&track_window_temp.height>0){
					if(tracking_flag == 1){//camshift�˶�׷�� 
						track_window=track_window_temp;
						if(debug)cout<<"track_window"<<track_window<<endl;
					}else if(tracking_flag == 2){//KCF�˶�����
						tracker.init(track_window_temp, image);//Rect(xMin, yMin, width, height)
					}
				}

				queue<int> list_temp=track_thread.get_list();
				track_thread.print(list_temp,"ԭʼ�ǣ�");

				thread threads[5];
				for(int t=0;t<5;t++){
					threads[t]=thread(run_thread,ref(track_thread));
				}

				for(auto& t:threads){
					t.join();
				}

				
				queue<int> result=track_thread.get_result();
				track_thread.print(result,"����ǣ�");

				//����
				track_thread.set_list();
				list_temp=track_thread.get_list();
				track_thread.print(list_temp,"���º�");

				if(debug) detection_time=clock();
			}

			if(current_frame>3){
				if(tracking_flag == 1){//camshift�˶�׷�� 
					trackBox=motion_tracking(track_window,image);
					ellipse( pre_frame, trackBox, Scalar(255,0,0), 3, CV_AA );
					imshow( "camshift�˶����ٽ��", pre_frame );
					if(debug)cout<<"track_window"<<track_window<<endl;
				}else if(tracking_flag == 2){//KCF�˶�����
					result = tracker.update(image);
					rectangle(pre_frame, Point(result.x, result.y), Point(result.x + result.width, result.y + result.height), Scalar(0, 255, 255), 1, 8);
					imshow( "KCF�˶����ٽ��", pre_frame );
				}
			}

			if(debug) tracking_time=clock();

			++current_frame;//ͼƬ��+1


			//��ȡ�����������
			char c = (char)waitKey(delay);
			if( c == 27 ||(current_frame >= total_frame_num-1&&total_frame_num>0))
				break;
			switch(c){
			case '1':
				break;
			}
			if(debug) finish = clock();
		}

		if(debug){
			duration = (double)(finish - start) / CLOCKS_PER_SEC; //����Ч��
			if(detection_time>start){
				detection_duration=(double)(detection_time - start) / CLOCKS_PER_SEC;
				tracking_duration=(double)(tracking_time - detection_time) / CLOCKS_PER_SEC;
			}else{
				detection_duration=0;
				tracking_duration=(double)(tracking_time - start) / CLOCKS_PER_SEC;
			}

			cout << "duration" << duration<<endl;
			cout << "detection_duration=" << detection_duration<<endl;
			cout << "tracking_duration=" << tracking_duration<<endl;
		}

		//��ͼ
		//if(current_frame ==sframe1||current_frame ==sframe2||current_frame ==sframe3){
		//	cout<<"��ͼ"<<endl;
		//	getchar(); 
		//}

	}


	total_finish = clock();
	total_duration = (double)(total_finish - total_start) / CLOCKS_PER_SEC; //����Ч��
	cout << "total_duration" << total_duration<<endl;

	//�ͷ���Դ
	capture.release();
	video_capture.release();

	return 0;
}

