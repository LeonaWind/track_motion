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
int track_num=0;//追踪的窗口数
mutex m;//互斥量

const char* keys =
{
	"{1|  | 0 | camera number}"
};

int main( int argc, const char** argv )
{
	Rect track_window;//跟踪的区域
	char choice,tracking_choice;
	int capture_flag=0,tracking_flag=0;
	int delay=0;
	int current_frame = 0;//计算当前为第几帧
	clock_t start, finish,detection_time,tracking_time,total_start,total_finish;//用于计算时间复杂度
	double duration,detection_duration,tracking_duration,total_duration;
	double rate = 100;//帧率

	//截屏
	//int sframe1=5,sframe2=90,sframe3=900;//倒车入库
	//int sframe1=6,sframe2=80,sframe3=150;//花样滑冰
	//int sframe1=6,sframe2=72,sframe3=140;//骑自行车

	cout<<"***************请输入您的选择*****************"<<endl;
	cout<<"1--打开电脑摄像头"<<endl;
	cout<<"2--打开测试视频"<<endl;
	cout<<"3--打开测试相片序列"<<endl;
	cout<<"其他键--退出"<<endl;
	cout<<"**********************************************"<<endl;
	//choice=getchar();
	//getchar();
	choice='2';

	cout<<"***************请选择运动跟踪方法*****************"<<endl;
	cout<<"1--CamShift"<<endl;
	cout<<"2--KCF"<<endl;
	cout<<"其他键--退出"<<endl;
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
		cout<<"退出"<<endl;
		return -1;
	}

	if(tracking_choice == '1'){
		tracking_flag=1;
	}else if(tracking_choice == '2'){
		tracking_flag=2;
	}else{
		cout<<"退出"<<endl;
		return -1;
	}

	//打开摄像头
	VideoCapture capture;
	CommandLineParser parser(argc, argv, keys);
	int camNum = parser.get<int>("0");
	if(capture_flag == 1){
		capture.open(camNum);
		delay = 20;//两帧间的间隔时间:
		if( !capture.isOpened() )
		{
			cout << "***Could not initialize capturing...***\n";
			cout << "Current parameter's value: \n";
			parser.printParams();
			return -1;
		}
	}

	//打开视频
	VideoCapture video_capture;
	long total_frame_num = 0;
	if(capture_flag == 2){
		video_capture.open("G:/毕设/data/football.mp4");
		total_frame_num = video_capture.get(CV_CAP_PROP_FRAME_COUNT);
		rate = video_capture.get(CV_CAP_PROP_FPS);//帧率
		delay = 1;//两帧间的间隔时间:
		if( !video_capture.isOpened() )
		{
			cout << "***Could not initialize capturing...***\n";
			cout << "Current parameter's value: \n";
			return -1;
		}
		cout<<"******************基本信息********************"<<endl;
		cout<<"整个视频共"<<total_frame_num<<"帧"<<endl;
		cout<<"帧率为    "<<rate<<"帧/秒"<<endl;
		cout<<"长为"<<video_capture.get(CV_CAP_PROP_FRAME_WIDTH)<<"     宽为"<<video_capture.get(CV_CAP_PROP_FRAME_HEIGHT)<<endl;
		cout<<"**********************************************"<<endl;
	}

	//打开测试相片序列
	string pic_path="G:\\毕设\\data\\Subway\\img\\";
	int max_pic=175;
	if(capture_flag == 3){
		delay = 1;//两帧间的间隔时间:
		current_frame = 1;
	}

	Mat pre_frame, cur_frame;//读取的帧
	Mat pre_gray,cur_gray;
	int nFrmNum = 0;//读取的帧数
	bool paused = false;
	Mat image,background_image;//当前处理图像，背景图
	Mat image1, image2, image3;//改进三帧差法检测运动物体时存储的三帧图像
	Mat image_gray,background_gray;//当前处理图像，背景图的灰度图
	RotatedRect trackBox;//camshift追踪结果

	//KCFT方法
	bool HOG = true;
	bool FIXEDWINDOW = false;
	bool MULTISCALE = true;
	bool SILENT = true;
	bool LAB = false;
	//KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
	// Tracker results
	Rect result;

	//测量速度
	int per=15;

	//多线程
	int list_num=5;
	int thread_num=5;
	double ratio=150/9.15;
	trackThread track_thread(rate, ratio, per, HOG, FIXEDWINDOW, MULTISCALE, LAB);

	total_start = clock();
	while(1)
	{
		if(debug) start = clock();
		//读入一帧图像
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
			//如果太大，将图片变小
			//resize(pre_frame, pre_frame, Size(), 0.5, 0.5);
			imshow("原视频",pre_frame);
			cout<<"这是第"<<current_frame<<"帧"<<endl;
			if(debug) {
				imshow("current_frame",pre_frame);
				cout<<"行"<<pre_frame.rows<<"列"<<pre_frame.cols<<endl;
			}

			pre_frame.copyTo(image);

			//图像预处理
			GaussianBlur(image,image,cv::Size(0,0), 3, 0, 0);//高斯滤波
			cvtColor(image, image_gray, CV_BGR2GRAY);//获取灰度图image_gray,单通道图像
			//equalizeHist(image_gray,image_gray);//直方图均衡化
			//imshow("equalizeHist_gray",image_gray);

			//背景减法检测运动物体
			/*if((current_frame%50==0)||(current_frame%50==1)||(current_frame%50==2)){
			Rect track_window_temp = background_motion_detection(image_gray,background_gray);//运动检测,返回跟踪区域
			track_window=rectA_intersect_rectB(track_window_temp,track_window);//求两个区域的交叉区域
			}*/

			if(current_frame == 1){//如果是第一帧，需要申请内存，并初始化    
				image_gray.convertTo(background_gray,CV_32F); //第一帧作为背景图
			}

			//改进三帧差法检测运动物体
			if(current_frame%per==1){
				image1=image_gray.clone();//获取第一张图
			}
			if(current_frame%per==2){
				image2=image_gray.clone();//获取第二张图
			}
			if(current_frame%per==3){
				if(debug) cout<<"帧差法检测"<<endl;
				image3=image_gray.clone();//获取第三张图
				//Mat detection_image = frame3_diff_motion_detection(image1,image2,image3,background_gray);//运动检测,返回跟踪区域
				Mat segment_image = motion_segment(pre_frame);
				//vector<Rect> track_rect=get_track_selection_many(detection_image,segment_image);
				vector<Rect> track_rect=get_track_selection_many_by_detection(segment_image);//获得追踪的区域
				track_num=track_rect.size();
				if(track_num>0){
					track_thread.update(track_rect,image);//跟新跟踪器
				}
			
				/*
				//简易做法
				int max_area=0;
				Rect track_window_temp;
				for (vector<Rect>::const_iterator iter = track_rect.begin(); iter != track_rect.end(); iter++){
					if((*iter).area()>max_area){
						max_area=(*iter).area();
						track_window_temp=(*iter);
					}
				}

				if(track_window_temp.width>0&&track_window_temp.height>0){
					if(tracking_flag == 1){//camshift运动追踪 
						track_window=track_window_temp;
						if(debug)cout<<"track_window"<<track_window<<endl;
					}else if(tracking_flag == 2){//KCF运动跟踪
						tracker.init(track_window_temp, image);//Rect(xMin, yMin, width, height)
					}
				}
				*/

				if(debug) detection_time=clock();
			}

			/*****************************运动跟踪**********************/
			    //track_num是需要追踪的总数，thread_num是线程总数
				int count_track=track_num;//计算已经追踪的窗口数
				track_thread.update_image(image,pre_frame,current_frame);//跟新图片

				while(count_track>0){
					//每次执行thread_num个
					thread threads[5];
					for(int t=0;t<thread_num;t++){
						threads[t]=thread(run_thread,ref(track_thread));
					}
					for(auto& t:threads){//等待执行完
						t.join();
					}
					count_track-=thread_num;
				}

				track_thread.set_list();//更新

			/*
			if(current_frame>3){
				if(tracking_flag == 1){//camshift运动追踪 
					trackBox=motion_tracking(track_window,image);
					ellipse( pre_frame, trackBox, Scalar(255,0,0), 3, CV_AA );
					imshow( "camshift运动跟踪结果", pre_frame );
					if(debug)cout<<"track_window"<<track_window<<endl;
				}else if(tracking_flag == 2){//KCF运动跟踪
					result = tracker.update(image);
					rectangle(pre_frame, Point(result.x, result.y), Point(result.x + result.width, result.y + result.height), Scalar(0, 255, 255), 1, 8);
					imshow( "KCF运动跟踪结果", pre_frame );
				}
			}
			*/

			if(debug) tracking_time=clock();

			++current_frame;//图片数+1


			//读取键盘输入操作
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
			duration = (double)(finish - start) / CLOCKS_PER_SEC; //计算效率
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

		//截图
		//if(current_frame ==sframe1||current_frame ==sframe2||current_frame ==sframe3){
		//	cout<<"截图"<<endl;
		//	getchar(); 
		//}

	}


	total_finish = clock();
	total_duration = (double)(total_finish - total_start) / CLOCKS_PER_SEC; //计算效率
	cout << "total_duration" << total_duration<<endl;

	//释放资源
	capture.release();
	video_capture.release();

	return 0;
}

