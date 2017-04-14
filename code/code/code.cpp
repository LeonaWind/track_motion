#include <opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <iomanip>
#include <cxcore.h>
#include"motion_detection.h"
#include"motion_tracking.h"

using namespace cv;
using namespace std;

int vmin = 10;
int vmax = 256;
int smin = 60;
bool debug=true;

const char* keys =
{
	"{1|  | 0 | camera number}"
};

int main( int argc, const char** argv )
{
	Rect track_window;//跟踪的区域
	char choice;
	int capture_flag=0;
	int delay=0;//帧率
	clock_t start, finish,detection_time,tracking_time;//用于计算时间复杂度
	double duration,detection_duration,tracking_duration;

	cout<<"***************请输入您的选择*****************"<<endl;
	cout<<"1--打开电脑摄像头"<<endl;
	cout<<"2--打开测试视频"<<endl;
	cout<<"其他键--退出"<<endl;
	cout<<"**********************************************"<<endl;

	choice=getchar();
	if(choice == '1'){
		capture_flag=1;
	}else if(choice == '2'){
		capture_flag=2;
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
	VideoCapture videoCapture;
	if(capture_flag == 2){
		videoCapture.open("G:/毕设/test.mp4");
		double rate = videoCapture.get(CV_CAP_PROP_FPS);
		delay = 2000/rate;//两帧间的间隔时间:
		if( !videoCapture.isOpened() )
		{
			cout << "***Could not initialize capturing...***\n";
			cout << "Current parameter's value: \n";
			return -1;
		}
		cout<<"******************基本信息********************"<<endl;
		cout<<"帧率"<<delay<<endl;
		cout<<"**********************************************"<<endl;
	}

	Mat pre_frame, cur_frame;//读取的帧
	Mat pre_gray,cur_gray;
	int nFrmNum = 0;//读取的帧数
	bool paused = false;
	int get_background_flag = 0;//计算当前为第几帧
	Mat image,background_image;//当前处理图像，背景图
	Mat image1, image2, image3;//改进三帧差法检测运动物体时存储的三帧图像
	Mat image_gray,background_gray;//当前处理图像，背景图的灰度图
	RotatedRect trackBox;//追踪结果

	namedWindow( "CamShift Demo", 1);
	createTrackbar( "Vmin", "CamShift Demo", &vmin, 256, 0 );
	createTrackbar( "Vmax", "CamShift Demo", &vmax, 256, 0 );
	createTrackbar( "Smin", "CamShift Demo", &smin, 256, 0 );

	while(1)
	{
		if(debug) start = clock();
		//读入一帧图像
		if(capture_flag == 1){
			capture >> pre_frame;
		}else if(capture_flag == 2){
			videoCapture.read(pre_frame);
		}

		if( !pre_frame.empty() )
		{
			if(debug) imshow("current_frame",pre_frame);
			if(debug) cout<<"这是第"<<get_background_flag<<"帧"<<endl;
			pre_frame.copyTo(image);

			//图像预处理
			GaussianBlur(image,image,cv::Size(0,0), 3, 0, 0);//高斯滤波
			cvtColor(image, image_gray, CV_BGR2GRAY);//获取灰度图image_gray,单通道图像
			//equalizeHist(image_gray,image_gray);//直方图均衡化
			//imshow("equalizeHist_gray",image_gray);

			//如果是第一帧，需要申请内存，并初始化    
			if(get_background_flag == 0) { 
				image_gray.convertTo(background_gray,CV_32F); //第一帧作为背景图
				++get_background_flag;
			}else{
				//背景减法检测运动物体
				/*if((get_background_flag%50==0)||(get_background_flag%50==1)||(get_background_flag%50==2)){
				Rect track_window_temp = background_motion_detection(image_gray,background_gray);//运动检测,返回跟踪区域
				track_window=rectA_intersect_rectB(track_window_temp,track_window);//求两个区域的交叉区域
				}*/

				//改进三帧差法检测运动物体
				if(get_background_flag%30==1){
					image1=image_gray.clone();//获取第一张图
				}
				if(get_background_flag%30==2){
					image2=image_gray.clone();//获取第二张图
				}
				if(get_background_flag%30==3){
					image3=image_gray.clone();//获取第三张图
					Rect track_window_temp = frame3_diff_motion_detection(image1,image2,image3,background_gray);//运动检测,返回跟踪区域
					if(track_window_temp.width>0&&track_window_temp.height>0){
						track_window=track_window_temp;
					}
					if(debug) detection_time=clock();
				}

				trackBox=motion_tracking(track_window,image);//运动追踪 
				if(debug) tracking_time=clock();
				++get_background_flag;

				//显示结果
				ellipse( pre_frame, trackBox, Scalar(0,0,255), 3, CV_AA );
				imshow( "CamShift Demo", pre_frame );
				if(debug) finish = clock();

			}

			//读取键盘输入操作
			char c = (char)waitKey(delay);
			if( c == 27 )
				break;
			switch(c){
			case '1':
				break;
			}
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
	}
	return 0;
}

