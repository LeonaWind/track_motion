#include <opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <iomanip>
#include <cxcore.h>
#include"motion_detection.h"
#include"motion_tracking.h"

using namespace cv;
using namespace std;

const char* keys =
{
	"{1|  | 0 | camera number}"
};

int main( int argc, const char** argv )
{
	Rect track_window;//跟踪的区域

	/*
	//打开摄像头
	VideoCapture capture;
	CommandLineParser parser(argc, argv, keys);
	int camNum = parser.get<int>("0");
	capture.open(camNum);

	if( !capture.isOpened() )
	{
	cout << "***Could not initialize capturing...***\n";
	cout << "Current parameter's value: \n";
	parser.printParams();
	return -1;
	}
	*/

	//打开视频
	VideoCapture videoCapture("G:/毕设/test.mp4");
	double rate = videoCapture.get(CV_CAP_PROP_FPS);
	int delay = 1000/rate;//两帧间的间隔时间:

	if( !videoCapture.isOpened() )
	{
		cout << "***Could not initialize capturing...***\n";
		cout << "Current parameter's value: \n";
		return -1;
	}

	Mat pre_frame, cur_frame;//读取的帧
	Mat pre_gray,cur_gray;
	int nFrmNum = 0;//读取的帧数
	bool paused = false;
	int get_background_flag = 0;//计算当前为第几帧
	Mat image,background_image;//当前处理图像，背景图
	Mat image1, image2, image3;//改进三帧差法检测运动物体时存储的三帧图像
	Mat image_gray,background_gray;//当前处理图像，背景图的灰度图

	cout<<"******************基本信息********************"<<endl;
	cout<<"帧率"<<delay<<endl;
	cout<<"**********************************************"<<endl;

	while(1)
	{
		//capture >> pre_frame;//读入一帧图像
		videoCapture.read(pre_frame);

		imshow("current_frame",pre_frame);

		if( !pre_frame.empty() )
		{
			cout<<"这是第"<<get_background_flag<<"帧"<<endl;
			pre_frame.copyTo(image);

			//图像预处理
			GaussianBlur(image,image,cv::Size(0,0), 3, 0, 0);//高斯滤波
			cvtColor(image, image_gray, CV_BGR2GRAY);//获取灰度图image_gray,单通道图像
			imshow("image_gray_start",image_gray);
			//equalizeHist(image_gray,image_gray);//直方图均衡化
			//imshow("equalizeHist_gray",image_gray);

			//如果是第一帧，需要申请内存，并初始化    
			if(get_background_flag == 0) { 
				image_gray.convertTo(background_gray,CV_32F); //第一帧作为背景图
				++get_background_flag;
			}else{
				//背景减法检测运动物体
				/*if((get_background_flag%50==0)||(get_background_flag%50==1)||(get_background_flag%50==2)){
				Rect track_window_temp = motion_detection(image_gray,background_gray);//运动检测,返回跟踪区域
				track_window=rectA_intersect_rectB(track_window_temp,track_window);//求两个区域的交叉区域
				}*/

				//改进三帧差法检测运动物体
				if(get_background_flag%10==1){
					image1=image_gray.clone();//获取第一张图
				}
				if(get_background_flag%10==2){
					image2=image_gray.clone();//获取第二张图
				}
				if(get_background_flag%10==3){
					image3=image_gray.clone();//获取第三张图
					track_window = frame3_diff_motion_detection(image1,image2,image3,background_gray);//运动检测,返回跟踪区域
				}

				motion_tracking(track_window,image);//运动追踪 
				++get_background_flag;
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
	}
	return 0;
}

