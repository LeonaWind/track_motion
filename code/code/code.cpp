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
	VideoCapture capture;//打开摄像头
	Rect trackWindow;//跟踪的区域
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

	Mat pre_frame, cur_frame;//读取的帧
	Mat pre_gray,cur_gray;
	int nFrmNum = 0;//读取的帧数
	bool paused = false;
	bool get_background_flag = false;//是否为第一帧图像
	Mat image,background_image;//当前处理图像，背景图
	Mat image_gray,background_gray;//当前处理图像，背景图的灰度图


	while(1)
	{
		capture >> pre_frame;//读入一帧图像
		if( !pre_frame.empty() )
		{
			pre_frame.copyTo(image);
			//如果是第一帧，需要申请内存，并初始化    
			if(get_background_flag == false) { 
				//转化成单通道图像再处理   
				cvtColor(image, image_gray, CV_BGR2GRAY); 
				image_gray.convertTo(background_gray,CV_32F); 
				get_background_flag = true;
			}else{
				cvtColor(image, image_gray, CV_BGR2GRAY);//变为灰度图 
				trackWindow = motion_detection(image_gray,background_gray);//运动检测,返回跟踪区域
				motion_tracking(trackWindow,image);//运动追踪 
			}

			//读取键盘输入操作
			char c = (char)waitKey(10);
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

