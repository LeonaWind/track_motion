#include <opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <iomanip>
#include <cxcore.h>
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
	Mat image,background_image;//当前处理图像，背景图
	Mat image_gray,background_gray;//当前处理图像，背景图的灰度图


	while(1)
	{
		capture >> pre_frame;//读入一帧图像
		if( !pre_frame.empty() )
		{
			nFrmNum++;
			pre_frame.copyTo(image);
			//如果是第一帧，需要申请内存，并初始化    
			if(nFrmNum == 1) { 
				image.copyTo(background_image);
				//转化成单通道图像再处理  
				cvtColor(image, background_gray, CV_BGR2GRAY);  
				cvtColor(image, image_gray, CV_BGR2GRAY);  
			}else{
				cvtColor(image, image_gray, CV_BGR2GRAY);//变为灰度图 
				GaussianBlur(image, image, cv::Size(0,0), 3, 0, 0);//先做高斯滤波，以平滑图像 
				absdiff(image, background_gray, image_gray);//当前帧跟背景图相减  
				threshold(background_gray, image_gray, 10, 255.0, CV_THRESH_BINARY);//二值化前景图
				//cvRunningAvg(image, background_image, 0.003, 0);//更新背景  
				  
				imshow("background", background_image);  
				imshow("foreground", image);  
			}
			
			//读取键盘输入操作
			char c = (char)waitKey(10);
			if( c == 27 )
				break;
		}
	}
	return 0;
}
