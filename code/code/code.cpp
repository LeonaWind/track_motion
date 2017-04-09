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
	Rect trackWindow;//���ٵ�����

	/*
	//������ͷ
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

	//����Ƶ
	VideoCapture videoCapture("G:/����/test.mp4");
	double rate = videoCapture.get(CV_CAP_PROP_FPS);
	int delay = 1000/rate;//��֡��ļ��ʱ��:

    if( !videoCapture.isOpened() )
	{
		cout << "***Could not initialize capturing...***\n";
		cout << "Current parameter's value: \n";
		return -1;
	}

	Mat pre_frame, cur_frame;//��ȡ��֡
	Mat pre_gray,cur_gray;
	int nFrmNum = 0;//��ȡ��֡��
	bool paused = false;
	bool get_background_flag = false;//�Ƿ�Ϊ��һ֡ͼ��
	Mat image,background_image;//��ǰ����ͼ�񣬱���ͼ
	Mat image_gray,background_gray;//��ǰ����ͼ�񣬱���ͼ�ĻҶ�ͼ


	while(1)
	{
		//capture >> pre_frame;//����һ֡ͼ��
		videoCapture.read(pre_frame);

		imshow("current_frame",pre_frame);

		if( !pre_frame.empty() )
		{
			pre_frame.copyTo(image);
			//����ǵ�һ֡����Ҫ�����ڴ棬����ʼ��    
			if(get_background_flag == false) { 
				//ת���ɵ�ͨ��ͼ���ٴ���   
				cvtColor(image, image_gray, CV_BGR2GRAY); 
				image_gray.convertTo(background_gray,CV_32F); 
				get_background_flag = true;
			}else{
				cvtColor(image, image_gray, CV_BGR2GRAY);//��Ϊ�Ҷ�ͼ 
				trackWindow = motion_detection(image_gray,background_gray);//�˶����,���ظ�������
				motion_tracking(trackWindow,image);//�˶�׷�� 
			}

			//��ȡ�����������
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

