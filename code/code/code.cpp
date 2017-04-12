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
	Rect track_window;//���ٵ�����

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
	int get_background_flag = 0;//���㵱ǰΪ�ڼ�֡
	Mat image,background_image;//��ǰ����ͼ�񣬱���ͼ
	Mat image_gray,background_gray;//��ǰ����ͼ�񣬱���ͼ�ĻҶ�ͼ


	while(1)
	{
		//capture >> pre_frame;//����һ֡ͼ��
		videoCapture.read(pre_frame);

		imshow("current_frame",pre_frame);

		if( !pre_frame.empty() )
		{
			cout<<"���ǵ�"<<get_background_flag<<"֡"<<endl;
			pre_frame.copyTo(image);
			//����ǵ�һ֡����Ҫ�����ڴ棬����ʼ��    
			if(get_background_flag == 0) { 
				//ת���ɵ�ͨ��ͼ���ٴ���   
				cvtColor(image, image_gray, CV_BGR2GRAY); 
				image_gray.convertTo(background_gray,CV_32F); 
				++get_background_flag;
			}else{
				cvtColor(image, image_gray, CV_BGR2GRAY);//��Ϊ�Ҷ�ͼ 
				if((get_background_flag%50==0)||(get_background_flag%50==1)||(get_background_flag%50==2)){
					Rect track_window_temp = motion_detection(image_gray,background_gray);//�˶����,���ظ�������
					track_window=rectA_intersect_rectB(track_window_temp,track_window);//����������Ľ�������
				}
				motion_tracking(track_window,image);//�˶�׷�� 
				++get_background_flag;
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

