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
	VideoCapture capture;//������ͷ
	Rect trackWindow;//���ٵ�����
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

	Mat pre_frame, cur_frame;//��ȡ��֡
	Mat pre_gray,cur_gray;
	int nFrmNum = 0;//��ȡ��֡��
	bool paused = false;
	Mat image,background_image;//��ǰ����ͼ�񣬱���ͼ
	Mat image_gray,background_gray;//��ǰ����ͼ�񣬱���ͼ�ĻҶ�ͼ


	while(1)
	{
		capture >> pre_frame;//����һ֡ͼ��
		if( !pre_frame.empty() )
		{
			nFrmNum++;
			pre_frame.copyTo(image);
			//����ǵ�һ֡����Ҫ�����ڴ棬����ʼ��    
			if(nFrmNum == 1) { 
				image.copyTo(background_image);
				//ת���ɵ�ͨ��ͼ���ٴ���  
				cvtColor(image, background_gray, CV_BGR2GRAY);  
				cvtColor(image, image_gray, CV_BGR2GRAY);  
			}else{
				cvtColor(image, image_gray, CV_BGR2GRAY);//��Ϊ�Ҷ�ͼ 
				GaussianBlur(image, image, cv::Size(0,0), 3, 0, 0);//������˹�˲�����ƽ��ͼ�� 
				absdiff(image, background_gray, image_gray);//��ǰ֡������ͼ���  
				threshold(background_gray, image_gray, 10, 255.0, CV_THRESH_BINARY);//��ֵ��ǰ��ͼ
				//cvRunningAvg(image, background_image, 0.003, 0);//���±���  
				  
				imshow("background", background_image);  
				imshow("foreground", image);  
			}
			
			//��ȡ�����������
			char c = (char)waitKey(10);
			if( c == 27 )
				break;
		}
	}
	return 0;
}
