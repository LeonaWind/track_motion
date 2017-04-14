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
	Rect track_window;//���ٵ�����
	char choice;
	int capture_flag=0;
	int delay=0;//֡��
	clock_t start, finish,detection_time,tracking_time;//���ڼ���ʱ�临�Ӷ�
	double duration,detection_duration,tracking_duration;

	cout<<"***************����������ѡ��*****************"<<endl;
	cout<<"1--�򿪵�������ͷ"<<endl;
	cout<<"2--�򿪲�����Ƶ"<<endl;
	cout<<"������--�˳�"<<endl;
	cout<<"**********************************************"<<endl;

	choice=getchar();
	if(choice == '1'){
		capture_flag=1;
	}else if(choice == '2'){
		capture_flag=2;
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
	VideoCapture videoCapture;
	if(capture_flag == 2){
		videoCapture.open("G:/����/test.mp4");
		double rate = videoCapture.get(CV_CAP_PROP_FPS);
		delay = 2000/rate;//��֡��ļ��ʱ��:
		if( !videoCapture.isOpened() )
		{
			cout << "***Could not initialize capturing...***\n";
			cout << "Current parameter's value: \n";
			return -1;
		}
		cout<<"******************������Ϣ********************"<<endl;
		cout<<"֡��"<<delay<<endl;
		cout<<"**********************************************"<<endl;
	}

	Mat pre_frame, cur_frame;//��ȡ��֡
	Mat pre_gray,cur_gray;
	int nFrmNum = 0;//��ȡ��֡��
	bool paused = false;
	int get_background_flag = 0;//���㵱ǰΪ�ڼ�֡
	Mat image,background_image;//��ǰ����ͼ�񣬱���ͼ
	Mat image1, image2, image3;//�Ľ���֡�����˶�����ʱ�洢����֡ͼ��
	Mat image_gray,background_gray;//��ǰ����ͼ�񣬱���ͼ�ĻҶ�ͼ
	RotatedRect trackBox;//׷�ٽ��

	namedWindow( "CamShift Demo", 1);
	createTrackbar( "Vmin", "CamShift Demo", &vmin, 256, 0 );
	createTrackbar( "Vmax", "CamShift Demo", &vmax, 256, 0 );
	createTrackbar( "Smin", "CamShift Demo", &smin, 256, 0 );

	while(1)
	{
		if(debug) start = clock();
		//����һ֡ͼ��
		if(capture_flag == 1){
			capture >> pre_frame;
		}else if(capture_flag == 2){
			videoCapture.read(pre_frame);
		}

		if( !pre_frame.empty() )
		{
			if(debug) imshow("current_frame",pre_frame);
			if(debug) cout<<"���ǵ�"<<get_background_flag<<"֡"<<endl;
			pre_frame.copyTo(image);

			//ͼ��Ԥ����
			GaussianBlur(image,image,cv::Size(0,0), 3, 0, 0);//��˹�˲�
			cvtColor(image, image_gray, CV_BGR2GRAY);//��ȡ�Ҷ�ͼimage_gray,��ͨ��ͼ��
			//equalizeHist(image_gray,image_gray);//ֱ��ͼ���⻯
			//imshow("equalizeHist_gray",image_gray);

			//����ǵ�һ֡����Ҫ�����ڴ棬����ʼ��    
			if(get_background_flag == 0) { 
				image_gray.convertTo(background_gray,CV_32F); //��һ֡��Ϊ����ͼ
				++get_background_flag;
			}else{
				//������������˶�����
				/*if((get_background_flag%50==0)||(get_background_flag%50==1)||(get_background_flag%50==2)){
				Rect track_window_temp = background_motion_detection(image_gray,background_gray);//�˶����,���ظ�������
				track_window=rectA_intersect_rectB(track_window_temp,track_window);//����������Ľ�������
				}*/

				//�Ľ���֡�����˶�����
				if(get_background_flag%30==1){
					image1=image_gray.clone();//��ȡ��һ��ͼ
				}
				if(get_background_flag%30==2){
					image2=image_gray.clone();//��ȡ�ڶ���ͼ
				}
				if(get_background_flag%30==3){
					image3=image_gray.clone();//��ȡ������ͼ
					Rect track_window_temp = frame3_diff_motion_detection(image1,image2,image3,background_gray);//�˶����,���ظ�������
					if(track_window_temp.width>0&&track_window_temp.height>0){
						track_window=track_window_temp;
					}
					if(debug) detection_time=clock();
				}

				trackBox=motion_tracking(track_window,image);//�˶�׷�� 
				if(debug) tracking_time=clock();
				++get_background_flag;

				//��ʾ���
				ellipse( pre_frame, trackBox, Scalar(0,0,255), 3, CV_AA );
				imshow( "CamShift Demo", pre_frame );
				if(debug) finish = clock();

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
	}
	return 0;
}

