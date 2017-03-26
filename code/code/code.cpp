#include <opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <iomanip>
#include <cxcore.h>


using namespace cv;
using namespace std;

Mat image;

bool backprojMode = false;
bool selectObject = false;
int trackObject = 0;
bool showHist = true;
Point origin;
Rect selection;
int vmin = 10, vmax = 256, smin = 30;
vector<Point2f> points[2]; 
vector<Point2f> initial; 
vector<uchar> status; // ����������״̬��������������Ϊ1������Ϊ0
vector<float> err;

bool addNewPoints();
bool acceptTrackedPoint(int i);

const char* keys =
{
	"{1|  | 0 | camera number}"
};

int main( int argc, const char** argv )
{
	VideoCapture cap;
	Rect trackWindow;
	int hsize = 16;
	float hranges[] = {0,180};
	const float* phranges = hranges;
	CommandLineParser parser(argc, argv, keys);
	int camNum = parser.get<int>("0");

	cap.open(camNum);

	if( !cap.isOpened() )
	{
		cout << "***Could not initialize capturing...***\n";
		cout << "Current parameter's value: \n";
		parser.printParams();
		return -1;
	}

	Mat frame, hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
	bool paused = false;
	Mat gray;
	int minDis =10;

	for(;;)
	{
		cap >> frame;//����һ֡ͼ��
		if( !frame.empty() )
		{
			frame.copyTo(image);

			if( !paused )
			{
				//�ǵ���
				cvtColor(image,gray,CV_BGR2GRAY);//��Ϊ�Ҷ�ͼ
				std::vector<Point2f> corner;
				goodFeaturesToTrack(gray,corner,500,0.04,minDis,Mat(),3,false,0);
				for(int i=0;i<corner.size();i++){
					circle(image,Point(corner.at(i).x,corner.at(i).y),3,Scalar(255,255,0));
				}

				imshow( "CamShift Demo", image );

				Mat frame2,gray2;
				cap >> frame2;//����ڶ�֡ͼ��
				cvtColor(frame2,gray2,CV_BGR2GRAY);//��Ϊ�Ҷ�ͼ

				// ���������
				if (addNewPoints())
				{
					points[0].insert(points[0].end(), corner.begin(), corner.end());
					initial.insert(initial.end(), corner.begin(), corner.end());
				}

				calcOpticalFlowPyrLK (gray, gray2, points[0], points[1], status, err); //�������

				// ȥ��һЩ���õ�������
				int k = 0;
				for (size_t i=0; i<points[1].size(); i++)
				{
					if (acceptTrackedPoint(i))
					{
						initial[k] = initial[i];
						points[1][k++] = points[1][i];
					}
				}
				points[1].resize(k);
				initial.resize(k);
				// ��ʾ��������˶��켣
				for (size_t i=0; i<points[1].size(); i++)
				{
					line(frame2, initial[i], points[1][i], Scalar(0, 0, 255));
					circle(frame2, points[1][i], 3, Scalar(0, 255, 0), -1);
				}

				imshow("KL", frame2);

			}



			//��ȡ�����������
			char c = (char)waitKey(10);
			if( c == 27 )
				break;
		}
	}
	return 0;
}

//-------------------------------------------------------------------------------------------------
// function: addNewPoints
// brief: ����µ��Ƿ�Ӧ�ñ����
// parameter:
// return: �Ƿ���ӱ�־
//-------------------------------------------------------------------------------------------------
bool addNewPoints()
{
	return points[0].size() <= 10;
}

//-------------------------------------------------------------------------------------------------
// function: acceptTrackedPoint
// brief: ������Щ���ٵ㱻����
// parameter:
// return:
//-------------------------------------------------------------------------------------------------
bool acceptTrackedPoint(int i)
{
return status[i] && ((abs(points[0][i].x - points[1][i].x) + abs(points[0][i].y - points[1][i].y)) > 2);
}