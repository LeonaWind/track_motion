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

	namedWindow( "motion detection demo", 0 );

	Mat frame, hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
	bool paused = false;

	for(;;)
	{
		cap >> frame;//读入一帧图像
		if( !frame.empty() )
		{
			frame.copyTo(image);

			if( !paused )
			{

			}

			imshow( "CamShift Demo", image );

			//读取键盘输入操作
			char c = (char)waitKey(10);
			if( c == 27 )
				break;
		}
	}
	return 0;
}