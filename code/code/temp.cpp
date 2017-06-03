/*
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

static void onMouse( int event, int x, int y, int, void* )
{
if( selectObject )
{
selection.x = MIN(x, origin.x);
selection.y = MIN(y, origin.y);
selection.width = std::abs(x - origin.x);
selection.height = std::abs(y - origin.y);

selection &= Rect(0, 0, image.cols, image.rows);
}

switch( event )
{
case CV_EVENT_LBUTTONDOWN:
origin = Point(x,y);
selection = Rect(x,y,0,0);
selectObject = true;
break;
case CV_EVENT_LBUTTONUP:
selectObject = false;
if( selection.width > 0 && selection.height > 0 )
trackObject = -1;
break;
}
}

static void help()
{
cout << "\nThis is a demo that shows mean-shift based tracking\n"
"You select a color objects such as your face and it tracks it.\n"
"This reads from video camera (0 by default, or the camera number the user enters\n"
"Usage: \n"
"   ./camshiftdemo [camera number]\n";

cout << "\n\nHot keys: \n"
"\tESC - quit the program\n"
"\tc - stop the tracking\n"
"\tb - switch to/from backprojection view\n"
"\th - show/hide object histogram\n"
"\tp - pause video\n"
"To initialize tracking, select the object with mouse\n";
}

const char* keys =
{
"{1|  | 0 | camera number}"
};

int main( int argc, const char** argv )
{
help();

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
help();
cout << "***Could not initialize capturing...***\n";
cout << "Current parameter's value: \n";
parser.printParams();
return -1;
}

namedWindow( "Histogram", 0 );
namedWindow( "CamShift Demo", 0 );
setMouseCallback( "CamShift Demo", onMouse, 0 );
createTrackbar( "Vmin", "CamShift Demo", &vmin, 256, 0 );
createTrackbar( "Vmax", "CamShift Demo", &vmax, 256, 0 );
createTrackbar( "Smin", "CamShift Demo", &smin, 256, 0 );

Mat frame, hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
bool paused = false;

for(;;)
{
cap >> frame;//����һ֡ͼ��
if( !frame.empty() )
{
frame.copyTo(image);

if( !paused )
{
cvtColor(image, hsv, COLOR_BGR2HSV);//��ɫ�ռ�ת������,��RGB��ɫת��HSV

if( trackObject )
{
int _vmin = vmin, _vmax = vmax;

//inRange�����Ĺ����Ǽ����������ÿ��Ԫ�ش�С�Ƿ���2��������ֵ֮�䣬�����ж�ͨ��,mask����0ͨ������Сֵ��Ҳ����h����
inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
Scalar(180, 256, MAX(_vmin, _vmax)), mask);
int ch[] = {0, 0};
hue.create(hsv.size(), hsv.depth());
mixChannels(&hsv, 1, &hue, 1, ch, 1);//��hsv��һ��ͨ��(Ҳ����ɫ��)�������Ƶ�hue��

//trackObject��ʼ��Ϊ0,���߰�����̵�'c'����ҲΪ0������굥���ɿ���Ϊ-1 
//���ѡ�������ɿ��󣬸ú����ڲ��ֽ��丳ֵ1  
if( trackObject < 0 )
{
Mat roi(hue, selection), maskroi(mask, selection);//�˴��Ĺ��캯��roi�õ���Mat hue�ľ���ͷ����roi������ָ��ָ��hue����������ͬ�����ݣ�selectΪ�����Ȥ������ 
calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);//��roi��0ͨ������ֱ��ͼ��ͨ��mask����hist�У�hsizeΪÿһάֱ��ͼ�Ĵ�С  
normalize(hist, hist, 0, 255, CV_MINMAX);//��hist����������鷶Χ��һ��������һ����0~255

trackWindow = selection;//��������Ϊѡ������
trackObject = 1;//ֻҪ���ѡ�������ɿ�����û�а�������0��'c'����trackObjectһֱ����Ϊ1����˸�if����ֻ��ִ��һ�Σ���������ѡ��������� 

histimg = Scalar::all(0);//�밴��'c'����һ���ģ������all(0)��ʾ���Ǳ���ȫ����0
int binW = histimg.cols / hsize;//histing��һ��200*300�ľ���hsizeӦ����ÿһ��bin�Ŀ�ȣ�Ҳ����histing�����ֳܷ�����bin����
Mat buf(1, hsize, CV_8UC3);//����һ�����嵥bin����
for( int i = 0; i < hsize; i++ )
buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);//Vec3bΪ3��charֵ������
cvtColor(buf, buf, CV_HSV2BGR);//��hsv��ת����bgr

for( int i = 0; i < hsize; i++ )
{
int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);//at����Ϊ����һ��ָ������Ԫ�صĲο�ֵ
rectangle( histimg, Point(i*binW,histimg.rows),
Point((i+1)*binW,histimg.rows - val),
Scalar(buf.at<Vec3b>(i)), -1, 8 );//��һ������ͼ���ϻ�һ���򵥳�ľ��Σ�ָ�����ϽǺ����½ǣ���������ɫ����С�����͵� 
}
}

//����ֱ��ͼ�ķ���ͶӰ������hueͼ��0ͨ��ֱ��ͼhist�ķ���ͶӰ��������backproj��
calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
backproj &= mask;
//ʹ��CamShift���и���,TermCriteria���������Ϊ�����㷨����ֹ����
RotatedRect trackBox = CamShift(backproj, trackWindow,
TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
if( trackWindow.area() <= 1 )
{
int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
trackWindow.x + r, trackWindow.y + r) &
Rect(0, 0, cols, rows);//Rect����Ϊ�����ƫ�ƺʹ�С������һ��������Ϊ��������Ͻǵ����꣬�����ĸ�����Ϊ����Ŀ�͸�
}

if( backprojMode )
cvtColor( backproj, image, COLOR_GRAY2BGR );
ellipse( image, trackBox, Scalar(0,0,255), 3, CV_AA );
}
}
else if( trackObject < 0 )
paused = false;

if( selectObject && selection.width > 0 && selection.height > 0 )
{
Mat roi(image, selection);
bitwise_not(roi, roi);
}

imshow( "CamShift Demo", image );
imshow( "Histogram", histimg );

//��ȡ�����������
char c = (char)waitKey(10);
if( c == 27 )
break;
switch(c)
{
case 'b':
backprojMode = !backprojMode;
break;
case 'c':
trackObject = 0;
histimg = Scalar::all(0);
break;
case 'h':
showHist = !showHist;
if( !showHist )
destroyWindow( "Histogram" );
else
namedWindow( "Histogram", 1 );
break;
case 'p':
paused = !paused;
break;
default:
;
}
}
}
return 0;
}
*/

/*������
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
vector<Point2f> points1;
vector<Point2f> points2;
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
initial.resize(1);
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
if( !frame.empty() )
{
cvtColor(frame2,gray2,CV_BGR2GRAY);//��Ϊ�Ҷ�ͼ
// ���������
if (addNewPoints())
{
points1.insert(points1.end(), corner.begin(), corner.end());
initial.insert(initial.end(), corner.begin(), corner.end());
}
calcOpticalFlowPyrLK (gray, gray2, points1, points2, status, err); //�������
// ȥ��һЩ���õ�������
int k = 0;
for (size_t i=0; i<points2.size(); i++)
{
if (acceptTrackedPoint(i))
{
initial[k] = initial[i];
points2[k++] = points2[i];
}
}
points2.resize(k);
initial.resize(k);
points1.resize(k);
// ��ʾ��������˶��켣
for (size_t i=0; i<points2.size(); i++)
{
line(frame2, initial[i], points2[i], Scalar(0, 0, 255));
circle(frame2, points2[i], 3, Scalar(0, 255, 0), -1);
}
imshow("KL", frame2);
}
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
return points1.size() <= 10;
}
//-------------------------------------------------------------------------------------------------
// function: acceptTrackedPoint
// brief: ������Щ���ٵ㱻����
// parameter:
// return:
//-------------------------------------------------------------------------------------------------
bool acceptTrackedPoint(int i)
{
return status[i] && ((abs(points1[i].x - points2[i].x) + abs(points1[i].y - points2[i].y)) > 2);
}
*/

/*
		//�ҵ�������������
		for (vector<vector<Point>>::const_iterator iter = contours.begin(); iter != contours.end(); iter++){
		if(!(*iter).empty()){
		point_count=0;
		for (vector<Point>::const_iterator inner_iter = (*iter).begin(); inner_iter != (*iter).end(); inner_iter++){
		point_count++;
		}
		}
		if(point_count>point_count_max){
		point_count_max=point_count;
		contours_count_max=contours_count;
		}
		contours_count++;
		}

		//�ҵ���������
		vector<Point> all_contours;
		for (vector<Point>::const_iterator inner_iter = contours[contours_count_max].begin(); inner_iter != contours[contours_count_max].end(); inner_iter++){
		Point x;
		x.x=(*inner_iter).x;
		x.y=(*inner_iter).y;
		all_contours.push_back(x);
		}
		if(!all_contours.empty()){
		rect = boundingRect(all_contours); 
		rectangle(dest_image, cvPoint(rect.x, rect.y), cvPoint(rect.x + rect.width, rect.y + rect.height),CV_RGB(255,255, 255), 1, 8, 0);
		}
		*/

		/*
		//���������������˶��ĵ����һ���������
		vector<Point> all_contours;
		for (vector<vector<Point>>::const_iterator iter = contours.begin(); iter != contours.end(); iter++)
		{
		for (vector<Point>::const_iterator inner_iter = (*iter).begin(); inner_iter != (*iter).end(); inner_iter++){
		Point x;
		x.x=(*inner_iter).x;
		x.y=(*inner_iter).y;
		all_contours.push_back(x);
		point_count++;
		if(point_count>=count_max) break;
		}
		}  
		if(!all_contours.empty()){
		rect = boundingRect(all_contours); 
		rectangle(dest_image, cvPoint(rect.x, rect.y), cvPoint(rect.x + rect.width, rect.y + rect.height),CV_RGB(255,255, 255), 1, 8, 0);
		}
		*/


/*Rect get_track_selection_all(Mat &image){

vector<vector<Point>> contours;
vector<Vec4i> hierarchy;
Mat dest_image=Mat::zeros(image.rows,image.cols,CV_8UC3);
int count_max=1000;//��ౣ���
int point_count=0;
CvRect rect;//׷������

findContours(image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // ��������                
Scalar color(rand()&255,rand()&255,rand()&255);
drawContours(dest_image, contours,-1,color,3);// ����������� 

vector<Point> all_contours;
for (vector<vector<Point>>::const_iterator iter = contours.begin(); iter != contours.end(); iter++)
{
for (vector<Point>::const_iterator inner_iter = (*iter).begin(); inner_iter != (*iter).end(); inner_iter++){
Point x;
x.x=(*inner_iter).x;
x.y=(*inner_iter).y;
all_contours.push_back(x);
point_count++;
if(point_count>=count_max) break;
}
}  
if(!all_contours.empty()){
rect = boundingRect(all_contours); 
rectangle(dest_image, cvPoint(rect.x, rect.y), cvPoint(rect.x + rect.width, rect.y + rect.height),CV_RGB(255,255, 255), 1, 8, 0);
}

imshow("get_track_selection",dest_image);  
return rect;

}*/
//int main( int argc, const char** argv )
//{	
//	VideoCapture capture;//��Ƶ�����ṹ
//	int delay =20;//��֡�ļ��ʱ�䣬Ĭ��20
//	int current_frame_num=0;//��ǰ֡��
//	Mat current_frame;//��ȡ��֡ͼ��
//	Mat image,image_gray;//��ǰ����ͼ��,��ǰ����ͼ��Ҷ�ͼ
//	Mat background,background_gray_cv8u;//����ͼ�ĻҶ�ͼ��CV_8U��ʽ����ͼ 
//	Mat diff_gray;//��ǰ֡ͼ���뱳��ͼ�Ĳ���
//	Mat output;//�������˶�ͼ��
//	double learningRate=0.2;//����������
//
//	/******************************************/
//	/*���������choiceֵ����ȡ����ͷ����Ƶ����*/
//	/******************************************/
//	char choice;
//	cout<<"***************����������������Դ*****************"<<endl;
//	cout<<"1--�򿪵�������ͷ"<<endl;
//	cout<<"2--�򿪲�����Ƶ"<<endl;
//	cout<<"������--�˳�"<<endl;
//	cout<<"**********************************************"<<endl;
//	choice=getchar();
//	getchar();
//	if(choice == '1'){//�򿪵�������ͷ
//		delay = 20;//������֡��ļ��ʱ��:
//		capture.open(0);
//	}else if(choice == '2'){//�򿪲�����Ƶ
//		capture.open("test.avi");//����Ƶ�ļ�
//		double rate = capture.get(CV_CAP_PROP_FPS);//֡��
//		long total_frame_num = capture.get(CV_CAP_PROP_FRAME_COUNT);//��ȡ��֡��
//		delay = 1000/rate;//������֡��ļ��ʱ��:
//	}else{
//		cout<<"�˳�"<<endl;
//		return -1;
//	}
//	if( !capture.isOpened() ){
//		return -1;
//	}
//
//
//	//����while ѭ����ȡ֡
//	while(1){
//		//��ȡһ֡ͼ��
//		if(choice == '1'){//����ͷ����
//			capture >> current_frame;
//		}else if(choice == '2'){//��Ƶ����
//			capture.read(current_frame);
//		}else{
//			return -1;
//		}
//
//		if( !current_frame.empty()){
//			resize(current_frame, current_frame, Size(), 0.8, 0.8);
//			imshow("ԭ��Ƶ",current_frame);
//			current_frame.copyTo(image);
//			//��ͼ��ת���ɵ�ͨ��ͼ����
//			cvtColor(image, image_gray, CV_BGR2GRAY);
//
//			/******************************************/
//			/*������ģ���������������output��      */
//			/*�˶������ص�ֵΪ255���������ص�ֵΪ0    */
//			/******************************************/
//			if(current_frame_num == 0) {
//				//����1�����ǵ�һ֡��ֱ�ӽ�ͼ����Ϊ����ͼbackground
//				image_gray.convertTo(background,CV_32F); 
//			}else{
//				//����2����ǰ֡������ͼ���
//				background.convertTo (background_gray_cv8u,CV_8U);
//				absdiff(image_gray, background_gray_cv8u, diff_gray);  
//				//����3����ֵ��ǰ��ͼ
//				threshold(diff_gray, output, 30, 255.0, CV_THRESH_BINARY);
//				//����4�����±���ͼbackground��output��Ϊ����
//				accumulateWeighted (image_gray, background, learningRate, output);
//				imshow("������ַ������",output);//��ʾ���
//			}
//			++current_frame_num;//ͼƬ��+1
//		}
//
//		//��ȡ����������������esc�˳�
//		char c = (char)waitKey(delay);
//		if(c == 27)
//			break;//����Ƿ��˳�
//	}
//	/******************************************/
//	/*�ͷ���Դ���˳�                          */
//	/******************************************/
//	capture.release();//�ر���Ƶ�ļ�
//	waitKey(0);
//	return 0;
//}

//RotatedRect motion_tracking(Rect &track_window,Mat image){
//
//	Mat frame, hsv;//image��HSV�ռ�ͼ��
//	Mat hue;//hsv��h����
//	Mat mask;//ԭͼ��ֵ�����
//	Mat hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
//	int hsize = 16;
//	float hranges[] = {0,180};
//	const float* phranges = hranges;
//	int _vmin = vmin, _vmax = vmax;
//
//	if(debug)cout<<"motion_tracking track_window"<<track_window<<endl;
//	if(track_window.width>0&&track_window.height>0){
//		cvtColor(image, hsv, COLOR_BGR2HSV);//��ɫ�ռ�ת������,��RGB��ɫת��HSV
//		//inRange�����Ĺ����Ǽ����������ÿ��Ԫ�ش�С�Ƿ���2��������ֵ֮�䣬�����ж�ͨ��,mask����0ͨ������Сֵ��Ҳ����h����
//		//��ֵ�����������mask��
//		inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
//			Scalar(180, 256, MAX(_vmin, _vmax)), mask);
//
//		int ch[] = {0, 0};
//		hue.create(hsv.size(), hsv.depth());
//		mixChannels(&hsv, 1, &hue, 1, ch, 1);//��hsv��һ��ͨ��(Ҳ����ɫ��)�������Ƶ�hue��
//
//		Mat roi(hue, track_window);//hue����������ֱ��ͼ������,track_windowΪ�����Ȥ������ 
//		Mat maskroi(mask, track_window);//roi��Ӧ�����룬��roi��ӦmaskroiֵΪ1�ĵ㽫��������ֱ��ͼ
//		calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);//��roi��0ͨ������ֱ��ͼ����hist�У���maskroiΪ���룩��hsizeΪÿһάֱ��ͼ�Ĵ�С  
//		normalize(hist, hist, 0, 255, CV_MINMAX);//��hist����������鷶Χ��һ��������һ����0~255
//
//		histimg = Scalar::all(0);//�밴��'c'����һ���ģ������all(0)��ʾ���Ǳ���ȫ����0
//		int binW = histimg.cols / hsize;//histing��һ��200*300�ľ���hsizeӦ����ÿһ��bin�Ŀ�ȣ�Ҳ����histing�����ֳܷ�����bin����
//		Mat buf(1, hsize, CV_8UC3);//����һ�����嵥bin����
//		for( int i = 0; i < hsize; i++ )
//			buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);//Vec3bΪ3��charֵ������
//		cvtColor(buf, buf, CV_HSV2BGR);//��hsv��ת����bgr
//
//		for( int i = 0; i < hsize; i++ )
//		{
//			int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);//at����Ϊ����һ��ָ������Ԫ�صĲο�ֵ
//			rectangle( histimg, Point(i*binW,histimg.rows),
//				Point((i+1)*binW,histimg.rows - val),
//				Scalar(buf.at<Vec3b>(i)), -1, 8 );//��һ������ͼ���ϻ�һ���򵥳�ľ��Σ�ָ�����ϽǺ����½ǣ���������ɫ����С�����͵� 
//		}
//
//		//����ֱ��ͼ�ķ���ͶӰ������hueͼ��0ͨ��ֱ��ͼhist�ķ���ͶӰ��������backproj��
//		calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
//		backproj &= mask;
//		//ʹ��CamShift���и��٣�����track_window,TermCriteria���������Ϊ�����㷨����ֹ����,
//		RotatedRect trackBox = CamShift(backproj, track_window,
//			TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 5, 1 ));
//		if( track_window.area() <= 1 )
//		{
//			int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
//			track_window = Rect(track_window.x - r, track_window.y - r,
//				track_window.x + r, track_window.y + r) &
//				Rect(0, 0, cols, rows);//Rect����Ϊ�����ƫ�ƺʹ�С������һ��������Ϊ��������Ͻǵ����꣬�����ĸ�����Ϊ����Ŀ�͸�
//		}
//
//		//cvtColor( backproj, image, COLOR_GRAY2BGR );
//		if(debug) imshow( "Histogram", histimg );
//
//		return trackBox;
//	}
//}
