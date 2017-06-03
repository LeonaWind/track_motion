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
cap >> frame;//读入一帧图像
if( !frame.empty() )
{
frame.copyTo(image);

if( !paused )
{
cvtColor(image, hsv, COLOR_BGR2HSV);//颜色空间转换函数,将RGB颜色转向HSV

if( trackObject )
{
int _vmin = vmin, _vmax = vmax;

//inRange函数的功能是检查输入数组每个元素大小是否在2个给定数值之间，可以有多通道,mask保存0通道的最小值，也就是h分量
inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
Scalar(180, 256, MAX(_vmin, _vmax)), mask);
int ch[] = {0, 0};
hue.create(hsv.size(), hsv.depth());
mixChannels(&hsv, 1, &hue, 1, ch, 1);//将hsv第一个通道(也就是色调)的数复制到hue中

//trackObject初始化为0,或者按完键盘的'c'键后也为0，当鼠标单击松开后为-1 
//鼠标选择区域松开后，该函数内部又将其赋值1  
if( trackObject < 0 )
{
Mat roi(hue, selection), maskroi(mask, selection);//此处的构造函数roi用的是Mat hue的矩阵头，且roi的数据指针指向hue，即共用相同的数据，select为其感兴趣的区域 
calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);//将roi的0通道计算直方图并通过mask放入hist中，hsize为每一维直方图的大小  
normalize(hist, hist, 0, 255, CV_MINMAX);//将hist矩阵进行数组范围归一化，都归一化到0~255

trackWindow = selection;//跟踪区域为选择区域
trackObject = 1;//只要鼠标选完区域松开后，且没有按键盘清0键'c'，则trackObject一直保持为1，因此该if函数只能执行一次，除非重新选择跟踪区域 

histimg = Scalar::all(0);//与按下'c'键是一样的，这里的all(0)表示的是标量全部清0
int binW = histimg.cols / hsize;//histing是一个200*300的矩阵，hsize应该是每一个bin的宽度，也就是histing矩阵能分出几个bin出来
Mat buf(1, hsize, CV_8UC3);//定义一个缓冲单bin矩阵
for( int i = 0; i < hsize; i++ )
buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);//Vec3b为3个char值的向量
cvtColor(buf, buf, CV_HSV2BGR);//将hsv又转换成bgr

for( int i = 0; i < hsize; i++ )
{
int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);//at函数为返回一个指定数组元素的参考值
rectangle( histimg, Point(i*binW,histimg.rows),
Point((i+1)*binW,histimg.rows - val),
Scalar(buf.at<Vec3b>(i)), -1, 8 );//在一幅输入图像上画一个简单抽的矩形，指定左上角和右下角，并定义颜色，大小，线型等 
}
}

//计算直方图的反向投影，计算hue图像0通道直方图hist的反向投影，并让入backproj中
calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
backproj &= mask;
//使用CamShift进行跟踪,TermCriteria这个类是作为迭代算法的终止条件
RotatedRect trackBox = CamShift(backproj, trackWindow,
TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
if( trackWindow.area() <= 1 )
{
int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
trackWindow.x + r, trackWindow.y + r) &
Rect(0, 0, cols, rows);//Rect函数为矩阵的偏移和大小，即第一二个参数为矩阵的左上角点坐标，第三四个参数为矩阵的宽和高
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

//读取键盘输入操作
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

/*光流法
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
vector<uchar> status; // 跟踪特征的状态，特征的流发现为1，否则为0
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
cap >> frame;//读入一帧图像
if( !frame.empty() )
{
frame.copyTo(image);
if( !paused )
{
//角点检测
cvtColor(image,gray,CV_BGR2GRAY);//变为灰度图
std::vector<Point2f> corner;
goodFeaturesToTrack(gray,corner,500,0.04,minDis,Mat(),3,false,0);
for(int i=0;i<corner.size();i++){
circle(image,Point(corner.at(i).x,corner.at(i).y),3,Scalar(255,255,0));
}
imshow( "CamShift Demo", image );
Mat frame2,gray2;
cap >> frame2;//读入第二帧图像
if( !frame.empty() )
{
cvtColor(frame2,gray2,CV_BGR2GRAY);//变为灰度图
// 添加特征点
if (addNewPoints())
{
points1.insert(points1.end(), corner.begin(), corner.end());
initial.insert(initial.end(), corner.begin(), corner.end());
}
calcOpticalFlowPyrLK (gray, gray2, points1, points2, status, err); //光流检测
// 去掉一些不好的特征点
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
// 显示特征点和运动轨迹
for (size_t i=0; i<points2.size(); i++)
{
line(frame2, initial[i], points2[i], Scalar(0, 0, 255));
circle(frame2, points2[i], 3, Scalar(0, 255, 0), -1);
}
imshow("KL", frame2);
}
}
//读取键盘输入操作
char c = (char)waitKey(10);
if( c == 27 )
break;
}
}
return 0;
}
//-------------------------------------------------------------------------------------------------
// function: addNewPoints
// brief: 检测新点是否应该被添加
// parameter:
// return: 是否被添加标志
//-------------------------------------------------------------------------------------------------
bool addNewPoints()
{
return points1.size() <= 10;
}
//-------------------------------------------------------------------------------------------------
// function: acceptTrackedPoint
// brief: 决定哪些跟踪点被接受
// parameter:
// return:
//-------------------------------------------------------------------------------------------------
bool acceptTrackedPoint(int i)
{
return status[i] && ((abs(points1[i].x - points2[i].x) + abs(points1[i].y - points2[i].y)) > 2);
}
*/

/*
		//找到轮廓最多的区域
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

		//找到跟踪区域
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
		//简单做法，把所有运动的点放在一个大矩形内
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
int count_max=1000;//最多保存点
int point_count=0;
CvRect rect;//追踪区域

findContours(image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // 查找轮廓                
Scalar color(rand()&255,rand()&255,rand()&255);
drawContours(dest_image, contours,-1,color,3);// 填充所有轮廓 

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
//	VideoCapture capture;//视频操作结构
//	int delay =20;//两帧的间隔时间，默认20
//	int current_frame_num=0;//当前帧数
//	Mat current_frame;//读取的帧图像
//	Mat image,image_gray;//当前处理图像,当前处理图像灰度图
//	Mat background,background_gray_cv8u;//背景图的灰度图，CV_8U格式背景图 
//	Mat diff_gray;//当前帧图像与背景图的差异
//	Mat output;//检测出的运动图像
//	double learningRate=0.2;//背景更新率
//
//	/******************************************/
//	/*根据输入的choice值，读取摄像头或视频数据*/
//	/******************************************/
//	char choice;
//	cout<<"***************请输入您的数据来源*****************"<<endl;
//	cout<<"1--打开电脑摄像头"<<endl;
//	cout<<"2--打开测试视频"<<endl;
//	cout<<"其他键--退出"<<endl;
//	cout<<"**********************************************"<<endl;
//	choice=getchar();
//	getchar();
//	if(choice == '1'){//打开电脑摄像头
//		delay = 20;//设置两帧间的间隔时间:
//		capture.open(0);
//	}else if(choice == '2'){//打开测试视频
//		capture.open("test.avi");//打开视频文件
//		double rate = capture.get(CV_CAP_PROP_FPS);//帧率
//		long total_frame_num = capture.get(CV_CAP_PROP_FRAME_COUNT);//获取总帧数
//		delay = 1000/rate;//设置两帧间的间隔时间:
//	}else{
//		cout<<"退出"<<endl;
//		return -1;
//	}
//	if( !capture.isOpened() ){
//		return -1;
//	}
//
//
//	//利用while 循环读取帧
//	while(1){
//		//读取一帧图像
//		if(choice == '1'){//摄像头数据
//			capture >> current_frame;
//		}else if(choice == '2'){//视频数据
//			capture.read(current_frame);
//		}else{
//			return -1;
//		}
//
//		if( !current_frame.empty()){
//			resize(current_frame, current_frame, Size(), 0.8, 0.8);
//			imshow("原视频",current_frame);
//			current_frame.copyTo(image);
//			//将图像转化成单通道图像处理
//			cvtColor(image, image_gray, CV_BGR2GRAY);
//
//			/******************************************/
//			/*背景建模法，检测结果保存在output中      */
//			/*运动的像素点值为255，背景像素点值为0    */
//			/******************************************/
//			if(current_frame_num == 0) {
//				//步骤1：若是第一帧，直接将图像作为背景图background
//				image_gray.convertTo(background,CV_32F); 
//			}else{
//				//步骤2：当前帧跟背景图相减
//				background.convertTo (background_gray_cv8u,CV_8U);
//				absdiff(image_gray, background_gray_cv8u, diff_gray);  
//				//步骤3：二值化前景图
//				threshold(diff_gray, output, 30, 255.0, CV_THRESH_BINARY);
//				//步骤4：更新背景图background，output作为掩码
//				accumulateWeighted (image_gray, background, learningRate, output);
//				imshow("背景差分法检测结果",output);//显示结果
//			}
//			++current_frame_num;//图片数+1
//		}
//
//		//读取键盘输入操作，点击esc退出
//		char c = (char)waitKey(delay);
//		if(c == 27)
//			break;//检测是否退出
//	}
//	/******************************************/
//	/*释放资源，退出                          */
//	/******************************************/
//	capture.release();//关闭视频文件
//	waitKey(0);
//	return 0;
//}

//RotatedRect motion_tracking(Rect &track_window,Mat image){
//
//	Mat frame, hsv;//image的HSV空间图像
//	Mat hue;//hsv的h分量
//	Mat mask;//原图二值化结果
//	Mat hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
//	int hsize = 16;
//	float hranges[] = {0,180};
//	const float* phranges = hranges;
//	int _vmin = vmin, _vmax = vmax;
//
//	if(debug)cout<<"motion_tracking track_window"<<track_window<<endl;
//	if(track_window.width>0&&track_window.height>0){
//		cvtColor(image, hsv, COLOR_BGR2HSV);//颜色空间转换函数,将RGB颜色转向HSV
//		//inRange函数的功能是检查输入数组每个元素大小是否在2个给定数值之间，可以有多通道,mask保存0通道的最小值，也就是h分量
//		//二值化结果保存在mask中
//		inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
//			Scalar(180, 256, MAX(_vmin, _vmax)), mask);
//
//		int ch[] = {0, 0};
//		hue.create(hsv.size(), hsv.depth());
//		mixChannels(&hsv, 1, &hue, 1, ch, 1);//将hsv第一个通道(也就是色调)的数复制到hue中
//
//		Mat roi(hue, track_window);//hue中用来计算直方图的区域,track_window为其感兴趣的区域 
//		Mat maskroi(mask, track_window);//roi对应的掩码，即roi对应maskroi值为1的点将用来计算直方图
//		calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);//将roi的0通道计算直方图放入hist中（以maskroi为掩码），hsize为每一维直方图的大小  
//		normalize(hist, hist, 0, 255, CV_MINMAX);//将hist矩阵进行数组范围归一化，都归一化到0~255
//
//		histimg = Scalar::all(0);//与按下'c'键是一样的，这里的all(0)表示的是标量全部清0
//		int binW = histimg.cols / hsize;//histing是一个200*300的矩阵，hsize应该是每一个bin的宽度，也就是histing矩阵能分出几个bin出来
//		Mat buf(1, hsize, CV_8UC3);//定义一个缓冲单bin矩阵
//		for( int i = 0; i < hsize; i++ )
//			buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);//Vec3b为3个char值的向量
//		cvtColor(buf, buf, CV_HSV2BGR);//将hsv又转换成bgr
//
//		for( int i = 0; i < hsize; i++ )
//		{
//			int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);//at函数为返回一个指定数组元素的参考值
//			rectangle( histimg, Point(i*binW,histimg.rows),
//				Point((i+1)*binW,histimg.rows - val),
//				Scalar(buf.at<Vec3b>(i)), -1, 8 );//在一幅输入图像上画一个简单抽的矩形，指定左上角和右下角，并定义颜色，大小，线型等 
//		}
//
//		//计算直方图的反向投影，计算hue图像0通道直方图hist的反向投影，并让入backproj中
//		calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
//		backproj &= mask;
//		//使用CamShift进行跟踪，更新track_window,TermCriteria这个类是作为迭代算法的终止条件,
//		RotatedRect trackBox = CamShift(backproj, track_window,
//			TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 5, 1 ));
//		if( track_window.area() <= 1 )
//		{
//			int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
//			track_window = Rect(track_window.x - r, track_window.y - r,
//				track_window.x + r, track_window.y + r) &
//				Rect(0, 0, cols, rows);//Rect函数为矩阵的偏移和大小，即第一二个参数为矩阵的左上角点坐标，第三四个参数为矩阵的宽和高
//		}
//
//		//cvtColor( backproj, image, COLOR_GRAY2BGR );
//		if(debug) imshow( "Histogram", histimg );
//
//		return trackBox;
//	}
//}
