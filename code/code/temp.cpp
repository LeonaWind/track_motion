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