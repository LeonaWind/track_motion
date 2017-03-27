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
