#ifndef MOTION_SEGMENT_H
#define MOTION_SEGMENT_H

#include <opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <iomanip>
#include <cxcore.h>
#include <stack>
using namespace cv;
using namespace std;

Mat motion_segment(Mat image);//物体分割
vector<Rect> icvprCcaBySeedFill(Mat& _binImg,Mat& _lableImg);
vector<Rect> get_track_selection_many(Mat detection_image,Mat segment_image);
vector<Rect> get_track_selection_many_simple(Mat image);

class WatershedSegmenter {
  private:
	  Mat markers;//标记图，水淹过程就是从这组预先定义好的标记像素开始
  public:
	  void setMarkers(const Mat& markerImage);//设置初始化的标记图像
	  Mat process(const Mat &image);//对图片进行分水岭算法分割
	  Mat getSegmentation();//返回分割后的图像
	  Mat getWatersheds();//以图像的形式返回分水岭
};

#endif