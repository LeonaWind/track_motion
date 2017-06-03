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

Mat motion_segment(Mat image);//����ָ�
vector<Rect> icvprCcaBySeedFill(Mat& _binImg,Mat& _lableImg);
vector<Rect> get_track_selection_many(Mat detection_image,Mat segment_image);
vector<Rect> get_track_selection_many_simple(Mat image);

class WatershedSegmenter {
  private:
	  Mat markers;
  public:
	  void setMarkers(const Mat& markerImage) {
		// Convert to image of ints
		markerImage.convertTo(markers,CV_32S);
	  }

	  Mat process(const Mat &image){
		// Apply watershed
		watershed(image,markers);
		return markers;
	  }

	  // Return result in the form of an image
	  Mat getSegmentation(){
		Mat tmp;
		// all segment with label higher than 255
		// will be assigned value 255
		markers.convertTo(tmp,CV_8U);
		return tmp;
	  }

	  // Return watershed in the form of an image��ͼ�����ʽ���ط�ˮ��
	  Mat getWatersheds(){
		Mat tmp;
		//�ڱ任ǰ����ÿ������pת��Ϊ255p+255����conertTo��ʵ�֣�
		markers.convertTo(tmp,CV_8U,255,255);
		return tmp;
	  }
};

#endif