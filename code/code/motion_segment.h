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
	  Mat markers;//���ͼ��ˮ�͹��̾��Ǵ�����Ԥ�ȶ���õı�����ؿ�ʼ
  public:
	  void setMarkers(const Mat& markerImage);//���ó�ʼ���ı��ͼ��
	  Mat process(const Mat &image);//��ͼƬ���з�ˮ���㷨�ָ�
	  Mat getSegmentation();//���طָ���ͼ��
	  Mat getWatersheds();//��ͼ�����ʽ���ط�ˮ��
};

#endif