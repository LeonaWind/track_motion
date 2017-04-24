#include"motion_segment.h"
extern bool debug;

//����ָ�
void motion_segment(Mat image){
	Mat binary;
	Mat foreground;
	Mat background;
	Mat result;

	threshold(image,binary, 40, 255.0, CV_THRESH_BINARY);//��ֵ��
	//����ֵ���Ķ�ֵͼ����и�ʴ��ȥ��С�İ�ɫ���򣬵õ�ͼ���ǰ�����򡣲���ǰ��������255��ɫ���
	Mat element=getStructuringElement(MORPH_RECT,Size(3,3));//��ʴ�õ�ǰ��ͼ��
	morphologyEx(binary,foreground,MORPH_ERODE,element);
	imshow("foreground",foreground);

	//ͬ������ֵ�����ͼ��������ͣ�Ȼ������ֵ����ȡ�����õ��������򡣲���128�Ҷȱ�ʾ
	morphologyEx(binary,background,MORPH_DILATE,element);//����
	//ȡ��
	imshow("background",background);

	//��ǰ���ͱ���������һ����ͬһ��ͼ������ʾ


	//�ñ��ͼ��ԭͼ������opencv��watershed��ͼ����зָ�


}