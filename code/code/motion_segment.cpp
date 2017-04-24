#include"motion_segment.h"
extern bool debug;

//物体分割
void motion_segment(Mat image){
	Mat binary;
	Mat foreground;
	Mat background;
	Mat result;

	threshold(image,binary, 40, 255.0, CV_THRESH_BINARY);//二值化
	//对阈值化的二值图像进行腐蚀，去掉小的白色区域，得到图像的前景区域。并对前景区域用255白色标记
	Mat element=getStructuringElement(MORPH_RECT,Size(3,3));//腐蚀得到前景图像
	morphologyEx(binary,foreground,MORPH_ERODE,element);
	imshow("foreground",foreground);

	//同样对阈值化后的图像进行膨胀，然后再阈值化并取反。得到背景区域。并用128灰度表示
	morphologyEx(binary,background,MORPH_DILATE,element);//膨胀
	//取反
	imshow("background",background);

	//将前景和背景叠加在一起在同一幅图像中显示


	//用标记图和原图，利用opencv的watershed对图像进行分割


}