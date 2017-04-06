#include"motion_detection.h"

//运动检测
Rect motion_detection(Mat &image,Mat &image_gray,Mat &background_gray){
	Mat  diff_gray,for_image;
	GaussianBlur(image_gray, image_gray, cv::Size(0,0), 3, 0, 0);//先做高斯滤波，以平滑图像 
	absdiff(image_gray, background_gray, diff_gray);//当前帧跟背景图相减  
	threshold(diff_gray, image, 10, 255.0, CV_THRESH_BINARY);//二值化前景图
	image_gray.copyTo(background_gray);//更新背景图

	Rect selection = get_track_selection(image); 
	return selection;

}

//获取追踪区域
Rect get_track_selection(Mat &image)     
{     
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	Mat dest_image=Mat::zeros(image.rows,image.cols,CV_8UC3);
	int point_count_max=1000;
	int point_count=0;
	CvRect rect;//追踪区域

	if (!image.empty())     
	{           
		findContours(image, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE); // 查找轮廓                
		Scalar color(rand()&255,rand()&255,rand()&255);
		drawContours(dest_image, contours,-1,color,3);// 填充所有轮廓 

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
				if(point_count>=point_count_max) break;

			}
		}  
		if(!all_contours.empty()){
			rect = boundingRect(all_contours); 
			rectangle(dest_image, cvPoint(rect.x, rect.y), cvPoint(rect.x + rect.width, rect.y + rect.height),CV_RGB(255,255, 255), 1, 8, 0);
		}
	}  
	imshow("result",dest_image);  
	return rect;
}  