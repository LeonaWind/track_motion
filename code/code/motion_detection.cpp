#include"motion_detection.h"

//-------------------------------------------------------------------------------------------------
// function: motion_detection
// brief: 运动检测
// parameter:当前图像Mat image，当前图像灰度图Mat image_gray，CV_32F背景图像灰度图Mat background_gray
// return: 被跟踪区域Rect selection
//-------------------------------------------------------------------------------------------------
Rect motion_detection(Mat &image_gray,Mat &background_gray_cv32f){
	Mat diff_gray;//当前图与背景图的差异
	Mat output;//检测出的运动图像
	double learningRate=0.8;//学习率
	Mat background_gray_cv8u;//CV_8U格式背景图 
	background_gray_cv32f.convertTo (background_gray_cv8u,CV_8U);

	GaussianBlur(image_gray, image_gray, cv::Size(0,0), 3, 0, 0);//先做高斯滤波，以平滑图像 
	absdiff(image_gray, background_gray_cv8u, diff_gray);//当前帧跟背景图相减  
	threshold(diff_gray, output, 30, 255.0, CV_THRESH_BINARY);//二值化前景图
	//image_gray.copyTo(background_gray);//更新背景图
	accumulateWeighted (image_gray,background_gray_cv32f,learningRate,output);//更新背景，output作为掩码
	
	//imshow("foreground",output);
	//imshow("image_gray",image_gray);
	//imshow("background", background_gray_cv8u); 

	Rect selection = get_track_selection(output); 
	return selection;

}

//-------------------------------------------------------------------------------------------------
// function: get_track_selection
// brief: 获取追踪区域
// parameter:输入图像Mat image
// return: 被跟踪区域Rect selection
//-------------------------------------------------------------------------------------------------
Rect get_track_selection(Mat &image)     
{     
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	Mat dest_image=Mat::zeros(image.rows,image.cols,CV_8UC3);
	int point_count_max=0,contours_count_max=0;//记录要获取的点
	int point_count=0,contours_count=0;
	CvRect rect;//追踪区域

	if (!image.empty())     
	{           
		findContours(image, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE); // 查找轮廓                
		Scalar color(rand()&255,rand()&255,rand()&255);
		drawContours(dest_image, contours,-1,color,3);// 填充所有轮廓 

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
				if(point_count>=point_count_max) break;
			}
		}  
		if(!all_contours.empty()){
			rect = boundingRect(all_contours); 
			rectangle(dest_image, cvPoint(rect.x, rect.y), cvPoint(rect.x + rect.width, rect.y + rect.height),CV_RGB(255,255, 255), 1, 8, 0);
		}
		*/
	}  
	imshow("get_track_selection",dest_image);  
	return rect;
}  