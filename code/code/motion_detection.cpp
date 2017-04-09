#include"motion_detection.h"

//-------------------------------------------------------------------------------------------------
// function: motion_detection
// brief: �˶����
// parameter:��ǰͼ��Mat image����ǰͼ��Ҷ�ͼMat image_gray��CV_32F����ͼ��Ҷ�ͼMat background_gray
// return: ����������Rect selection
//-------------------------------------------------------------------------------------------------
Rect motion_detection(Mat &image_gray,Mat &background_gray_cv32f){
	Mat diff_gray;//��ǰͼ�뱳��ͼ�Ĳ���
	Mat output;//�������˶�ͼ��
	double learningRate=0.8;//ѧϰ��
	Mat background_gray_cv8u;//CV_8U��ʽ����ͼ 
	background_gray_cv32f.convertTo (background_gray_cv8u,CV_8U);

	GaussianBlur(image_gray, image_gray, cv::Size(0,0), 3, 0, 0);//������˹�˲�����ƽ��ͼ�� 
	absdiff(image_gray, background_gray_cv8u, diff_gray);//��ǰ֡������ͼ���  
	threshold(diff_gray, output, 30, 255.0, CV_THRESH_BINARY);//��ֵ��ǰ��ͼ
	//image_gray.copyTo(background_gray);//���±���ͼ
	accumulateWeighted (image_gray,background_gray_cv32f,learningRate,output);//���±�����output��Ϊ����
	
	//imshow("foreground",output);
	//imshow("image_gray",image_gray);
	//imshow("background", background_gray_cv8u); 

	Rect selection = get_track_selection(output); 
	return selection;

}

//-------------------------------------------------------------------------------------------------
// function: get_track_selection
// brief: ��ȡ׷������
// parameter:����ͼ��Mat image
// return: ����������Rect selection
//-------------------------------------------------------------------------------------------------
Rect get_track_selection(Mat &image)     
{     
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	Mat dest_image=Mat::zeros(image.rows,image.cols,CV_8UC3);
	int point_count_max=0,contours_count_max=0;//��¼Ҫ��ȡ�ĵ�
	int point_count=0,contours_count=0;
	CvRect rect;//׷������

	if (!image.empty())     
	{           
		findContours(image, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE); // ��������                
		Scalar color(rand()&255,rand()&255,rand()&255);
		drawContours(dest_image, contours,-1,color,3);// ����������� 

		//�ҵ�������������
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

		//�ҵ���������
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
		//���������������˶��ĵ����һ���������
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