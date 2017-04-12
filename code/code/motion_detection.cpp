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
	int count_max=1000;//最多保存点
	int point_count_max=0,contours_count_max=0;//记录要获取的点
	int point_count=0,contours_count=0;
	CvRect rect;//追踪区域

	if (!image.empty())     
	{           
		findContours(image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // 查找轮廓                
		//Scalar color(rand()&255,rand()&255,rand()&255);
		//drawContours(dest_image, contours,-1,color,3);// 填充所有轮廓 
		if(!contours.empty()){
			//聚类方法找到跟踪区域
			//获取样本点的大小，保存在point_count中
			for (vector<vector<Point>>::const_iterator iter = contours.begin(); iter != contours.end(); iter++)
			{
				point_count+=(*iter).size();
			}

			Scalar colorTab[] =     //因为最多只有5类，所以最多也就给5个颜色  
			{  
				Scalar(0, 0, 255),  
				Scalar(0, 255, 0),  
				Scalar(255, 100, 100),  
				Scalar(255, 0, 255),  
				Scalar(0, 255, 255)  
			};  
			int clusterCount=4;//聚类大小
			int* clusterIdxArr =  new int[clusterCount];
			double* clusterIdxDis =  new double[clusterCount];//保存平均距离
			clusterCount = MIN(clusterCount, point_count); 
			Mat points(point_count, 1, CV_32FC2,Scalar::all(0));//points为输入矩阵，其中存储的是采样点
			Mat labels;//labels中放的是执行kmeans算法后points中采样点的簇的索引
			Mat centers(clusterCount, 1, points.type());//用来存储聚类后的中心点
			float* dataDst=points.ptr<float>(0);;//dataDst指向points数据

			//初始化
			for(int i = 0;i < clusterCount;i++)
			{ 
				clusterIdxArr[i]=0;
				clusterIdxDis[i]=0.0;
			} 

			//将样本点的坐标存入points中
			int i = 0;
			for (vector<vector<Point>>::const_iterator iter = contours.begin(); iter != contours.end(); iter++)
			{
				for (vector<Point>::const_iterator inner_iter = (*iter).begin(); inner_iter != (*iter).end(); inner_iter++){
					dataDst = points.ptr<float>(i);
					dataDst[0] = (double)(*inner_iter).x;
					dataDst[1] = (double)(*inner_iter).y;
					i++;

				}
			}
			//cout <<points<<endl;

			//聚类3次，取结果最好的那次，聚类的初始化采用PP特定的随机算法。
			kmeans(points, clusterCount, labels, TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0),  3, KMEANS_PP_CENTERS, centers);

			//去除多余的点

			//选取点
			vector<Point> all_contours;
			for(i = 0;i < clusterCount;i++)//标出中心点
			{ 
				Point ipt = Point(centers.at<float>(i*2), centers.at<float>(i*2+1)); 
				circle(dest_image, ipt, 4, Scalar(100,100,100),CV_FILLED, CV_AA);

			} 
			for(int i = 0; i < point_count; i++ ) 
			{ 
				int clusterIdx = labels.at<int>(i);  
				++clusterIdxArr[clusterIdx];
				Point2f ipt;  

				dataDst = points.ptr<float>(i);
				ipt.x=dataDst[0];
				ipt.y=dataDst[1];
				circle(dest_image, ipt, 1, colorTab[clusterIdx], CV_FILLED, CV_AA);   
				//cout<<i<<"      "<<ipt.x<<","<<ipt.y<<"------"<<clusterIdx<<endl;
			}

			//找出最多的聚类,将他的点保存在all_contours中
			int max_clusterCount=0;
			int temp=0;
			for (int i=0;i<clusterCount;i++){
				if(clusterIdxArr[i]>temp){
					temp=clusterIdxArr[i];
					max_clusterCount=i;
				}
			}

			for(int i = 0; i < point_count; i++ ) 
			{ 
				int clusterIdx = labels.at<int>(i);  
				if(clusterIdx == max_clusterCount){
					Point x;
					dataDst = points.ptr<float>(i);
					x.x=dataDst[0];
					x.y=dataDst[1];
					all_contours.push_back(x);
				}
			}

			if(!all_contours.empty()&&!centers.empty()){
				Point max_center_point = Point(centers.at<float>(max_clusterCount*2), centers.at<float>(max_clusterCount*2+1));
				//计算均值
				Point first_point=all_contours[0];
				double dis_avge=sqrt(pow((max_center_point.x-first_point.x),2)+pow((max_center_point.y-first_point.y),2));
				for (vector<Point>::const_iterator iter = all_contours.begin(); iter != all_contours.end(); iter++){
					double dis_temp=sqrt(pow((max_center_point.x-(*iter).x),2)+pow((max_center_point.y-(*iter).y),2));
					dis_avge=(dis_avge+dis_temp)/2;
					//cout<<"dis_avge"<<dis_avge<<endl;
				}

				//去除多余的点
				for (vector<Point>::const_iterator iter = all_contours.begin(); iter != all_contours.end();){
					double dis_temp=sqrt(pow((max_center_point.x-(*iter).x),2)+pow((max_center_point.y-(*iter).y),2));
					if(dis_temp>dis_avge){
						iter=all_contours.erase(iter);
						//cout<<"erase"<<dis_temp<<endl;
					}else{
						iter++;
					}
				}
			}

			//划分跟踪区域
			if(!all_contours.empty()){
				rect = boundingRect(all_contours); 
				rectangle(dest_image, cvPoint(rect.x, rect.y), cvPoint(rect.x + rect.width, rect.y + rect.height),CV_RGB(255,255, 255), 1, 8, 0);
			}

		}
	}  
	imshow("get_track_selection",dest_image);  
	return rect;
}  

Rect rectA_intersect_rectB(Rect rectA, Rect rectB){
	float start_min_x=min(rectA.x,rectB.x);
	float start_max_x=max(rectA.x,rectB.x);
	float start_min_y=min(rectA.y,rectB.y);
	float start_max_y=max(rectA.y,rectB.y);

	float end_min_x=min(rectA.x+rectA.width,rectB.x+rectB.width);
	float end_max_x=max(rectA.x+rectA.width,rectB.x+rectB.width);
	float end_min_y=min(rectA.y+rectA.height,rectB.y+rectB.height);
	float end_max_y=max(rectA.y+rectA.height,rectB.y+rectB.height);

	Rect result;

	if(end_min_x<start_max_x||end_min_y<start_max_y){
		return rectA;
	}

	result.x=start_max_x;
	result.y=start_max_y;
	result.width=end_min_x-start_max_x;
	result.height=end_min_y-start_max_y;

	return result;
}


