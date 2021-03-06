/*
文件名:motion_detection.cpp
作者:甘晓蓉 
编写日期:2017-5
文件描述:运动检测模块，包含运动检测中使用的函数
*/
#include"motion_detection.h"

extern bool debug;

//-------------------------------------------------------------------------------------------------
// function: frame3_diff_motion_detection
// brief: 改进三帧差法结合背景差分法检测运动物体
// parameter:第k-1帧图像Mat image_gray_pre，第k帧图像Mat image_gray，第k+1帧图像Mat image_gray_next
//			背景图像background_gray_cv32f
// return: 运动检测结果图像Mat output,运动像素值为255，背景像素值为0
//-------------------------------------------------------------------------------------------------
Mat frame3_diff_motion_detection(Mat image_gray_pre,Mat image_gray,Mat image_gray_next,Mat &background_gray_cv32f){
	int rows=image_gray.rows;
	int cols=image_gray.cols;
	Mat output(rows,cols,CV_8UC1);//检测出的运动图像
	int thres=15;

	//1.两两图像做差分
	Mat diff_gray(rows,cols,CV_8UC1);
	Mat diff_gray1, diff_gray2, diff_gray3;
	absdiff(image_gray_pre, image_gray, diff_gray1);
	absdiff(image_gray, image_gray_next, diff_gray2);
	absdiff(image_gray_next, image_gray_pre, diff_gray3);

	//2.求均值
	for(int i=0;i<rows;i++)  {  
		for(int j=0;j<cols;j++)  {    
			diff_gray.at<uchar>(i,j) = (diff_gray1.at<uchar>(i,j)+diff_gray2.at<uchar>(i,j)+diff_gray3.at<uchar>(i,j))/3; 
			if(diff_gray.at<uchar>(i,j)>thres){//二值化
				diff_gray.at<uchar>(i,j) = 255;
			}else{
				diff_gray.at<uchar>(i,j) = 0;
			}
		}    
	} 
	if(debug) imshow("差分结果",diff_gray);
	waitKey(30);

	//3.形态学处理
	Mat element(6,6,CV_8U,Scalar(1));
	morphologyEx(diff_gray,diff_gray,MORPH_CLOSE,element);
	if(debug) imshow("形态学处理结果",diff_gray);

	//4.第k帧背景建模
	/*Mat background_diff_gray(rows,cols,CV_8UC1);//当前图与背景图的差异
	Mat background_gray_cv8u;//CV_8U格式背景图 
	double learningRate=0.8;//学习率
	background_gray_cv32f.convertTo (background_gray_cv8u,CV_8U);

	absdiff(image_gray, background_gray_cv8u, background_diff_gray);//当前帧跟背景图相减  
	threshold(background_diff_gray, background_diff_gray, 40, 255.0, CV_THRESH_BINARY);//二值化前景图
	accumulateWeighted (image_gray,background_gray_cv32f,learningRate,background_diff_gray);//更新背景，output作为掩码
	*/

	//5.第k帧canny边缘检测
	Mat image_gray_canny;
	Canny(image_gray, image_gray_canny, 3, 9, 3);

	//6.第k帧边缘检测结果与帧差法结果进行与运算
	mat_and(diff_gray,image_gray_canny,diff_gray);

	//7.与运算结果与背景建模结果进行或运算
	//mat_or(diff_gray,background_diff_gray,output);
	output=diff_gray;

	//8.形态学处理
	Mat element1(5,5,CV_8U,Scalar(1));
	morphologyEx(output,output,MORPH_CLOSE,element1);
	if(debug){
		imshow("运动检测算法结果",output);
		waitKey(30);
	}

	return output;//返回运动检测结果图像，运动像素值为255，背景像素值为0
}


//-------------------------------------------------------------------------------------------------
// function: get_track_selection_all
// brief: 获取追踪区域,简单做法：把所有运动的点放在一个大矩形内
// parameter:输入图像Mat image
// return: 被跟踪区域vector<Rect> result
//-------------------------------------------------------------------------------------------------
vector<Rect> get_track_selection_all(Mat &image){

	int rows=image.rows;
	int cols=image.cols;
	vector<Point> all_contours;
	vector<Rect> result;
	Rect rect;//追踪区域

	for (int i = 0; i < rows; i++){  
		for (int j = 0; j < cols; j++){
			if(image.at<uchar>(i,j)>0){
				Point x;
				x.x=j;
				x.y=i;
				all_contours.push_back(x);
			}
		}
	} 

	if(!all_contours.empty()){
		rect = boundingRect(all_contours); 
		rectangle(image, cvPoint(rect.x, rect.y), cvPoint(rect.x + rect.width, rect.y + rect.height),CV_RGB(255,255, 255), 1, 8, 0);
	}

	if(debug) imshow("改进三帧差法",image);
	result.push_back(rect);
	return result;
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
				Scalar(255, 255, 255),  
				Scalar(0, 255, 0),  
				Scalar(255, 100, 100),  
				Scalar(255, 0, 255),  
				Scalar(0, 255, 255)  
			};  
			int clusterCount=1;//聚类大小
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
	if(debug) imshow("背景建模法",dest_image);

	return rect;
}  


//-------------------------------------------------------------------------------------------------
// function: mat_and
// brief: 两个矩阵进行与运算
// parameter:输入图像Mat src1,Mat src2，与运算结果Mat &dst
// return: 空，与运算结果保存在dst中
//-------------------------------------------------------------------------------------------------
void mat_and(Mat src1,Mat src2,Mat &dst){
	int rows=src1.rows;
	int cols=src1.cols;
	if(src2.rows != rows || src2.cols != cols){
		cout<<"两个矩阵尺寸不相等"<<endl;
		return;
	}
	for (int i = 0; i < rows; i++){  
		for (int j = 0; j < cols; j++){
			//cout<<(int)src1.at<uchar>(i,j)<<" "<<(int)src2.at<uchar>(i,j)<<endl;
			dst.at<uchar>(i,j) = src1.at<uchar>(i,j)&src2.at<uchar>(i,j);
			if(dst.at<uchar>(i,j)==1) dst.at<uchar>(i,j)=255;
		}
	} 
}

//-------------------------------------------------------------------------------------------------
// function: mat_or
// brief: 两个矩阵进行或运算
// parameter:输入图像Mat src1,Mat src2，与运算结果Mat &dst
// return: 空，或运算结果保存在dst中
//-------------------------------------------------------------------------------------------------
void mat_or(Mat src1,Mat src2,Mat &dst){
	int rows=src1.rows;
	int cols=src1.cols;
	if(src2.rows != rows || src2.cols != cols){
		cout<<"两个矩阵尺寸不相等"<<endl;
		return;
	}
	for (int i = 0; i < rows; i++){  
		for (int j = 0; j < cols; j++){
			//cout<<(int)src1.at<uchar>(i,j)<<" "<<(int)src2.at<uchar>(i,j)<<endl;
			dst.at<uchar>(i,j) = src1.at<uchar>(i,j)|src2.at<uchar>(i,j);
			if(dst.at<uchar>(i,j)==1) dst.at<uchar>(i,j)=255;
		}
	} 
}
