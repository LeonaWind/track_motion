#include"motion_detection.h"

extern bool debug;

//-------------------------------------------------------------------------------------------------
// function: frame3_diff_motion_detection
// brief: 改进三帧差法检测运动物体
// parameter:第k-1帧图像Mat image_gray_pre，第k帧图像Mat image_gray，第k+1帧图像Mat image_gray_next
// return: 被跟踪区域Rect selection
//-------------------------------------------------------------------------------------------------
Rect frame3_diff_motion_detection(Mat image_gray_pre,Mat image_gray,Mat image_gray_next,Mat &background_gray_cv32f){
	int rows=image_gray.rows;
	int cols=image_gray.cols;
	Mat output(rows,cols,CV_8UC1);//检测出的运动图像
	int thres=10;

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
			//cout<<(int)diff_gray.at<uchar>(i,j)<<" ";
		}    
	} 


	//if(debug) imshow("形态学处理前",diff_gray);
	//进行分割
	Mat seed_fill;
	icvprCcaBySeedFill(diff_gray,seed_fill);
	//if(debug) imshow("分割结果",seed_fill);

	//3.形态学处理
	Mat element=getStructuringElement(MORPH_RECT,Size(15,15));
	morphologyEx(diff_gray,diff_gray,MORPH_CLOSE,element);
	//if(debug) imshow("形态学处理结果",diff_gray);
	//imshow("形态学处理结果",diff_gray);

	//4.第k帧背景建模
	/*Mat background_diff_gray;//当前图与背景图的差异
	Mat background_gray_cv8u;//CV_8U格式背景图 
	double learningRate=0.9;//学习率
	background_gray_cv32f.convertTo (background_gray_cv8u,CV_8U);

	absdiff(image_gray, background_gray_cv8u, background_diff_gray);//当前帧跟背景图相减  
	threshold(background_diff_gray, background_diff_gray, 40, 255.0, CV_THRESH_BINARY);//二值化前景图
	accumulateWeighted (image_gray,background_gray_cv32f,learningRate,background_diff_gray);//更新背景，output作为掩码

	//imshow("background_diff_gray",background_diff_gray);
	*/

	//5.第k帧canny边缘检测
	Mat image_gray_canny;
	Canny(image_gray, image_gray_canny, 3, 9, 3);
	//if(debug) imshow("image_gray_canny",image_gray_canny);

	//6.第k帧边缘检测结果与帧差法结果进行与运算
	mat_and(diff_gray,image_gray_canny,diff_gray);

	//7.与运算结果与背景建模结果进行或运算
	//mat_or(diff_gray,background_diff_gray,output);

	//8.形态学处理
	morphologyEx(diff_gray,output,MORPH_CLOSE,element);
	//if(debug) imshow("检测最后结果",output);

	//imshow("image_gray_canny",image_gray_canny);
	//imshow("diff_gray",diff_gray);
	Rect selection = get_track_selection_all(output); //简单做法获取追踪区域
	return selection;

}

//-------------------------------------------------------------------------------------------------
// function: background_motion_detection
// brief: 背景减法检测运动物体
// parameter:当前图像Mat image，当前图像灰度图Mat image_gray，CV_32F背景图像灰度图Mat background_gray
// return: 被跟踪区域Rect selection
//-------------------------------------------------------------------------------------------------
Rect background_motion_detection(Mat &image_gray,Mat &background_gray_cv32f){
	Mat diff_gray;//当前图与背景图的差异
	Mat output;//检测出的运动图像
	double learningRate=0.9;//学习率
	Mat background_gray_cv8u;//CV_8U格式背景图 
	background_gray_cv32f.convertTo (background_gray_cv8u,CV_8U);

	absdiff(image_gray, background_gray_cv8u, diff_gray);//当前帧跟背景图相减  
	threshold(diff_gray, output, 30, 255.0, CV_THRESH_BINARY);//二值化前景图
	accumulateWeighted (image_gray,background_gray_cv32f,learningRate,output);//更新背景，output作为掩码

	//imshow("foreground",output);
	//imshow("image_gray",image_gray);
	//imshow("background", background_gray_cv8u); 

	Rect selection = get_track_selection(output); 
	return selection;

}

//-------------------------------------------------------------------------------------------------
// function: get_track_selection_all
// brief: 获取追踪区域,简单做法：把所有运动的点放在一个大矩形内
// parameter:输入图像Mat image
// return: 被跟踪区域Rect selection
//-------------------------------------------------------------------------------------------------
/*Rect get_track_selection_all(Mat &image){

vector<vector<Point>> contours;
vector<Vec4i> hierarchy;
Mat dest_image=Mat::zeros(image.rows,image.cols,CV_8UC3);
int count_max=1000;//最多保存点
int point_count=0;
CvRect rect;//追踪区域

findContours(image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // 查找轮廓                
Scalar color(rand()&255,rand()&255,rand()&255);
drawContours(dest_image, contours,-1,color,3);// 填充所有轮廓 

vector<Point> all_contours;
for (vector<vector<Point>>::const_iterator iter = contours.begin(); iter != contours.end(); iter++)
{
for (vector<Point>::const_iterator inner_iter = (*iter).begin(); inner_iter != (*iter).end(); inner_iter++){
Point x;
x.x=(*inner_iter).x;
x.y=(*inner_iter).y;
all_contours.push_back(x);
point_count++;
if(point_count>=count_max) break;
}
}  
if(!all_contours.empty()){
rect = boundingRect(all_contours); 
rectangle(dest_image, cvPoint(rect.x, rect.y), cvPoint(rect.x + rect.width, rect.y + rect.height),CV_RGB(255,255, 255), 1, 8, 0);
}

imshow("get_track_selection",dest_image);  
return rect;

}*/
Rect get_track_selection_all(Mat &image){

	int rows=image.rows;
	int cols=image.cols;
	vector<Point> all_contours;
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
	return rect;
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

//-------------------------------------------------------------------------------------------------
// function: mat_or
// brief: 两个矩阵进行或运算
// parameter:输入图像Mat src1,Mat src2，与运算结果Mat &dst
// return: 空，或运算结果保存在dst中
//-------------------------------------------------------------------------------------------------
void icvprCcaBySeedFill(Mat& _binImg,Mat& _lableImg)  
{  
	// connected component analysis (4-component)  
	// use seed filling algorithm  
	// 1. begin with a foreground pixel and push its foreground neighbors into a stack;  
	// 2. pop the top pixel on the stack and label it with the same label until the stack is empty  
	//   
	// foreground pixel: _binImg(x,y) = 1  
	// background pixel: _binImg(x,y) = 0  


	if (_binImg.empty() ||  
		_binImg.type() != CV_8UC1)  
	{  
		return ;  
	}  

	_binImg.convertTo(_lableImg, CV_32SC1) ;  

	int label = 1 ;  // start by 2  
	int sign=255;

	int rows = _binImg.rows - 1 ;  
	int cols = _binImg.cols - 1 ;  
	for (int i = 1; i < rows-1; i++)  
	{  
		int* data= _lableImg.ptr<int>(i) ;  
		for (int j = 1; j < cols-1; j++)  
		{  
			if (data[j] == sign)  
			{  
				std::stack<std::pair<int,int>> neighborPixels ;     
				neighborPixels.push(std::pair<int,int>(i,j)) ;     // pixel position: <i,j>  
				++label;  // begin with a new label  
				while (!neighborPixels.empty())  
				{  
					// get the top pixel on the stack and label it with the same label  
					std::pair<int,int> curPixel = neighborPixels.top() ;  
					int curX = curPixel.first ;  
					int curY = curPixel.second ;  
					_lableImg.at<int>(curX, curY) = label ;  

					// pop the top pixel  
					neighborPixels.pop() ;  

					// push the 4-neighbors (foreground pixels)  
					if (_lableImg.at<int>(curX, curY-1) == sign)  
					{// left pixel  
						neighborPixels.push(std::pair<int,int>(curX, curY-1)) ;  
					}  
					if (_lableImg.at<int>(curX, curY+1) == sign)  
					{// right pixel  
						neighborPixels.push(std::pair<int,int>(curX, curY+1)) ;  
					}  
					if (_lableImg.at<int>(curX-1, curY) == sign)  
					{// up pixel  
						neighborPixels.push(std::pair<int,int>(curX-1, curY)) ;  
					}  
					if (_lableImg.at<int>(curX+1, curY) == sign)  
					{// down pixel  
						neighborPixels.push(std::pair<int,int>(curX+1, curY)) ;  
					}  
				}         
			}  
		}  
	} 
	_lableImg.convertTo(_lableImg, CV_8UC1);
	imshow("_lableImg",_lableImg);

	Mat show_img(rows,cols,CV_8UC3);	
	Scalar *show_color=new Scalar[label];
	int show_rows=show_img.rows;
	int show_cols=show_img.cols;
	for(int k=0;k<label;k++){
		show_color[k]=Scalar(rand()&255,rand()&255,rand()&255);
	}
	//生成随机颜色
	//显示结果
	for (int i = 0;i<show_rows;i++)  
	{  
		int* data= _lableImg.ptr<int>(i); 
		for (int j = 0;j<show_cols;j++)  {  
			if (data[j] >0&&data[j]<label){
				cout<<data[j]<<endl;
				show_img.at<cv::Vec3b>(i, j)[0]=show_color[data[j]].val[0];
				show_img.at<cv::Vec3b>(i, j)[1]=show_color[data[j]].val[1];
				show_img.at<cv::Vec3b>(i, j)[2]=show_color[data[j]].val[2];
			}
			else{
				_lableImg.at<int>(i, j) = 0; 
			}
		}

	}
	imshow("show_img",show_img);
}
