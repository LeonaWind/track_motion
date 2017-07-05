/*
文件名:motion_segment.cpp
作者:甘晓蓉 
编写日期:2017-5
文件描述:轮廓分割模块，包含对图像进行物体轮廓分割时使用的函数
*/
#include"motion_segment.h"

extern bool debug;

//-------------------------------------------------------------------------------------------------
// function: motion_segment
// brief:运用分水岭算法检测出图像中各物体的轮廓
// parameter:待分割图像Mat image
// return:图像分割结果Mat output,前景像素值为255，背景像素值为128
//-------------------------------------------------------------------------------------------------
Mat motion_segment(Mat image){
	Mat result;
	int rows=image.rows;
	int cols=image.cols;

	if (!image.data)
		return result;

	//1.阈值分割原图的灰度图，获得二值图像binary
	Mat binary;
	cvtColor(image,binary,COLOR_BGRA2GRAY);
	threshold(binary,binary,100,255,THRESH_BINARY_INV);

	//足球运动员删除最上面的观众部分
	for (int i = 0; i < rows; i++){  
		for (int j = 0; j < 100; j++){
			binary.at<uchar>(i,j)=255;
		}
	} 

	//2.前景图像进行闭运算
	Mat fg1;
	Mat element5(8,8,CV_8U,Scalar(1));//5*5正方形，8位uchar型，全1结构元素
	morphologyEx(binary, fg1,MORPH_CLOSE,element5);

	//3.背景图像进行膨胀运算
	Mat bg1;
	dilate(binary,bg1,Mat());
	threshold(bg1,bg1,1,128,THRESH_BINARY_INV);//>=1的像素设置为128（即背景）

	//4.形态学处理后的前景图fg1和背景图bg1做与运算
	Mat markers1 = fg1 + bg1; //使用Mat类的重载运算符+来合并图像。

	//5.使用分水岭算法进行分割
	WatershedSegmenter segmenter1;//实例化一个分水岭分割方法的对象
	segmenter1.setMarkers(markers1);//设置算法的标记图像，使得水淹过程从这组预先定义好的标记像素开始
	segmenter1.process(image);//传入待分割原图

	//6.分割结果做闭运算
	//将修改后的标记图markers转换为可显示的8位灰度图并返回分割结果（白色为前景，灰色为背景，0为边缘）
	result=segmenter1.getSegmentation();
	Mat element=getStructuringElement(MORPH_RECT,Size(8,8));
	morphologyEx(result,result,MORPH_CLOSE,element);
	if(debug){
		imshow("轮廓分割结果",result);
		waitKey(30);
	}

	return result;//返回分割出的各个物体，前景为255，背景为128
}

//-------------------------------------------------------------------------------------------------
// function: icvprCcaBySeedFill
// brief:运用种子填充法进行连通区域分析
// parameter:待连通图像Mat con_image，标记图Mat label_image
// return:各个独立的运动检测结果vector<Rect> track_rect，保存了需要跟踪对象的坐标和大小
//-------------------------------------------------------------------------------------------------
vector<Rect> icvprCcaBySeedFill(Mat& con_image,Mat& label_image)  
{  
	vector<Rect> track_rect;
	int label = 1 ;  // start by 1  
	int sign=255;

	int rows = con_image.rows - 1 ;  
	int cols = con_image.cols - 1 ; 
	if (con_image.empty()||con_image.type() != CV_8UC1){  
		return track_rect;  
	}  

	//1.连通区域分析，将相邻位置的点标记为同一个值，存在label_image中
	con_image.convertTo(label_image, CV_32SC1) ;  
	for (int i = 1; i < rows-1; i++)  {  
		int* data= label_image.ptr<int>(i) ;  
		for (int j = 1; j < cols-1; j++)  {  
			//遍历找到一个前景像素点
			if (data[j] == sign)  {  
				std::stack<std::pair<int,int>> neighborPixels ;     
				neighborPixels.push(std::pair<int,int>(i,j)) ;
				++label;
				while (!neighborPixels.empty())  {  
					std::pair<int,int> curPixel = neighborPixels.top() ;  
					int cur_x = curPixel.first ;  
					int cur_y = curPixel.second ;  
					label_image.at<int>(cur_x, cur_y) = label ;  
					neighborPixels.pop() ;  

					if(cur_x==0||cur_y==0||cur_x==rows||cur_y==cols) continue;

					//将相邻的前景像素点入栈
					if (label_image.at<int>(cur_x, cur_y-1) == sign){//左像素
						neighborPixels.push(std::pair<int,int>(cur_x, cur_y-1)) ;  
					}  
					if (label_image.at<int>(cur_x, cur_y+1) == sign){//右像素
						neighborPixels.push(std::pair<int,int>(cur_x, cur_y+1)) ;  
					}  
					if (label_image.at<int>(cur_x-1, cur_y) == sign){//上像素
						neighborPixels.push(std::pair<int,int>(cur_x-1, cur_y)) ;  
					}  
					if (label_image.at<int>(cur_x+1, cur_y) == sign){//下像素
						neighborPixels.push(std::pair<int,int>(cur_x+1, cur_y)) ;  
					}  
				}         
			}  
		}  
	} 
	label_image.convertTo(label_image, CV_8UC1);

	//2.获取追踪区域，根据label_image的值划定Rect区域
	vector<Point> all_contours;
	Rect rect;//追踪区域
	for(int k=1;k<=label;k++){
		//将同一个label值的像素点放入同一个点集，all_contours
		for (int i = 0; i < rows; i++){  
			for (int j = 0; j < cols; j++){
				if(label_image.at<uchar>(i,j)==k){
					Point x;
					x.x=j;
					x.y=i;
					all_contours.push_back(x);
				}
			}
		} 

		//计算点集的最外面矩形边界
		if(!all_contours.empty()){
			rect = boundingRect(all_contours);
			if(debug) cout<<k<<"个面积"<<rect.area()<<endl;
			rect.x=rect.x-10>0?rect.x-10:0;
			rect.y=rect.y-10>0?rect.y-10:0;
			rect.width=rect.width+10<cols?rect.width+10:cols;
			rect.height=rect.height+10<rows?rect.height+10:rows;
			//rect.x=rect.x>0?rect.x:0;
			//rect.y=rect.y>0?rect.y:0;
			//rect.width=rect.width<cols?rect.width:cols;
			//rect.height=rect.height<rows?rect.height:rows;
			if(rect.area()>150&&rect.area()<10000&&rect.y>40){ //&&rect.y>40
				rectangle(label_image, cvPoint(rect.x, rect.y), cvPoint(rect.x + rect.width, rect.y + rect.height),CV_RGB(255,255, 255), 1, 8, 0);
				track_rect.push_back(rect);
			}
		}
		all_contours.clear();
	}

	return track_rect;//返回待跟踪区域的大小和坐标
}

//-------------------------------------------------------------------------------------------------
// function: get_track_selection_many
// brief:求运动检测结果和轮廓分割图的交集,并进行连通区域分析
// parameter:运动检测结果Mat detection_image，轮廓分割结果Mat segment_image
// return:各个独立的运动检测结果vector<Rect> track_rect，保存了需要跟踪对象的坐标和大小
//-------------------------------------------------------------------------------------------------
vector<Rect> get_track_selection_many(Mat detection_image,Mat segment_image){
	//求两个图的交叉
	vector<Rect> track_rect;
	int sign=255;
	int rows=detection_image.rows;
	int cols=detection_image.cols;
	Mat inter(rows,cols,CV_8UC1);
	if(segment_image.rows != rows || segment_image.cols != cols){
		cout<<"两个矩阵尺寸不相等"<<endl;
		return track_rect;
	}

	//1.求两个图的交集
	for (int i = 0; i < rows; i++){  
		for (int j = 0; j < cols; j++){
			if((int)detection_image.at<uchar>(i,j)==255&&(int)segment_image.at<uchar>(i,j)==255){
				inter.at<uchar>(i,j)=255;
			}else{
				inter.at<uchar>(i,j)=128;
			}
		}
	} 

	//若有需要，就进行形态学运算
	Mat element=getStructuringElement(MORPH_RECT,Size(15,15));
	morphologyEx(inter,inter,MORPH_CLOSE,element);
	if(debug) {
		imshow("交集",inter);
		waitKey(30);
	}

	//2.连通区域分析，返回各个独立的运动检测结果track_rect
	Mat label;
	track_rect= icvprCcaBySeedFill(inter,label);
	return track_rect;
}

//-------------------------------------------------------------------------------------------------
// function: get_track_selection_many_simple
// brief:对单张图像进行连通区域分析
// parameter:待连通图像Mat image
// return:各个独立的运动检测结果vector<Rect> track_rect，保存了需要跟踪对象的坐标和大小
//-------------------------------------------------------------------------------------------------
vector<Rect> get_track_selection_many_simple(Mat image){
	Mat label;
	vector<Rect> track_rect;
	track_rect=icvprCcaBySeedFill(image,label);
	return track_rect;
}

//-------------------------------------------------------------------------------------------------
// function:setMarkers
// brief:WatershedSegmenter类中方法，设置初始化的标记图像
// parameter:初始标记图const Mat& markerImage
// return:无
//-------------------------------------------------------------------------------------------------
void WatershedSegmenter::setMarkers(const Mat& markerImage) {
	markerImage.convertTo(markers,CV_32S);
}

//-------------------------------------------------------------------------------------------------
// function:process
// brief:WatershedSegmenter类中方法，对图片进行分水岭算法分割
// parameter:const Mat &image
// return:分割结果Mat markers
//-------------------------------------------------------------------------------------------------
Mat WatershedSegmenter::process(const Mat &image){
	watershed(image,markers);//调用OpenCV中的watershed实现分水岭算法
	return markers;
}

//-------------------------------------------------------------------------------------------------
// function:getSegmentation
// brief:WatershedSegmenter类中方法，返回分割后的图像
// parameter:无
// return:分割后的图像Mat temp
//-------------------------------------------------------------------------------------------------
Mat WatershedSegmenter::getSegmentation(){
	Mat temp;
	//分割结果中所有高于255的值都会被赋值为255
	markers.convertTo(temp,CV_8U);
	return temp;
}

//-------------------------------------------------------------------------------------------------
// function:getWatersheds
// brief:WatershedSegmenter类中方法，以图像的形式返回分水岭
// parameter:无
// return:分割后的图像Mat temp
//-------------------------------------------------------------------------------------------------
Mat WatershedSegmenter::getWatersheds(){
	Mat temp;
	//在变换前，把每个像素p转换为255p+255（在conertTo中实现）
	markers.convertTo(temp,CV_8U,255,255);
	return temp;
}