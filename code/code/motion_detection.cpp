#include"motion_detection.h"

extern bool debug;

//-------------------------------------------------------------------------------------------------
// function: frame3_diff_motion_detection
// brief: �Ľ���֡�����˶�����
// parameter:��k-1֡ͼ��Mat image_gray_pre����k֡ͼ��Mat image_gray����k+1֡ͼ��Mat image_gray_next
// return: ����������Rect selection
//-------------------------------------------------------------------------------------------------
Mat frame3_diff_motion_detection(Mat image_gray_pre,Mat image_gray,Mat image_gray_next,Mat &background_gray_cv32f){
	int rows=image_gray.rows;
	int cols=image_gray.cols;
	Mat output(rows,cols,CV_8UC1);//�������˶�ͼ��
	int thres=10;

	//1.����ͼ�������
	Mat diff_gray(rows,cols,CV_8UC1);
	Mat diff_gray1, diff_gray2, diff_gray3;
	absdiff(image_gray_pre, image_gray, diff_gray1);
	absdiff(image_gray, image_gray_next, diff_gray2);
	absdiff(image_gray_next, image_gray_pre, diff_gray3);

	//2.���ֵ
	for(int i=0;i<rows;i++)  {  
		for(int j=0;j<cols;j++)  {    
			diff_gray.at<uchar>(i,j) = (diff_gray1.at<uchar>(i,j)+diff_gray2.at<uchar>(i,j)+diff_gray3.at<uchar>(i,j))/3; 
			if(diff_gray.at<uchar>(i,j)>thres){//��ֵ��
				diff_gray.at<uchar>(i,j) = 255;
			}else{
				diff_gray.at<uchar>(i,j) = 0;
			}
			//cout<<(int)diff_gray.at<uchar>(i,j)<<" ";
		}    
	} 

	//3.��̬ѧ����
	Mat element=getStructuringElement(MORPH_RECT,Size(15,15));
	morphologyEx(diff_gray,diff_gray,MORPH_CLOSE,element);
	//if(debug) imshow("��̬ѧ������",diff_gray);
	//imshow("��̬ѧ������",diff_gray);

	//4.��k֡������ģ
	/*Mat background_diff_gray;//��ǰͼ�뱳��ͼ�Ĳ���
	Mat background_gray_cv8u;//CV_8U��ʽ����ͼ 
	double learningRate=0.9;//ѧϰ��
	background_gray_cv32f.convertTo (background_gray_cv8u,CV_8U);

	absdiff(image_gray, background_gray_cv8u, background_diff_gray);//��ǰ֡������ͼ���  
	threshold(background_diff_gray, background_diff_gray, 40, 255.0, CV_THRESH_BINARY);//��ֵ��ǰ��ͼ
	accumulateWeighted (image_gray,background_gray_cv32f,learningRate,background_diff_gray);//���±�����output��Ϊ����

	//imshow("background_diff_gray",background_diff_gray);
	*/

	//5.��k֡canny��Ե���
	Mat image_gray_canny;
	Canny(image_gray, image_gray_canny, 3, 9, 3);
	//if(debug) imshow("image_gray_canny",image_gray_canny);

	//6.��k֡��Ե�������֡��������������
	mat_and(diff_gray,image_gray_canny,diff_gray);

	//7.���������뱳����ģ������л�����
	//mat_or(diff_gray,background_diff_gray,output);

	//8.��̬ѧ����
	morphologyEx(diff_gray,output,MORPH_CLOSE,element);
	if(debug) imshow("��������",output);

	//imshow("image_gray_canny",image_gray_canny);
	//imshow("diff_gray",diff_gray);
	//Rect selection = get_track_selection_all(output); //��������ȡ׷������
	//return selection;

	return output;


}

//-------------------------------------------------------------------------------------------------
// function: background_motion_detection
// brief: ������������˶�����
// parameter:��ǰͼ��Mat image����ǰͼ��Ҷ�ͼMat image_gray��CV_32F����ͼ��Ҷ�ͼMat background_gray
// return: ����������Rect selection
//-------------------------------------------------------------------------------------------------
Rect background_motion_detection(Mat &image_gray,Mat &background_gray_cv32f){
	Mat diff_gray;//��ǰͼ�뱳��ͼ�Ĳ���
	Mat output;//�������˶�ͼ��
	double learningRate=0.9;//ѧϰ��
	Mat background_gray_cv8u;//CV_8U��ʽ����ͼ 
	background_gray_cv32f.convertTo (background_gray_cv8u,CV_8U);

	absdiff(image_gray, background_gray_cv8u, diff_gray);//��ǰ֡������ͼ���  
	threshold(diff_gray, output, 30, 255.0, CV_THRESH_BINARY);//��ֵ��ǰ��ͼ
	accumulateWeighted (image_gray,background_gray_cv32f,learningRate,output);//���±�����output��Ϊ����

	//imshow("foreground",output);
	//imshow("image_gray",image_gray);
	//imshow("background", background_gray_cv8u); 

	Rect selection = get_track_selection(output); 
	return selection;

}

//-------------------------------------------------------------------------------------------------
// function: get_track_selection_all
// brief: ��ȡ׷������,���������������˶��ĵ����һ���������
// parameter:����ͼ��Mat image
// return: ����������Rect selection
//-------------------------------------------------------------------------------------------------
Rect get_track_selection_all(Mat &image){

	int rows=image.rows;
	int cols=image.cols;
	vector<Point> all_contours;
	Rect rect;//׷������

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

	if(debug) imshow("�Ľ���֡�",image);
	return rect;
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
	int count_max=1000;//��ౣ���
	int point_count_max=0,contours_count_max=0;//��¼Ҫ��ȡ�ĵ�
	int point_count=0,contours_count=0;
	CvRect rect;//׷������

	if (!image.empty())     
	{           
		findContours(image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // ��������                
		//Scalar color(rand()&255,rand()&255,rand()&255);
		//drawContours(dest_image, contours,-1,color,3);// ����������� 
		if(!contours.empty()){
			//���෽���ҵ���������
			//��ȡ������Ĵ�С��������point_count��
			for (vector<vector<Point>>::const_iterator iter = contours.begin(); iter != contours.end(); iter++)
			{
				point_count+=(*iter).size();
			}

			Scalar colorTab[] =     //��Ϊ���ֻ��5�࣬�������Ҳ�͸�5����ɫ  
			{  
				Scalar(255, 255, 255),  
				Scalar(0, 255, 0),  
				Scalar(255, 100, 100),  
				Scalar(255, 0, 255),  
				Scalar(0, 255, 255)  
			};  
			int clusterCount=1;//�����С
			int* clusterIdxArr =  new int[clusterCount];
			double* clusterIdxDis =  new double[clusterCount];//����ƽ������
			clusterCount = MIN(clusterCount, point_count); 
			Mat points(point_count, 1, CV_32FC2,Scalar::all(0));//pointsΪ����������д洢���ǲ�����
			Mat labels;//labels�зŵ���ִ��kmeans�㷨��points�в�����Ĵص�����
			Mat centers(clusterCount, 1, points.type());//�����洢���������ĵ�
			float* dataDst=points.ptr<float>(0);;//dataDstָ��points����

			//��ʼ��
			for(int i = 0;i < clusterCount;i++)
			{ 
				clusterIdxArr[i]=0;
				clusterIdxDis[i]=0.0;
			} 

			//����������������points��
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

			//����3�Σ�ȡ�����õ��ǴΣ�����ĳ�ʼ������PP�ض�������㷨��
			kmeans(points, clusterCount, labels, TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0),  3, KMEANS_PP_CENTERS, centers);

			//ȥ������ĵ�

			//ѡȡ��
			vector<Point> all_contours;
			for(i = 0;i < clusterCount;i++)//������ĵ�
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

			//�ҳ����ľ���,�����ĵ㱣����all_contours��
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
				//�����ֵ
				Point first_point=all_contours[0];
				double dis_avge=sqrt(pow((max_center_point.x-first_point.x),2)+pow((max_center_point.y-first_point.y),2));
				for (vector<Point>::const_iterator iter = all_contours.begin(); iter != all_contours.end(); iter++){
					double dis_temp=sqrt(pow((max_center_point.x-(*iter).x),2)+pow((max_center_point.y-(*iter).y),2));
					dis_avge=(dis_avge+dis_temp)/2;
					//cout<<"dis_avge"<<dis_avge<<endl;
				}

				//ȥ������ĵ�
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

			//���ָ�������
			if(!all_contours.empty()){
				rect = boundingRect(all_contours); 
				rectangle(dest_image, cvPoint(rect.x, rect.y), cvPoint(rect.x + rect.width, rect.y + rect.height),CV_RGB(255,255, 255), 1, 8, 0);
			}

		}
	}  
	if(debug) imshow("������ģ��",dest_image);

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
// brief: �����������������
// parameter:����ͼ��Mat src1,Mat src2����������Mat &dst
// return: �գ���������������dst��
//-------------------------------------------------------------------------------------------------
void mat_and(Mat src1,Mat src2,Mat &dst){
	int rows=src1.rows;
	int cols=src1.cols;
	if(src2.rows != rows || src2.cols != cols){
		cout<<"��������ߴ粻���"<<endl;
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
// brief: ����������л�����
// parameter:����ͼ��Mat src1,Mat src2����������Mat &dst
// return: �գ���������������dst��
//-------------------------------------------------------------------------------------------------
void mat_or(Mat src1,Mat src2,Mat &dst){
	int rows=src1.rows;
	int cols=src1.cols;
	if(src2.rows != rows || src2.cols != cols){
		cout<<"��������ߴ粻���"<<endl;
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
