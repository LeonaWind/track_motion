#include"motion_segment.h"
extern bool debug;

//-------------------------------------------------------------------------------------------------
// function: motion_segment
// brief:���÷�ˮ���㷨����ͼ���и����������
// parameter:���ָ�ͼ��Mat image
// return:ͼ��ָ���Mat output,ǰ������ֵΪ255����������ֵΪ128
//-------------------------------------------------------------------------------------------------
Mat motion_segment(Mat image){
	Mat result;
	int rows=image.rows;
	int cols=image.cols;

	if (!image.data)
		return result;

	//1.��ֵ�ָ�ԭͼ�ĻҶ�ͼ����ö�ֵͼ��binary
	Mat binary;
	cvtColor(image,binary,COLOR_BGRA2GRAY);
	threshold(binary,binary,100,255,THRESH_BINARY_INV);

	//�����˶�Աɾ��������Ĺ��ڲ���
	for (int i = 0; i < rows; i++){  
		for (int j = 0; j < 100; j++){
			binary.at<uchar>(i,j)=255;
		}
	} 

	//2.ǰ��ͼ����б�����
	Mat fg1;
	Mat element5(8,8,CV_8U,Scalar(1));//5*5�����Σ�8λuchar�ͣ�ȫ1�ṹԪ��
	morphologyEx(binary, fg1,MORPH_CLOSE,element5);

	//3.����ͼ�������������
	Mat bg1;
	dilate(binary,bg1,Mat());
	threshold(bg1,bg1,1,128,THRESH_BINARY_INV);//>=1����������Ϊ128����������

	//4.��̬ѧ������ǰ��ͼfg1�ͱ���ͼbg1��������
	Mat markers1 = fg1 + bg1; //ʹ��Mat������������+���ϲ�ͼ��

	//5.ʹ�÷�ˮ���㷨���зָ�
	WatershedSegmenter segmenter1;//ʵ����һ����ˮ��ָ���Ķ���
	segmenter1.setMarkers(markers1);//�����㷨�ı��ͼ��ʹ��ˮ�͹��̴�����Ԥ�ȶ���õı�����ؿ�ʼ
	segmenter1.process(image);//������ָ�ԭͼ

	//6.�ָ�����������
	//���޸ĺ�ı��ͼmarkersת��Ϊ����ʾ��8λ�Ҷ�ͼ�����طָ�������ɫΪǰ������ɫΪ������0Ϊ��Ե��
	result=segmenter1.getSegmentation();
	Mat element=getStructuringElement(MORPH_RECT,Size(8,8));
	morphologyEx(result,result,MORPH_CLOSE,element);
	if(debug){
		imshow("�����ָ���",result);
		waitKey(30);
	}

	return result;//���طָ���ĸ������壬ǰ��Ϊ255������Ϊ128
}

//-------------------------------------------------------------------------------------------------
// function: icvprCcaBySeedFill
// brief:����������䷨������ͨ�������
// parameter:����ͨͼ��Mat con_image�����ͼMat label_image
// return:�����������˶������vector<Rect> track_rect����������Ҫ���ٶ��������ʹ�С
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

	//1.��ͨ���������������λ�õĵ���Ϊͬһ��ֵ������label_image��
	con_image.convertTo(label_image, CV_32SC1) ;  
	for (int i = 1; i < rows-1; i++)  {  
		int* data= label_image.ptr<int>(i) ;  
		for (int j = 1; j < cols-1; j++)  {  
			//�����ҵ�һ��ǰ�����ص�
			if (data[j] == sign)  {  
				std::stack<std::pair<int,int>> neighborPixels ;     
				neighborPixels.push(std::pair<int,int>(i,j)) ;
				++label;
				while (!neighborPixels.empty())  {  
					std::pair<int,int> curPixel = neighborPixels.top() ;  
					int cur_pixel_x = curPixel.first ;  
					int cur_pixel_y = curPixel.second ;  
					label_image.at<int>(cur_pixel_x, cur_pixel_y) = label ;  
					neighborPixels.pop() ;  

					if(cur_pixel_x==0||cur_pixel_y==0||cur_pixel_x==rows||cur_pixel_y==cols) continue;

					//�����ڵ�ǰ�����ص���ջ
					if (label_image.at<int>(cur_pixel_x, cur_pixel_y-1) == sign){//�����ص� 
						neighborPixels.push(std::pair<int,int>(cur_pixel_x, cur_pixel_y-1)) ;  
					}  
					if (label_image.at<int>(cur_pixel_x, cur_pixel_y+1) == sign){//�����ص� 
						neighborPixels.push(std::pair<int,int>(cur_pixel_x, cur_pixel_y+1)) ;  
					}  
					if (label_image.at<int>(cur_pixel_x-1, cur_pixel_y) == sign){//�����ص� 
						neighborPixels.push(std::pair<int,int>(cur_pixel_x-1, cur_pixel_y)) ;  
					}  
					if (label_image.at<int>(cur_pixel_x+1, cur_pixel_y) == sign){//�����ص� 
						neighborPixels.push(std::pair<int,int>(cur_pixel_x+1, cur_pixel_y)) ;  
					}  
				}         
			}  
		}  
	} 
	label_image.convertTo(label_image, CV_8UC1);

	//2.��ȡ׷�����򣬸���label_image��ֵ����Rect����
	vector<Point> all_contours;
	Rect rect;//׷������
	for(int k=1;k<=label;k++){
		//��ͬһ��labelֵ�����ص����ͬһ���㼯��all_contours
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

		//����㼯����������α߽�
		if(!all_contours.empty()){
			rect = boundingRect(all_contours);
			if(debug) cout<<k<<"�����"<<rect.area()<<endl;
			rect.x=rect.x-10>0?rect.x-10:0;
			rect.y=rect.y-10>0?rect.y-10:0;
			rect.width=rect.width+10<cols?rect.width+10:cols;
			rect.height=rect.height+10<rows?rect.height+10:rows;
			if(rect.area()>150&&rect.area()<10000&&rect.y>40){ 
				rectangle(label_image, cvPoint(rect.x, rect.y), cvPoint(rect.x + rect.width, rect.y + rect.height),CV_RGB(255,255, 255), 1, 8, 0);
				track_rect.push_back(rect);
			}
		}
		all_contours.clear();
	}

	return track_rect;//���ش���������Ĵ�С������
}

//-------------------------------------------------------------------------------------------------
// function: get_track_selection_many
// brief:���˶�������������ָ�ͼ�Ľ���,��������ͨ�������
// parameter:�˶������Mat detection_image�������ָ���Mat segment_image
// return:�����������˶������vector<Rect> track_rect����������Ҫ���ٶ��������ʹ�С
//-------------------------------------------------------------------------------------------------
vector<Rect> get_track_selection_many(Mat detection_image,Mat segment_image){
	//������ͼ�Ľ���
	vector<Rect> track_rect;
	int sign=255;
	int rows=detection_image.rows;
	int cols=detection_image.cols;
	Mat inter(rows,cols,CV_8UC1);
	if(segment_image.rows != rows || segment_image.cols != cols){
		cout<<"��������ߴ粻���"<<endl;
		return track_rect;
	}

	//1.������ͼ�Ľ���
	for (int i = 0; i < rows; i++){  
		for (int j = 0; j < cols; j++){
			if((int)detection_image.at<uchar>(i,j)==255&&(int)segment_image.at<uchar>(i,j)==255){
				inter.at<uchar>(i,j)=255;
			}else{
				inter.at<uchar>(i,j)=128;
			}
		}
	} 
	
	//������Ҫ���ͽ�����̬ѧ����
	Mat element=getStructuringElement(MORPH_RECT,Size(15,15));
	morphologyEx(inter,inter,MORPH_CLOSE,element);
	if(debug) {
		imshow("inter",inter);
		waitKey(30);
	}

	//2.��ͨ������������ظ����������˶������track_rect
	Mat label;
	track_rect= icvprCcaBySeedFill(inter,label);
	return track_rect;
}

//-------------------------------------------------------------------------------------------------
// function: get_track_selection_many_simple
// brief:�Ե���ͼ�������ͨ�������
// parameter:����ͨͼ��Mat image
// return:�����������˶������vector<Rect> track_rect����������Ҫ���ٶ��������ʹ�С
//-------------------------------------------------------------------------------------------------
vector<Rect> get_track_selection_many_simple(Mat image){
	Mat label;
	vector<Rect> track_rect;
	track_rect=icvprCcaBySeedFill(image,label);
	return track_rect;
}