#include"motion_segment.h"
extern bool debug;

//����ָ�
Mat motion_segment(Mat image){
	Mat result;
	int rows=image.rows;
	int cols=image.cols;

	if (!image.data)
		return result;

	// Identify image pixels with object
	Mat binary;
	cvtColor(image,binary,COLOR_BGRA2GRAY);
	threshold(binary,binary,100,255,THRESH_BINARY_INV);//��ֵ�ָ�ԭͼ�ĻҶ�ͼ����ö�ֵͼ��
	//imshow("binary image",binary);
	//waitKey();

	//�����˶�Աɾ��������Ĺ��ڲ���
	for (int i = 0; i < rows; i++){  
		for (int j = 0; j < 90; j++){
			binary.at<uchar>(i,j)=255;
		}
	} 

	// CLOSE operation
	Mat element5(8,8,CV_8U,Scalar(1));//5*5�����Σ�8λuchar�ͣ�ȫ1�ṹԪ��
	Mat fg1;
	//morphologyEx(binary, fg1,MORPH_CLOSE,element5,Point(-1,-1),1);// ���������������ϸС�ն��������ڽ�����
	morphologyEx(binary, fg1,MORPH_CLOSE,element5);

	// Display the foreground image
	//namedWindow("Foreground Image");
	//imshow("Foreground Image",fg1);
	//waitKey();

	// Identify image pixels without objects

	Mat bg1;
	//dilate(binary,bg1,Mat(),Point(-1,-1),4);//����4�Σ�ê��Ϊ�ṹԪ�����ĵ�
	dilate(binary,bg1,Mat());
	threshold(bg1,bg1,1,128,THRESH_BINARY_INV);//>=1����������Ϊ128����������
	// Display the background image
	//namedWindow("Background Image");
	//imshow("Background Image",bg1);
	//waitKey();

	//Get markers image

	Mat markers1 = fg1 + bg1; //ʹ��Mat������������+���ϲ�ͼ��
	//namedWindow("markers Image");
	//imshow("markers Image",markers1);
	//waitKey();

	// Apply watershed segmentation

	WatershedSegmenter segmenter1;  //ʵ����һ����ˮ��ָ���Ķ���
	segmenter1.setMarkers(markers1);//�����㷨�ı��ͼ��ʹ��ˮ�͹��̴�����Ԥ�ȶ���õı�����ؿ�ʼ
	segmenter1.process(image);     //������ָ�ԭͼ

	// Display segmentation result
	//���޸ĺ�ı��ͼmarkersת��Ϊ����ʾ��8λ�Ҷ�ͼ�����طָ�������ɫΪǰ������ɫΪ������0Ϊ��Ե��
	result=segmenter1.getSegmentation();
	Mat element=getStructuringElement(MORPH_RECT,Size(15,15));
	morphologyEx(result,result,MORPH_CLOSE,element);
	if(debug) imshow("�����ָ���",result);
	waitKey(30);

	return result;
}

vector<Rect> icvprCcaBySeedFill(Mat& _binImg,Mat& _lableImg)  
{  
	vector<Rect> track_rect;
	if (_binImg.empty() ||  
		_binImg.type() != CV_8UC1)  
	{  
		return track_rect;  
	}  

	_binImg.convertTo(_lableImg, CV_32SC1) ;  

	int label = 1 ;  // start by 2  
	int sign=255;

	int rows = _binImg.rows - 1 ;  
	int cols = _binImg.cols - 1 ;  
	for (int i = 1; i < rows-1; i++)  {  
		int* data= _lableImg.ptr<int>(i) ;  
		for (int j = 1; j < cols-1; j++)  {  
			if (data[j] == sign)  {  
				std::stack<std::pair<int,int>> neighborPixels ;     
				neighborPixels.push(std::pair<int,int>(i,j)) ;     // pixel position: <i,j>  
				++label;  // begin with a new label  
				while (!neighborPixels.empty())  {  
					// get the top pixel on the stack and label it with the same label  
					std::pair<int,int> curPixel = neighborPixels.top() ;  
					int curX = curPixel.first ;  
					int curY = curPixel.second ;  
					_lableImg.at<int>(curX, curY) = label ;  

					// pop the top pixel  
					neighborPixels.pop() ;  

					if(curX==0||curY==0||curX==rows||curY==cols) continue;

					// push the 4-neighbors (foreground pixels)  
					if (_lableImg.at<int>(curX, curY-1) == sign){// left pixel  
						neighborPixels.push(std::pair<int,int>(curX, curY-1)) ;  
					}  
					if (_lableImg.at<int>(curX, curY+1) == sign){// right pixel  
						neighborPixels.push(std::pair<int,int>(curX, curY+1)) ;  
					}  
					if (_lableImg.at<int>(curX-1, curY) == sign){// up pixel  
						neighborPixels.push(std::pair<int,int>(curX-1, curY)) ;  
					}  
					if (_lableImg.at<int>(curX+1, curY) == sign){// down pixel  
						neighborPixels.push(std::pair<int,int>(curX+1, curY)) ;  
					}  
				}         
			}  
		}  
	} 
	_lableImg.convertTo(_lableImg, CV_8UC1);
	//imshow("_lableImg",_lableImg);
	//waitKey(30);
	if(debug) cout<<"��label"<<label<<endl;

	vector<Point> all_contours;
	Rect rect;//׷������
	for(int k=1;k<=label;k++){
		for (int i = 0; i < rows; i++){  
			for (int j = 0; j < cols; j++){
				if(_lableImg.at<uchar>(i,j)==k){
					Point x;
					x.x=j;
					x.y=i;
					all_contours.push_back(x);
				}
			}
		} 

		if(!all_contours.empty()){
			rect = boundingRect(all_contours); 
			if(debug) cout<<k<<"�����"<<rect.area()<<endl;
			rect.x=rect.x-10>0?rect.x-10:0;
			rect.y=rect.y-10>0?rect.y-10:0;
			rect.width=rect.width+10<cols?rect.width+10:cols;
			rect.height=rect.height+10<rows?rect.height+10:rows;
			if(rect.area()>300&&rect.area()<10000&&rect.y>40){ 
				rectangle(_lableImg, cvPoint(rect.x, rect.y), cvPoint(rect.x + rect.width, rect.y + rect.height),CV_RGB(255,255, 255), 1, 8, 0);
				track_rect.push_back(rect);
			}
		}
		//���all_contours
		all_contours.clear();
	}
	//imshow("_lableImg2",_lableImg);
	//waitKey(30);

	return track_rect;
}

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
	for (int i = 0; i < rows; i++){  
		for (int j = 0; j < cols; j++){
			//cout<<(int)src1.at<uchar>(i,j)<<" "<<(int)src2.at<uchar>(i,j)<<endl;
			if((int)detection_image.at<uchar>(i,j)==255&&(int)segment_image.at<uchar>(i,j)==255){
				inter.at<uchar>(i,j)=255;
				//cout<<(int)detection_image.at<uchar>(i,j)<<" "<<(int)segment_image.at<uchar>(i,j)<<endl;
			}else{
				inter.at<uchar>(i,j)=128;
			}
		}
	} 
	//Mat element=getStructuringElement(MORPH_RECT,Size(20,20));
	//morphologyEx(inter,inter,MORPH_CLOSE,element);
	if(debug) {
		imshow("inter",inter);
	waitKey(30);
	}

	//������
	Mat label;
	track_rect=icvprCcaBySeedFill(inter,label);
	return track_rect;

}

vector<Rect> get_track_selection_many_by_detection(Mat detection_image){
	//������
	Mat label;
	vector<Rect> track_rect;
	track_rect=icvprCcaBySeedFill(detection_image,label);
	return track_rect;

}