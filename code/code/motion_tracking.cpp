#include"motion_tracking.h"

extern bool debug;

//-------------------------------------------------------------------------------------------------
// function: motion_tracking
// brief: 运动追踪
// parameter:输入图像Mat image，被跟踪区域Rect track_window
// return: 更新被跟踪区域Rect track_window，显示追踪结果
//-------------------------------------------------------------------------------------------------
RotatedRect motion_tracking(Rect &track_window,Mat image){

	Mat frame, hsv;//image的HSV空间图像
	Mat hue;//hsv的h分量
	Mat mask;//原图二值化结果
	Mat hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
	int hsize = 16;
	float hranges[] = {0,180};
	const float* phranges = hranges;
	int _vmin = vmin, _vmax = vmax;

	if(debug)cout<<"motion_tracking track_window"<<track_window<<endl;
	if(track_window.width>0&&track_window.height>0){
		cvtColor(image, hsv, COLOR_BGR2HSV);//颜色空间转换函数,将RGB颜色转向HSV
		//inRange函数的功能是检查输入数组每个元素大小是否在2个给定数值之间，可以有多通道,mask保存0通道的最小值，也就是h分量
		//二值化结果保存在mask中
		inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
			Scalar(180, 256, MAX(_vmin, _vmax)), mask);

		int ch[] = {0, 0};
		hue.create(hsv.size(), hsv.depth());
		mixChannels(&hsv, 1, &hue, 1, ch, 1);//将hsv第一个通道(也就是色调)的数复制到hue中

		Mat roi(hue, track_window);//hue中用来计算直方图的区域,track_window为其感兴趣的区域 
		Mat maskroi(mask, track_window);//roi对应的掩码，即roi对应maskroi值为1的点将用来计算直方图
		calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);//将roi的0通道计算直方图放入hist中（以maskroi为掩码），hsize为每一维直方图的大小  
		normalize(hist, hist, 0, 255, CV_MINMAX);//将hist矩阵进行数组范围归一化，都归一化到0~255

		histimg = Scalar::all(0);//与按下'c'键是一样的，这里的all(0)表示的是标量全部清0
		int binW = histimg.cols / hsize;//histing是一个200*300的矩阵，hsize应该是每一个bin的宽度，也就是histing矩阵能分出几个bin出来
		Mat buf(1, hsize, CV_8UC3);//定义一个缓冲单bin矩阵
		for( int i = 0; i < hsize; i++ )
			buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);//Vec3b为3个char值的向量
		cvtColor(buf, buf, CV_HSV2BGR);//将hsv又转换成bgr

		for( int i = 0; i < hsize; i++ )
		{
			int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);//at函数为返回一个指定数组元素的参考值
			rectangle( histimg, Point(i*binW,histimg.rows),
				Point((i+1)*binW,histimg.rows - val),
				Scalar(buf.at<Vec3b>(i)), -1, 8 );//在一幅输入图像上画一个简单抽的矩形，指定左上角和右下角，并定义颜色，大小，线型等 
		}

		//计算直方图的反向投影，计算hue图像0通道直方图hist的反向投影，并让入backproj中
		calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
		backproj &= mask;
		//使用CamShift进行跟踪，更新track_window,TermCriteria这个类是作为迭代算法的终止条件,
		RotatedRect trackBox = CamShift(backproj, track_window,
			TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 5, 1 ));
		if( track_window.area() <= 1 )
		{
			int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
			track_window = Rect(track_window.x - r, track_window.y - r,
				track_window.x + r, track_window.y + r) &
				Rect(0, 0, cols, rows);//Rect函数为矩阵的偏移和大小，即第一二个参数为矩阵的左上角点坐标，第三四个参数为矩阵的宽和高
		}

		//cvtColor( backproj, image, COLOR_GRAY2BGR );
		if(debug) imshow( "Histogram", histimg );

		return trackBox;
	}
}

void run_thread(trackThread& track_thread){
	track_thread.thread_test();
}