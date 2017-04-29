#include"motion_tracking.h"

extern bool debug;

//-------------------------------------------------------------------------------------------------
// function: motion_tracking
// brief: �˶�׷��
// parameter:����ͼ��Mat image������������Rect track_window
// return: ���±���������Rect track_window����ʾ׷�ٽ��
//-------------------------------------------------------------------------------------------------
RotatedRect motion_tracking(Rect &track_window,Mat image){

	Mat frame, hsv;//image��HSV�ռ�ͼ��
	Mat hue;//hsv��h����
	Mat mask;//ԭͼ��ֵ�����
	Mat hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
	int hsize = 16;
	float hranges[] = {0,180};
	const float* phranges = hranges;
	int _vmin = vmin, _vmax = vmax;

	if(debug)cout<<"motion_tracking track_window"<<track_window<<endl;
	if(track_window.width>0&&track_window.height>0){
		cvtColor(image, hsv, COLOR_BGR2HSV);//��ɫ�ռ�ת������,��RGB��ɫת��HSV
		//inRange�����Ĺ����Ǽ����������ÿ��Ԫ�ش�С�Ƿ���2��������ֵ֮�䣬�����ж�ͨ��,mask����0ͨ������Сֵ��Ҳ����h����
		//��ֵ�����������mask��
		inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),
			Scalar(180, 256, MAX(_vmin, _vmax)), mask);

		int ch[] = {0, 0};
		hue.create(hsv.size(), hsv.depth());
		mixChannels(&hsv, 1, &hue, 1, ch, 1);//��hsv��һ��ͨ��(Ҳ����ɫ��)�������Ƶ�hue��

		Mat roi(hue, track_window);//hue����������ֱ��ͼ������,track_windowΪ�����Ȥ������ 
		Mat maskroi(mask, track_window);//roi��Ӧ�����룬��roi��ӦmaskroiֵΪ1�ĵ㽫��������ֱ��ͼ
		calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);//��roi��0ͨ������ֱ��ͼ����hist�У���maskroiΪ���룩��hsizeΪÿһάֱ��ͼ�Ĵ�С  
		normalize(hist, hist, 0, 255, CV_MINMAX);//��hist����������鷶Χ��һ��������һ����0~255

		histimg = Scalar::all(0);//�밴��'c'����һ���ģ������all(0)��ʾ���Ǳ���ȫ����0
		int binW = histimg.cols / hsize;//histing��һ��200*300�ľ���hsizeӦ����ÿһ��bin�Ŀ�ȣ�Ҳ����histing�����ֳܷ�����bin����
		Mat buf(1, hsize, CV_8UC3);//����һ�����嵥bin����
		for( int i = 0; i < hsize; i++ )
			buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);//Vec3bΪ3��charֵ������
		cvtColor(buf, buf, CV_HSV2BGR);//��hsv��ת����bgr

		for( int i = 0; i < hsize; i++ )
		{
			int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);//at����Ϊ����һ��ָ������Ԫ�صĲο�ֵ
			rectangle( histimg, Point(i*binW,histimg.rows),
				Point((i+1)*binW,histimg.rows - val),
				Scalar(buf.at<Vec3b>(i)), -1, 8 );//��һ������ͼ���ϻ�һ���򵥳�ľ��Σ�ָ�����ϽǺ����½ǣ���������ɫ����С�����͵� 
		}

		//����ֱ��ͼ�ķ���ͶӰ������hueͼ��0ͨ��ֱ��ͼhist�ķ���ͶӰ��������backproj��
		calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
		backproj &= mask;
		//ʹ��CamShift���и��٣�����track_window,TermCriteria���������Ϊ�����㷨����ֹ����,
		RotatedRect trackBox = CamShift(backproj, track_window,
			TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 5, 1 ));
		if( track_window.area() <= 1 )
		{
			int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
			track_window = Rect(track_window.x - r, track_window.y - r,
				track_window.x + r, track_window.y + r) &
				Rect(0, 0, cols, rows);//Rect����Ϊ�����ƫ�ƺʹ�С������һ��������Ϊ��������Ͻǵ����꣬�����ĸ�����Ϊ����Ŀ�͸�
		}

		//cvtColor( backproj, image, COLOR_GRAY2BGR );
		if(debug) imshow( "Histogram", histimg );

		return trackBox;
	}
}

void run_thread(trackThread& track_thread){
	track_thread.thread_test();
}