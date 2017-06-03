#include"motion_tracking.h"

//-------------------------------------------------------------------------------------------------
// function: motion_tracking
// brief: 运动追踪
// parameter:输入图像Mat image，被跟踪区域Rect track_window
// return: 更新被跟踪区域Rect track_window，显示追踪结果
//-------------------------------------------------------------------------------------------------

void run_thread(trackThread& track_thread){
	track_thread.thread_test();
}

float get_distance(Point result_point,Point old_point){
	float x=(result_point.x-old_point.x)*(result_point.x-old_point.x);
	float y=(result_point.y-old_point.y)*(result_point.y-old_point.y);
	float result=sqrt(x+y);
	return result;
}