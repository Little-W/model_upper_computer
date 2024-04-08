#include <iostream>
#include <string>
#include <csignal>
#include <unistd.h>
#include"process.h"
#include"pid.h"
#include"timer.h"
#include"reader.h"
#include"serial/serial.h"
#include"shared_mem.h"
#include <chrono>
#include <ctime>
#include "comm.h"

using namespace std;
using namespace cv;
int stop = false;
MainImage MI;
AngleControl AC(Re.main.kp, Re.main.kd, Re.main.ki, Re.main.max_ag, Re.main.min_ag);
SpeedControl SC(Re.main.min_v_diff, Re.main.max_v_diff, Re.main.max_v, Re.main.min_v,
				Re.main.sc_kp,Re.main.sc_ki,Re.main.sc_kd,
				Re.main.dy_speed_bezier_p0_ctrl_x,Re.main.dy_speed_bezier_p0_ctrl_y,
				Re.main.dy_speed_bezier_p1_ctrl_x,Re.main.dy_speed_bezier_p1_ctrl_y,
				Re.main.slowdown_enhance_bezier_p0_ctrl_x,Re.main.slowdown_enhance_bezier_p0_ctrl_y,
				Re.main.slowdown_enhance_bezier_p1_ctrl_x,Re.main.slowdown_enhance_bezier_p1_ctrl_y,
				Re.main.slowdown_smooth_bezier_p0_ctrl_x,Re.main.slowdown_smooth_bezier_p0_ctrl_y,
				Re.main.slowdown_enhance_bezier_p1_ctrl_x,Re.main.slowdown_enhance_bezier_p1_ctrl_y				
				);
SpeedControl SC_turn (Re.turn.min_v_diff, Re.turn.max_v_diff, Re.main.max_v, Re.turn.speed_ground,
						Re.turn.sc_kp,Re.turn.sc_ki,Re.turn.sc_kd,
						Re.turn.dy_speed_bezier_p0_ctrl_x,Re.turn.dy_speed_bezier_p0_ctrl_y,
						Re.turn.dy_speed_bezier_p1_ctrl_x,Re.turn.dy_speed_bezier_p1_ctrl_y,
						Re.turn.slowdown_enhance_bezier_p0_ctrl_x,Re.turn.slowdown_enhance_bezier_p0_ctrl_y,
						Re.turn.slowdown_enhance_bezier_p1_ctrl_x,Re.turn.slowdown_enhance_bezier_p1_ctrl_y,
						Re.turn.slowdown_smooth_bezier_p0_ctrl_x,Re.turn.slowdown_smooth_bezier_p0_ctrl_y,
						Re.turn.slowdown_enhance_bezier_p1_ctrl_x,Re.turn.slowdown_enhance_bezier_p1_ctrl_y									
						);

SpeedControl SC_circel_r(Re.main.min_v_diff, Re.main.max_v_diff, Re.main.max_v, Re.main.min_v,
				Re.main.sc_kp,Re.main.sc_ki,Re.main.sc_kd,
				Re.main.dy_speed_bezier_p0_ctrl_x,Re.main.dy_speed_bezier_p0_ctrl_y,
				Re.main.dy_speed_bezier_p1_ctrl_x,Re.main.dy_speed_bezier_p1_ctrl_y,
				Re.r_circle.slowdown_enhance_bezier_p0_ctrl_x,Re.r_circle.slowdown_enhance_bezier_p0_ctrl_y,
				Re.r_circle.slowdown_enhance_bezier_p1_ctrl_x,Re.r_circle.slowdown_enhance_bezier_p1_ctrl_y,
				Re.main.slowdown_smooth_bezier_p0_ctrl_x,Re.main.slowdown_smooth_bezier_p0_ctrl_y,
				Re.main.slowdown_enhance_bezier_p1_ctrl_x,Re.main.slowdown_enhance_bezier_p1_ctrl_y				
				);

SpeedControl SC_circel_l(Re.main.min_v_diff, Re.main.max_v_diff, Re.main.max_v, Re.main.min_v,
				Re.main.sc_kp,Re.main.sc_ki,Re.main.sc_kd,
				Re.main.dy_speed_bezier_p0_ctrl_x,Re.main.dy_speed_bezier_p0_ctrl_y,
				Re.main.dy_speed_bezier_p1_ctrl_x,Re.main.dy_speed_bezier_p1_ctrl_y,
				Re.l_circle.slowdown_enhance_bezier_p0_ctrl_x,Re.l_circle.slowdown_enhance_bezier_p0_ctrl_y,
				Re.l_circle.slowdown_enhance_bezier_p1_ctrl_x,Re.l_circle.slowdown_enhance_bezier_p1_ctrl_y,
				Re.main.slowdown_smooth_bezier_p0_ctrl_x,Re.main.slowdown_smooth_bezier_p0_ctrl_y,
				Re.main.slowdown_enhance_bezier_p1_ctrl_x,Re.main.slowdown_enhance_bezier_p1_ctrl_y				
				);

std::chrono::time_point<std::chrono::high_resolution_clock> start_time_stamp;
bool disable_motor = false;
bool direct_motor_power_ctrl = false;
bool turn_accelery = false;
float kp, kd, ki;
float slow_down_kd;
int dv;
float angle_deviation = 0;
float speed_deviation = 0;
float cur_curvature_far = 0;
float cur_curvature_near = 0;
float cur_slope = 0;
float speed_result;
float angle_result = 0;
int circle_inside_count = 0;
int circle_last_y = IMGH - 2;
long long circle_out_ag = 0;
long long circle_out_sp = 0;

int hill_count = 0;//坡道中计数
int circle_count = 0;//环岛中计数
int zebra_count = 0;

int real_speed_enc = 0;//下位机传输的真实速度
int smoothed_real_speed_enc = 0;

bool ai_div;

//通信端口
serial::Serial ser;

void callback(int signum) {
	cerr << "Sender received signal, quit!" << endl;
	stop = true;
}


int main()
{
	int shem_count = 0;
	#pragma region 共享内存和信号
	//ctrl-z操作对应SIGTSTP信号，触发回调函数，停止程序
	signal(SIGTSTP, callback);
	//互斥信号量
	int image_sem = GetSem(IMAGE_ID);
	InitSem(image_sem);
	int result_sem = GetSem(RESULT_ID);
	InitSem(result_sem);

	//初始化两块共享内存
	int result_shm = GetShm(RESULT_SHM, 4096);
	int image_shm = GetShm(IMAGE_SHM, 44 * 4096);
	usleep(1000);
	//-存储结果的一块共享内存数组
	int* addr = (int*)shmat(result_shm, NULL, 0);
	//--AI元素识别标记（18th，待更新）
	//---'N'->none
	//---'B'->bridge
	//---'T'->tractor
	//---'C'->corn
	//---'P'->pig
	addr[0] = 'N';
	//--AI识别使能，0时AI冷却0.02s，1时识别AI元素
	addr[1] = 0;
	//--计数器：当前处理的图像数，主循环次数
	addr[2] = 0;

	//-存储当前相机图像的一块共享内存二维数组
	void* image_addr = shmat(image_shm, NULL, 0);
	#pragma endregion
	// auto now = std::chrono::system_clock::now();
	// std::time_t time = std::chrono::system_clock::to_time_t(now);
	// std::tm* start_tm;
	// start_tm = std::localtime(&time);

	//状态初始化，否则state_out初始化为garage_out
	if (!Re.main.garage_start) {
		MI.state_out = straight;
	}

	#pragma region 串口
	//尝试开启串口
	bool ser_ok = false;
	int ser_index = 0;
	while(!(ser_ok || ser_index > 10))
	{
		try {
			ser.setPort("/dev/ttyUSB" + to_string(ser_index));
			ser.setBaudrate(115200);
			serial::Timeout out = serial::Timeout::simpleTimeout(1000);
			ser.setTimeout(out);
			ser.open();
			ser_ok = true;
		}
		catch (serial::IOException& e) {
			ser_index ++;
		}
	}
	if(!ser_ok)
	{
		cerr << "Unable to open port!" << endl;
		return -1;
	}
	#pragma endregion
	
	//初始化保存的视频文件Open操作
	if (Re.set.video_save)
	{
		if (Re.set.color)MI.store.wri.open("Word.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 60, cv::Size(300, 200));
		else MI.store.wri.open("Word.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 60, cv::Size(320, 240));
		MI.store.Writer_Exist = true;
	}

	//主循环
	// ser.write("Hello World!");
	start_time_stamp = std::chrono::high_resolution_clock::now();
	cerr<< "control ready!"<<endl;
	while (!stop)
	{
		shem_count++;
		// now = std::chrono::system_clock::now();
		// time = std::chrono::system_clock::to_time_t(now);
		// std::tm* tm;
		// tm = std::localtime(&time);
		// cout << "test" << endl;
		// std::cout << "运行时间："<< tm->tm_hour - start_tm->tm_hour << ":" << tm->tm_min - start_tm->tm_min << ":" << tm->tm_sec - start_tm->tm_sec << std::flush;
		
		// cout << " " << endl;
		// cout << "time:" << MI.store.save_num  << endl;

		//更新处理后的图像和特征点查询结果
		MI.update_image();
		if(ai_div)
		{
			semP(image_sem);
			matWrite(image_addr, MI.store.image_BGR);
			semV(image_sem);
		}
		ai_div = ~ai_div;
	
		// // TEST 上位机通信测试代码
		// for(int i = -1023; i <= 1023; i+=5)
		// {
		// 	angle_result = i;
		// 	speed_result = 10;
		// 	encode_and_send();
		// }
		// for(int i = 1023; i >= -1023; i-=5)
		// {
		// 	angle_result = i;
		// 	speed_result = 5;
		// 	encode_and_send();
		// }
		// continue;

		real_speed_enc = get_speed_enc();
		cout << "true_speed_enc: " << real_speed_enc << endl;
		if(smoothed_real_speed_enc != 0)
		{
			smoothed_real_speed_enc = 0.1 * real_speed_enc + 0.9 * smoothed_real_speed_enc;
		}
		else
		{
			smoothed_real_speed_enc = real_speed_enc;
		}
		MI.last_enc_speed = MI.enc_speed;
		MI.enc_speed = smoothed_real_speed_enc;
		//只有在straight状态下识别AI元素
		if (MI.state_out == straight || MI.state_out == turn_state || MI.state_out == cone_find)
		{
			semP(result_sem);
			//开启AI识别
			addr[1] = 1;
			addr[2] = MI.store.save_num;
			MI.ai_bridge = false;
			MI.ai_tractor = false;
			MI.ai_corn = false;
			MI.ai_pig = false;
			MI.ai_bomb = false;
			MI.ai_right_garage = false;
			MI.ai_left_garage = false;
			if (addr[0] == 'B') { addr[0] = 'N'; MI.ai_bridge = true; }
			else if (addr[0] == 'O') { addr[0] = 'N'; MI.ai_bomb = true; }
			else if (addr[0] == 'R') { addr[0] = 'N'; MI.ai_right_garage = true; }
			else if (addr[0] == 'L') { addr[0] = 'N'; MI.ai_left_garage = true; }
			semV(result_sem);
		}
		else
		{
			semP(result_sem);
			//在特殊路段关闭AI识别
			addr[0] = 'N';
			addr[1] = 0;
			semV(result_sem);
		}

		cur_curvature_near = MI.get_curvature_near();
		cur_curvature_far = MI.get_curvature_far();
		cur_slope = MI.get_slope_near();
		//更新当前状态
		MI.state_judge();
		MI.update_control(kp, kd, ki, dv,slow_down_kd);

		AC.reset(kp, kd, ki);

		switch (MI.state_out)
		{
		case straight:
		case turn_state:
		{
			// MI.state_cone = judge_side;
			MI.mend_trunk();
			break;
		}
		case right_garage_find:
		{
			switch(MI.state_right_garage)
			{
				case right_garage_before:
				{
					// Point cone;	
					// // cone.x = 0;
					// // cone.y = IMGH-1;
					// MI.mend_trunk();
					// MI.refind_edge_point();
					// MI.find_center();
					// int cone_point = 0;
					// for(int i = IMGH-1; i > 0; i--){
					// 	uchar* row = MI.store.image_mat.ptr<uchar>(i);
					// 	int cone_flag = 0;
					// 	for(int j = MI.right_edge_point[i]+1; j < MI.right_edge_point[i]+10; j++){
					// 		if(row[j]==0){
					// 			cone_flag++;
					// 		}
					// 		cv::Vec3b val = MI.store.image_BGR_small.at<cv::Vec3b>(i, j);
				
					// 		if(val[0]<=50 && val[1]>=120 && val[2]>=120){
					// 			// int r,g,b;
					// 			// b = val[0];
					// 			// g = val[1];
					// 			// r = val[2];
					// 		    // cout<<"r "<<r<<" g "<<g<<" b "<<b<<endl;	
					// 			// cout<<i<<endl;
					// 			// cout<< "cone flag" << cone_flag << endl;
					// 			if(i > IMGH-5){
					// 				MI.top_cone.x = j;
					// 				MI.top_cone.y = i;
									
					// 				MI.state_right_garage = right_garage_into;
					// 				break;
					// 			}
							
					// 		}
					// 	}
					// }
					// break;


					MI.mend_trunk();
					MI.refind_edge_point();
					MI.find_center();
					Point record;
					int cone_count = 0;
					int cone_point = 0;
					for(int i = IMGH-1; i >  IMGH-30; i--){
						uchar* row_r = MI.store.image_mat.ptr<uchar>(i);
						uchar* row_b = MI.store.image_mat_cone.ptr<uchar>(i);
						int cone_flag = 0;
						
						for(int j = MI.left_edge_point[i]-15; j < MI.left_edge_point[i]; j++){
							
							// if(row_r[j]==0){
							// 	cone_flag++;
							// }
							// cv::Vec3b val = MI.store.image_BGR_small.at<cv::Vec3b>(i, j);
				
							if(row_r[j]!=0&&row_b[j]==0&&row_r[j+1]!=0&&row_r[j-1]!=0){
								cone_count++;
								// int r,g,b;
								// b = val[0];
								// g = val[1];
								// r = val[2];
								record.x = j;
								record.y = i;
								MI.right_cone_point.push_back(record);
							    // cout<<"r "<<r<<" g "<<g<<" b "<<b<<endl;	
								// cout<<i<<endl;
								// cout<< "cone flag" << cone_flag << endl;
								if(cone_count>30){
									if(i > IMGH-20){
										MI.top_cone.x = j;
										MI.top_cone.y = i;
										
										
										MI.state_right_garage = right_garage_into;
										break;
									}
								}

							
							}
						}
						// cerr<<cone_count<<endl;
					}
					break;
				}
				case right_garage_into:
				{
					Point cone;	
					cone.x = 120;
					cone.y = 80;
					Point start;
					start.x = 0;
					start.y = IMGH-1;
					int cone_point = 0;
					for(int i = IMGH/2; i > 1; i--){
						uchar* row_r = MI.store.image_mat.ptr<uchar>(i);
						uchar* row_b = MI.store.image_mat_cone.ptr<uchar>(i);
						int count_cone = 0;
						for(int j = MI.right_edge_point[i]; j < IMGW-2; j++){							
							if(row_r[j]!=0 && row_r[j+1]==0){
								count_cone++;
								// MI.cone_number[cone_point].x = j;
								// MI.cone_number[cone_point].y = i;
								cone_point++;
								if(j > cone.x&&count_cone > 1){
									cone.x = j;
									cone.y = i;
									MI.top_cone.x = j;
									MI.top_cone.y = i;
								}
							}
							
						}
						//cerr<< "cone count"<< count_cone<< endl;
					}
					line(MI.store.image_mat, start, cone);
					MI.lost_right = true;
					MI.lost_left = false;
					MI.refind_edge_point_in_garage();
					MI.find_center();
					if(MI.center_lost > 110){
						MI.state_right_garage = right_garage_inside;
						break;
					}
					break;
					// Point cone;	
					// cone.x = 120;
					// cone.y = 80;
					// Point start;
					// start.x = 0;
					// start.y = IMGH-1;
					// int cone_point = 0;
					// for(int i = IMGH/2; i > 1; i--){
					// 	uchar* row = MI.store.image_mat.ptr<uchar>(i);
					// 	int count_cone = 0;
					// 	for(int j = MI.right_edge_point[i]; j < IMGW-2; j++){							
					// 		if(row[j]!=0 && row[j+1]==0){
					// 			count_cone++;
					// 			// MI.cone_number[cone_point].x = j;
					// 			// MI.cone_number[cone_point].y = i;
					// 			cone_point++;
					// 			if(j > cone.x&&count_cone > 1){
					// 				cone.x = j;
					// 				cone.y = i;
					// 				MI.top_cone.x = j;
					// 				MI.top_cone.y = i;
					// 			}
					// 		}
							
					// 	}
					// 	//cerr<< "cone count"<< count_cone<< endl;
					// }
					// line(MI.store.image_mat, start, cone);
					// MI.lost_right = true;
					// MI.lost_left = false;
					// MI.refind_edge_point_in_garage();
					// MI.find_center();
					// if(MI.center_lost > 110){
					// 	MI.state_right_garage = right_garage_inside;
					// 	break;
					// }
					// break;
				}
				case right_garage_inside:
				{
					// Point top_cone;
					MI.top_cone.x = IMGW/2;
					MI.top_cone.y = IMGH-1;
					Point record;
					// int cone_point = 0;
					for(int i = IMGH-1; i > IMGH/3; i--){
						uchar* row_r = MI.store.image_mat.ptr<uchar>(i);
						uchar* row_b = MI.store.image_mat_cone.ptr<uchar>(i);
						int cone_flag = 0;
						// if(count_white(MI.store.image_mat, i, IMGW-1, 0)>50)break;
						for(int j = 0; j < IMGW-1; j++){
							// if(row[j]==0){
							// 	cone_flag++;
							// }
							// cv::Vec3b val = MI.store.image_BGR_small.at<cv::Vec3b>(i, j);
							
							// cout<< "row1 color"<< row[j] << "row2 color"<< row[j+1]<<endl;

							if(row_r[j]!=0&&row_b[j]==0&&row_r[j+1]!=0&&row_r[j-1]!=0){
								record.x = j;
								record.y = i;
								MI.right_cone_point.push_back(record);
								
								//cerr<< "cone flag" << endl;
								// int r,g,b;
								// r = val[0];
								// g = val[1];
								// b = val[2];
								// cout<<"r "<<r<<" g "<<g<<" b "<<b<<endl;
								if(MI.top_cone.y > i){
									MI.top_cone.y = i;
									MI.top_cone.x = j;
								}
							
							}
						}
					}
					MI.find_center_in_garage(MI.top_cone);
					if(MI.top_cone.y > 50){
						MI.state_right_garage = right_garage_stoped;
						MI.garage_count = 0;
						break;
					}
					break;	
					// Point top_cone;
					// MI.top_cone.x = IMGW/2;
					// MI.top_cone.y = IMGH-20;
					// // int cone_point = 0;
					// for(int i = IMGH-1; i > 0; i--){
					// 	uchar* row = MI.store.image_mat.ptr<uchar>(i);
					// 	int cone_flag = 0;
					// 	for(int j = 0; j < IMGW-1; j++){
					// 		// if(row[j]==0){
					// 		// 	cone_flag++;
					// 		// }
					// 		cv::Vec3b val = MI.store.image_BGR_small.at<cv::Vec3b>(i, j);
							
					// 		// cout<< "row1 color"<< row[j] << "row2 color"<< row[j+1]<<endl;						
					// 		if(val[0]<=55 && val[1]>=120 && val[2]>=120){
					// 			//cerr<< "cone flag" << endl;
					// 			// int r,g,b;
					// 			// r = val[0];
					// 			// g = val[1];
					// 			// b = val[2];
					// 			// cout<<"r "<<r<<" g "<<g<<" b "<<b<<endl;
					// 			if(MI.top_cone.y > i){
					// 				MI.top_cone.y = i;
					// 				MI.top_cone.x = j;
					// 			}
							
					// 		}
					// 	}
					// }
					// MI.find_center_in_garage(MI.top_cone);
					// if(MI.top_cone.y > 50){
					// 	MI.state_right_garage = right_garage_stoped;
					// 	MI.garage_count = 0;
					// 	break;
					// }
					// break;	
				}
			
				case right_garage_stoped: {
					MI.garage_count++;
					if(MI.garage_count>50){
						MI.state_right_garage = right_garage_try_out;
						break;
					}
					break;
				}
				case right_garage_try_out: {
					if(MI.center_lost < 30&&MI.right_end_point[0].x > 100){
						MI.state_right_garage = right_garage_out;
						MI.state_out = straight;
						break;
					}
					break;

				}
				break;
			}
			break;
		}


		case left_garage_find:
		{
			switch(MI.state_left_garage)
			{
				case left_garage_before:
				{
					// Point cone;	
					// // cone.x = 0;
					// // cone.y = IMGH-1;
					// MI.mend_trunk();
					// MI.refind_edge_point();
					// MI.find_center();
					// int cone_point = 0;
					// for(int i = IMGH-1; i > 0; i--){
					// 	uchar* row = MI.store.image_mat.ptr<uchar>(i);
					// 	int cone_flag = 0;
					// 	for(int j = MI.right_edge_point[i]+1; j < MI.right_edge_point[i]+10; j++){
					// 		if(row[j]==0){
					// 			cone_flag++;
					// 		}
					// 		cv::Vec3b val = MI.store.image_BGR_small.at<cv::Vec3b>(i, j);
				
					// 		if(val[0]<=50 && val[1]>=120 && val[2]>=120){
					// 			// int r,g,b;
					// 			// b = val[0];
					// 			// g = val[1];
					// 			// r = val[2];
					// 		    // cout<<"r "<<r<<" g "<<g<<" b "<<b<<endl;	
					// 			// cout<<i<<endl;
					// 			// cout<< "cone flag" << cone_flag << endl;
					// 			if(i > IMGH-5){
					// 				MI.top_cone.x = j;
					// 				MI.top_cone.y = i;
									
					// 				MI.state_right_garage = right_garage_into;
					// 				break;
					// 			}
							
					// 		}
					// 	}
					// }
					// break;


					MI.mend_trunk();
					MI.refind_edge_point();
					MI.find_center();
					Point record;
					int cone_count = 0;
					int cone_point = 0;
					for(int i = IMGH-1; i >  IMGH-30; i--){
						uchar* row_r = MI.store.image_mat.ptr<uchar>(i);
						uchar* row_b = MI.store.image_mat_cone.ptr<uchar>(i);
						int cone_flag = 0;
						if(MI.exist_left_edge_point[i]){
							for(int j = MI.left_edge_point[i]-15; j < MI.left_edge_point[i]; j++){
								
								// if(row_r[j]==0){
								// 	cone_flag++;
								// }
								// cv::Vec3b val = MI.store.image_BGR_small.at<cv::Vec3b>(i, j);
					
								if(row_r[j]!=0&&row_b[j]==0&&row_r[j+1]!=0&&row_r[j-1]!=0){
									cone_count++;
									// int r,g,b;
									// b = val[0];
									// g = val[1];
									// r = val[2];
									record.x = j;
									record.y = i;
									MI.left_cone_point.push_back(record);
									// cout<<"r "<<r<<" g "<<g<<" b "<<b<<endl;	
									// cout<<i<<endl;
									// cout<< "cone flag" << cone_flag << endl;
									if(cone_count>30){
										if(i > IMGH-40){
											MI.top_cone.x = j;
											MI.top_cone.y = i;
											
											
											MI.state_left_garage = left_garage_into;
											break;
										}
									}

								
								}
							}
						}

						// cerr<<cone_count<<endl;
					}
					break;
				}
				case left_garage_into:
				{
					Point cone;	
					cone.x = 120;
					cone.y = 80;
					Point start;
					start.x = IMGW-1;
					start.y = IMGH-1;
					int cone_point = 0;
					for(int i = IMGH/2; i > 1; i--){
						uchar* row_r = MI.store.image_mat.ptr<uchar>(i);
						uchar* row_b = MI.store.image_mat_cone.ptr<uchar>(i);
						int count_cone = 0;
						for(int j = MI.left_edge_point[i]; j > 1; j--){							
							if(row_r[j]!=0 && row_r[j-1]==0){
								count_cone++;
								// MI.cone_number[cone_point].x = j;
								// MI.cone_number[cone_point].y = i;
								cone_point++;
								if(j < cone.x&&count_cone > 1){
									cone.x = j;
									cone.y = i;
									MI.top_cone.x = j;
									MI.top_cone.y = i;
								}
							}
							
						}
						//cerr<< "cone count"<< count_cone<< endl;
					}
					line(MI.store.image_mat, start, cone);
					MI.lost_right = false;
					MI.lost_left = true;
					MI.refind_edge_point_in_garage();
					MI.find_center();
					if(MI.center_lost > 110){
						MI.state_left_garage = left_garage_inside;
						break;
					}
					break;
					// Point cone;	
					// cone.x = 120;
					// cone.y = 80;
					// Point start;
					// start.x = 0;
					// start.y = IMGH-1;
					// int cone_point = 0;
					// for(int i = IMGH/2; i > 1; i--){
					// 	uchar* row = MI.store.image_mat.ptr<uchar>(i);
					// 	int count_cone = 0;
					// 	for(int j = MI.right_edge_point[i]; j < IMGW-2; j++){							
					// 		if(row[j]!=0 && row[j+1]==0){
					// 			count_cone++;
					// 			// MI.cone_number[cone_point].x = j;
					// 			// MI.cone_number[cone_point].y = i;
					// 			cone_point++;
					// 			if(j > cone.x&&count_cone > 1){
					// 				cone.x = j;
					// 				cone.y = i;
					// 				MI.top_cone.x = j;
					// 				MI.top_cone.y = i;
					// 			}
					// 		}
							
					// 	}
					// 	//cerr<< "cone count"<< count_cone<< endl;
					// }
					// line(MI.store.image_mat, start, cone);
					// MI.lost_right = true;
					// MI.lost_left = false;
					// MI.refind_edge_point_in_garage();
					// MI.find_center();
					// if(MI.center_lost > 110){
					// 	MI.state_right_garage = right_garage_inside;
					// 	break;
					// }
					// break;
				}
				case left_garage_inside:
				{
					// Point top_cone;
					MI.top_cone.x = IMGW/2;
					MI.top_cone.y = IMGH-1;
					Point record;
					// int cone_point = 0;
					for(int i = IMGH-1; i > IMGH/3; i--){
						uchar* row_r = MI.store.image_mat.ptr<uchar>(i);
						uchar* row_b = MI.store.image_mat_cone.ptr<uchar>(i);
						int cone_flag = 0;
						// if(count_white(MI.store.image_mat, i, IMGW-1, 0)>50)break;
						for(int j = 0; j < IMGW-1; j++){
							// if(row[j]==0){
							// 	cone_flag++;
							// }
							// cv::Vec3b val = MI.store.image_BGR_small.at<cv::Vec3b>(i, j);
							
							// cout<< "row1 color"<< row[j] << "row2 color"<< row[j+1]<<endl;

							if(row_r[j]!=0&&row_b[j]==0&&row_r[j+1]!=0&&row_r[j-1]!=0){
								record.x = j;
								record.y = i;
								MI.left_cone_point.push_back(record);
								
								//cerr<< "cone flag" << endl;
								// int r,g,b;
								// r = val[0];
								// g = val[1];
								// b = val[2];
								// cout<<"r "<<r<<" g "<<g<<" b "<<b<<endl;
								if(MI.top_cone.y > i){
									MI.top_cone.y = i;
									MI.top_cone.x = j;
								}
							
							}
						}
					}
					MI.find_center_in_garage(MI.top_cone);
					if(MI.top_cone.y > 50){
						MI.state_left_garage = left_garage_stoped;
						MI.garage_count = 0;
						break;
					}
					break;	
					// Point top_cone;
					// MI.top_cone.x = IMGW/2;
					// MI.top_cone.y = IMGH-20;
					// // int cone_point = 0;
					// for(int i = IMGH-1; i > 0; i--){
					// 	uchar* row = MI.store.image_mat.ptr<uchar>(i);
					// 	int cone_flag = 0;
					// 	for(int j = 0; j < IMGW-1; j++){
					// 		// if(row[j]==0){
					// 		// 	cone_flag++;
					// 		// }
					// 		cv::Vec3b val = MI.store.image_BGR_small.at<cv::Vec3b>(i, j);
							
					// 		// cout<< "row1 color"<< row[j] << "row2 color"<< row[j+1]<<endl;						
					// 		if(val[0]<=55 && val[1]>=120 && val[2]>=120){
					// 			//cerr<< "cone flag" << endl;
					// 			// int r,g,b;
					// 			// r = val[0];
					// 			// g = val[1];
					// 			// b = val[2];
					// 			// cout<<"r "<<r<<" g "<<g<<" b "<<b<<endl;
					// 			if(MI.top_cone.y > i){
					// 				MI.top_cone.y = i;
					// 				MI.top_cone.x = j;
					// 			}
							
					// 		}
					// 	}
					// }
					// MI.find_center_in_garage(MI.top_cone);
					// if(MI.top_cone.y > 50){
					// 	MI.state_right_garage = right_garage_stoped;
					// 	MI.garage_count = 0;
					// 	break;
					// }
					// break;	
				}
			
				case left_garage_stoped: {
					MI.garage_count++;
					if(MI.garage_count>50){
						MI.state_left_garage = left_garage_try_out;
						break;
					}
					break;
				}
				case left_garage_try_out: {
					if(MI.center_lost < 30&&MI.left_end_point[0].x > 100){
						MI.state_left_garage = left_garage_out;
						MI.state_out = straight;
						break;
					}
					break;

				}
				break;
			}
			break;
		}
		case cone_find: {
			switch (MI.state_cone){
				case judge_side:
				{
					MI.judge_cone_side();
					break;
				}
				case right_block:
				{
					MI.find_edge_point();//找边界点
					MI.find_end_point();//找断点
					MI.edge_filter(10);
					MI.find_end_point();
					if(MI.right_end_point.size()<3){
						MI.state_out = straight;
						MI.state_cone = judge_side;
						break;
					}
					int right_edge_flag;
					for(int i = MI.right_end_point[3].y; i < MI.right_end_point[2].y; i++){
						MI.exist_right_edge_point[i] = true;
						MI.right_edge_point[i] = MI.center_point[i]-10;
						right_edge_flag = MI.right_edge_point[i];
						
					}
					for(int i = MI.right_end_point[2].y; i < IMGH-1; i++){
						MI.exist_right_edge_point[i] = true;
						MI.right_edge_point[i] = right_edge_flag;
					}
					MI.find_center();

					break;
				}
				case left_block:
				{
					MI.find_edge_point();//找边界点
					MI.find_end_point();//找断点
					MI.edge_filter(10);
					MI.find_end_point();
					if(MI.left_end_point.size()<3){
						MI.state_out = straight;
						MI.state_cone = judge_side;
						break;
					}
					int left_edge_flag;
					for(int i = MI.left_end_point[3].y; i < MI.left_end_point[2].y; i++){
						MI.exist_left_edge_point[i] = true;
						MI.left_edge_point[i] = MI.center_point[i]-10;
						left_edge_flag = MI.left_edge_point[i];
						
					}
					for(int i = MI.left_end_point[2].y; i < IMGH-1; i++){
						MI.exist_left_edge_point[i] = true;
						MI.left_edge_point[i] = left_edge_flag;
					}
					MI.find_center();

					break;
				}
				// case first_left_cone:
				// {
				// 	MI.find_edge_point_for_cone();
				// 	break;
				// }
				break;
			}
			// Point left_cone;
			// Point right_cone;
			// left_cone.x = MI.left_edge_point[60];
			// left_cone.y = 60;
			// right_cone.x = MI.right_edge_point[60];
			// right_cone.y = 60;
			// int left_count=0, right_count=0;
			// int left_temp=0, right_temp = 0;
			// int yellow_y_bottom = 0;
			// for(int i = IMGH-1; i > MI.center_lost; i--){
				
			// 	uchar* row_b = MI.store.image_mat_cone.ptr<uchar>(i);
			// 	uchar* row_r = MI.store.image_mat.ptr<uchar>(i);
			// 	for(int j = MI.left_edge_point[i]; j < MI.center_point[i]; j++){
			// 		// cv::Vec3b val = MI.store.image_BGR_small.at<cv::Vec3b>(i, j);

			// 		if(row_b[j]==0&&row_r[j]!=0){
			// 			if(yellow_y_bottom < i)
			// 			{
			// 				yellow_y_bottom = i;
			// 			}
			// 			if(left_count==0 || (pow(left_cone.y-i,2)+pow(left_cone.x-j,2))<pow((20+0.15*i),2))
			// 			{
			// 				if(left_count==0)
			// 				{
			// 					left_cone.x=j;
			// 					left_cone.y=i;
			// 					left_temp=i;
			// 				}
			// 				else{
			// 					left_cone.x=(left_cone.x*(left_count-1)+j)/left_count;
			// 					left_cone.y=(left_cone.y*(left_count-1)+i)/left_count;
			// 					if(left_temp > i)
			// 					{
			// 						left_temp=i;
			// 					}
			// 				}
			// 				left_count++;
			// 			}
			// 		}
			// 	}

			// 	for(int k = MI.right_edge_point[i]; k > MI.center_point[i]; k--){
			// 		cv::Vec3b val = MI.store.image_BGR_small.at<cv::Vec3b>(i, k);

			// 		if(val[0]<=150 && val[1]>=120 && val[2]>=120){
			// 			if(right_count==0 || (pow(right_cone.y-i,2)+pow(right_cone.x-k,2))<pow((20+0.15*i),2))
			// 			{
			// 				if(right_count==0)
			// 				{
			// 					right_cone.x=k;
			// 					right_cone.y=i;
			// 					right_temp=i;
			// 				}
			// 				else{
			// 					right_cone.x=(right_cone.x*(right_count-1)+k)/right_count;
			// 					right_cone.y=(right_cone.y*(right_count-1)+i)/right_count;
			// 					if(right_temp > i)
			// 					{
			// 						right_temp=i;
			// 					}
			// 				}
			// 				right_count++;
			// 			}
			// 		}

			// 	}
			// }
			// cout << "yellow_y_bottom : " << yellow_y_bottom <<endl;
			// if(yellow_y_bottom > Re.main.cone_slowdown_thresh)
			// {
			// 	MI.state_cone = cone_slowdown;
			// }
			// left_cone.y=left_temp;
			// right_cone.y=right_temp;
			// if(right_cone.x > MI.center_point[right_cone.y]+15){
			// 	right_cone.x = MI.center_point[right_cone.y]+15;
			// }
			// // cerr<<"right_count"<< right_count<<"right_cone.x"<< right_cone.x << "right_cone.y" << right_cone.y<<endl;
			
			// if(left_cone.x < MI.center_point[left_cone.y]-15){
			// 	left_cone.x = MI.center_point[left_cone.y]-15;
			// }
			// if(left_count < 5&& right_count < 5){
			// 	MI.state_out = straight;
			// }
			// if(left_count>5)
			// {
			// 	for(int nnrow=0;nnrow<IMGH;nnrow++)
			// 	{
			// 		for(int nncol=left_cone.x+15;nncol>MI.left_edge_point[nnrow]&&nncol>left_cone.x-35;nncol--)
			// 		{
			// 			int nowrow=nnrow;
			// 			int nowcol=nncol;
			// 			if(nowrow<1)
			// 			{
			// 				nowrow=1;
			// 			}
			// 			if(nowrow>119)
			// 			{
			// 				nowrow=119;
			// 			}
			// 			if(nowcol<1)
			// 			{
			// 				nowrow=1;
			// 			}
			// 			if(nowcol>159)
			// 			{
			// 				nowcol=159;
			// 			}
			// 			 if(nowcol<(left_cone.x-2+Re.main.cone_trapezium_long+0.25*left_cone.y-Re.main.cone_trapezium_slope*(abs(nowrow*1.0-left_cone.y*1.0)/(1-Re.main.cone_trapezium_slope_with_y*left_cone.y))))
			// 			{
			// 				if(nowcol<left_cone.x-5+0.25*left_cone.y)
			// 				{
			// 					MI.store.image_mat.at<uchar>(nowrow, nowcol) = uchar(0);
			// 				}
			// 			}
			// 		}
			// 	}
			// }
			// if(right_count>5)
			// {
			// 	for(int nnrow=0;nnrow<IMGH;nnrow++)
			// 	{
			// 		for(int nncol=right_cone.x-15;nncol<MI.right_edge_point[nnrow]&&nncol<right_cone.x+35;nncol++)
			// 		{
					
			// 			int nowrow=nnrow;
			// 			int nowcol=nncol;
			// 			if(nowrow<1)
			// 			{
			// 				nowrow=1;
			// 			}
			// 			if(nowrow>119)
			// 			{
			// 				nowrow=119;
			// 			}
			// 			if(nowcol<1)
			// 			{
			// 				nowrow=1;
			// 			}
			// 			if(nowcol>159)
			// 			{
			// 				nowcol=159;
			// 			}
			// 			 if(nowcol>(right_cone.x+2-Re.main.cone_trapezium_long-0.25*right_cone.y+Re.main.cone_trapezium_slope*(abs(nowrow*1.0-right_cone.y*1.0)/(1-Re.main.cone_trapezium_slope_with_y*right_cone.y))))
			// 			{
			// 				if(nowcol>right_cone.x+5-0.25*right_cone.y)
			// 				{
			// 					MI.store.image_mat.at<uchar>(nowrow, nowcol) = uchar(0);
			// 				}
			// 			}
			// 		}
			// 	}
			// }

			// if(MI.store.cone_flag == false){
			// 	MI.state_out = straight;
			// }
			break;
		}
		case start_state:{
			if(j_zebra_line(MI, MI.state_out)){
				break;
			}
			else{
				MI.state_out = straight;
			}
		}
		case garage_out: {
			uchar* row = MI.store.image_mat.ptr<uchar>(IMGH - Re.start.start_dist);
			if (Re.start.left) {
				if (row[IMGW - 1] == 0) {
					int i, b_count = 1;
					for (i = IMGW - 2; i > 0; i--) {
						if (row[i] == 0) b_count++;
						else break;
					}
					if (b_count > Re.start.x_thresh)
					{
						MI.state_out = straight;
					}
				}
			}
			else {
				if (row[0] == 0) {
					int i, b_count = 1;
					for (i = 1; i < IMGW; i++) {
						if (row[i] == 0) {
							b_count++;
							cout << "b_count" << b_count << endl;
						}
						else break;
					}
					if (b_count > Re.start.x_thresh)
					{
						MI.state_out = straight;
					}
				}
			}
			break;
		}
		case right_circle: {
			switch (MI.state_r_circle)
			{
			case right_circle_in_find: {
				if (MI.right_end_point.size() == 4 && MI.right_end_point[0].y > 40) {
					MI.state_r_circle = right_circle_in_strai;
					//circle_last_y = IMGH - 2;
				}
				else {
					if (MI.right_end_point.size() >= 6) {
						line(MI.store.image_mat, MI.right_end_point[1], MI.right_end_point[3]);
						line(MI.store.image_mat, MI.right_end_point[3], MI.right_end_point[4]);
					}
					else {
						line(MI.store.image_mat, MI.right_end_point[1], MI.right_end_point[3]);
						ray(MI.store.image_mat, MI.right_end_point[3], Re.r_circle.in_find_ray_ag);
					}
					//circle_last_y = MI.right_end_point[0].y;
				}
				break;
			}
			case right_circle_in_strai: {
				if (j_right_circle_in_circle(MI, MI.state_r_circle))
				{
					MI.state_r_circle = right_circle_in_circle;
					MI.mend_right_circle_in_straight();
					break;
				}
				MI.mend_right_circle_in_straight();
				MI.center_lost = MI.right_end_point[2].y;

				break;
			}
			case right_circle_in_circle: {
				MI.lost_right = true;
				if (j_right_circle_inside_before(MI, MI.state_r_circle))break;
				MI.mend_right_circle_in_circle();
				Point p = Re.r_circle.inside_before_p;
				line(MI.store.image_mat, MI.left_end_point[0], p);
				break;
			}
			case right_circle_inside_before: {
				MI.lost_right = true;
				Point p = Re.r_circle.inside_before_p;
				line(MI.store.image_mat, MI.left_end_point[0], p);
				MI.center_lost = MI.left_end_point.back().y;
				if (j_right_circle_inside(MI, MI.state_r_circle))circle_inside_count++;
				else circle_inside_count = 0;
				if (circle_inside_count > 3) {
					MI.state_r_circle = right_circle_inside;
					circle_inside_count = 0;
				}
				break;
			}
			case right_circle_inside: {
				circle_inside_count = 0;
				MI.mend_trunk();
				j_right_circle_out_find(MI, MI.state_r_circle);
				break;
			}
			case right_circle_out_find: {
				Point p = Re.r_circle.out_find_p;
				if (MI.left_end_point.size() >= 1) {
					line(MI.store.image_mat, MI.left_end_point[1], p);
				}
				j_right_circle_out_strai(MI, MI.state_r_circle);
				break;
			}
			case right_circle_out_strai: {
				//MI.mend_trunk();
				Point p1, p2;
				p1.x = (MI.left_end_point[0].x + MI.left_end_point[1].x) / 2;
				p1.y = (MI.left_end_point[0].y + MI.left_end_point[1].y) / 2;
				p2.x = 0;
				p2.y = IMGH - 1;
				line(MI.store.image_mat, p1, p2);
				line(MI.store.image_mat, MI.right_end_point[0], MI.right_end_point[1]);
				line(MI.store.image_mat, MI.right_end_point[1], MI.right_end_point[2]);
				if (MI.left_end_point.size() != 0 && (MI.left_end_point[0].y > IMGH - 10 || MI.left_end_point[1].x < IMGW/2) && MI.left_end_point[1].y < IMGH - 40) {
					MI.state_r_circle = right_circle_out_out;
				}
				break;
			}
			// case right_circle_out: {
			// 	j_right_circle_out_out(MI, MI.state_r_circle);
			// 	if (MI.right_end_point.size() != 0) {
			// 		ray(MI.store.image_mat, MI.right_end_point[0], Re.r_circle.out_ray_ag);
			// 	}
			// 	break;
			// }
			case right_circle_out_out: {
				MI.mend_trunk();
				MI.state_out = straight;
				MI.state_r_circle = right_circle_in_find;
				circle_out_ag = 0;
				circle_out_sp = 0;
				circle_count = 0;
				break;
			}
			}
			break;
		}
		case left_circle: {
			switch (MI.state_l_circle)
			{
			case left_circle_in_find: {
				if (MI.left_end_point.size() == 4 && MI.left_end_point[0].y > 40) {
					MI.state_l_circle = left_circle_in_strai;
					//circle_last_y = IMGH - 2;
				}
				else {
					if (MI.left_end_point.size() >= 6) {
						line(MI.store.image_mat, MI.left_end_point[1], MI.left_end_point[3]);
						line(MI.store.image_mat, MI.left_end_point[3], MI.left_end_point[4]);
					}
					else {
						line(MI.store.image_mat, MI.left_end_point[1], MI.left_end_point[3]);
						ray(MI.store.image_mat, MI.left_end_point[3], Re.l_circle.in_find_ray_ag);
					}
					//circle_last_y = MI.left_end_point[0].y;
				}
				break;
			}
			case left_circle_in_strai: {
				if (j_left_circle_in_circle(MI, MI.state_l_circle))
				{
					MI.state_l_circle = left_circle_in_circle;
					break;
				}
				MI.mend_left_circle_in_straight();
				MI.center_lost = MI.left_end_point[2].y;

				break;
			}
			case left_circle_in_circle: {
				MI.lost_left = true;
				if (j_left_circle_inside_before(MI, MI.state_l_circle))break;
				MI.mend_left_circle_in_circle();
				break;
			}
			case left_circle_inside_before: {
				MI.lost_left = true;
				Point p = Re.l_circle.inside_before_p;
				line(MI.store.image_mat, MI.left_end_point[0], p);
				MI.center_lost = MI.right_end_point.back().y;
				if (j_left_circle_inside(MI, MI.state_l_circle))circle_inside_count++;
				else circle_inside_count = 0;
				if (circle_inside_count > 3) {
					MI.state_l_circle = left_circle_inside;
					circle_inside_count = 0;
				}
				break;
			}
			case left_circle_inside: {
				circle_inside_count = 0;
				MI.mend_trunk();
				j_left_circle_out_find(MI, MI.state_l_circle);
				break;
			}
			case left_circle_out_find: {
				Point p = Re.l_circle.out_find_p;
				if (MI.right_end_point.size() >= 1) {
					line(MI.store.image_mat, MI.right_end_point[1], p);//2.6
				}
				j_left_circle_out_strai(MI, MI.state_l_circle);
				break;
			}
			case left_circle_out_strai: {
				//MI.mend_trunk();
				Point p1, p2;
				p1.x = (MI.right_end_point[0].x + MI.right_end_point[1].x) / 2;
				p1.y = (MI.right_end_point[0].y + MI.right_end_point[1].y) / 2;
				p2.x = IMGW - 1;
				p2.y = IMGH - 1;
				line(MI.store.image_mat, p1, p2);
				if (MI.right_end_point.size() != 0 && MI.right_end_point[0].y > IMGH - 10 && MI.right_end_point[1].y < IMGH - 40) {
					MI.state_l_circle = left_circle_out;
				}
				break;
			}
			case left_circle_out: {
				j_left_circle_out_out(MI, MI.state_l_circle);
				if (MI.left_end_point.size() != 0) {
					ray(MI.store.image_mat, MI.left_end_point[0], Re.l_circle.out_ray_ag);
				}
				break;
			}
			case left_circle_out_out: {
				MI.mend_trunk();
				MI.state_out = straight;
				MI.state_l_circle = left_circle_in_find;
				circle_out_ag = 0;
				circle_out_sp = 0;
				circle_count = 0;
				break;
			}
			}
			break;
		}
		case repair_find: {
			break;
		}
		case farm_find: {
			switch (MI.state_farm)
			{
			case farm_in_find: {
				int begin;
				if (MI.left_end_point.size() > 0 && MI.right_end_point.size() > 0)
				{
					begin = MI.left_end_point[1].y < MI.right_end_point[1].y ? MI.left_end_point[1].y : MI.right_end_point[1].y;
					MI.count_cone(begin);
				}
				MI.refind_edge_point();
				MI.find_center();
				MI.find_center_in_farm(begin);
				//MI.mend_farm_in_find();
				j_farm_inside(MI, MI.state_farm);
				break;
			}
			case farm_inside: {
				MI.count_cone();
				for (int i = IMGH - 1; i > 0; i--) {
					MI.exist_left_edge_point[i] = false;
					MI.exist_right_edge_point[i] = false;
				}
				MI.find_center_in_farm();
				//MI.mend_farm_inside();
				j_farm_out_find(MI, MI.state_farm);
				j_farm_out(MI, MI.state_farm);
				break;
			}
			case farm_out_find: {
				MI.count_cone();
				MI.mend_farm_out_find();
				MI.find_center_in_farm();

				MI.refind_edge_in_farm_out(MI.center_cone.back());
				j_farm_out(MI, MI.state_farm);
				break;
			}//弃用
			case farm_out: {
				MI.state_out = straight;
				//MI.state_farm = farm_in_find;
				MI.right_cone.clear();
				MI.left_cone.clear();
				MI.center_cone.clear();
				break;
			}
			}
			break;
		}
		case hump_find: {
			switch (MI.state_hump)
			{
			case hump_in_find: {
				j_hump_on(MI, MI.state_hump);
				break;
			}
			case hump_on: {
				MI.mend_in_hump_on();
				j_hump_out(MI, MI.state_hump);
				break;
			}
			case hump_out: {
				MI.state_out = straight;
				//MI.state_hump = hump_in_find;
				break;
			}
			}
			break;
		}
		case hill_find: {
			switch (MI.state_hill)
			{
			case hill_on: {
				int center_error;
				Point mid_bot, mid_top;
				mid_bot.y = IMGH - Re.hill.mid_bot_y;
				mid_top.y = IMGH - Re.hill.mid_top_y;
				if (MI.exist_left_edge_point[IMGH - Re.hill.mid_bot_y] && MI.exist_right_edge_point[IMGH - Re.hill.mid_bot_y]) {
					mid_bot.x = (MI.left_edge_point[IMGH - Re.hill.mid_bot_y] + MI.right_edge_point[IMGH - Re.hill.mid_bot_y]) / 2;
				}
				else if (MI.exist_left_edge_point[IMGH - Re.hill.mid_bot_y]) {
					mid_bot.x = (MI.left_edge_point[IMGH - Re.hill.mid_bot_y] + IMGW - 1) / 2;
				}
				else if (MI.exist_right_edge_point[IMGH - Re.hill.mid_bot_y]) {
					mid_bot.x = (MI.right_edge_point[IMGH - Re.hill.mid_bot_y]) / 2;
				}
				if (MI.exist_left_edge_point[IMGH - Re.hill.mid_top_y] && MI.exist_right_edge_point[IMGH - Re.hill.mid_top_y]) {
					mid_top.x = (MI.left_edge_point[IMGH - Re.hill.mid_top_y] + MI.right_edge_point[IMGH - Re.hill.mid_top_y]) / 2;
				}
				else if (MI.exist_left_edge_point[IMGH - Re.hill.mid_top_y]) {
					mid_top.x = (MI.left_edge_point[IMGH - Re.hill.mid_top_y] + IMGW - 1) / 2;
				}
				else if (MI.exist_right_edge_point[IMGH - Re.hill.mid_top_y]) {
					mid_top.x = (MI.right_edge_point[IMGH - Re.hill.mid_top_y]) / 2;
				}
				center_error = abs(mid_bot.x - mid_top.x);
				cout << "center_error: " << center_error << endl;
				if (center_error < 15) {
					hill_count++;
				}
				if (hill_count > Re.hill.frame) {
					MI.state_hill = hill_out;
					hill_count = 0;
				}
				break;
			}
			case hill_out: {
				MI.state_out = straight;
				//MI.state_hill = hill_on;
				break;
			}
			}
			break;
		}
		case state_end: {
			switch (MI.state_end_line)
			{
			case end_line_find: {
				MI.find_edge_point_end();
				MI.find_center();
				if (MI.zebra_near_find) {
					MI.end_count = 0;
					MI.state_end_line = end_acc;
				}
				break;
			}
			case end_acc: {
				// int count = 0;
				// uchar* r = MI.store.image_mat.ptr<uchar>(IMGH - Re.end.end_dist);
				// for (int i = 0; i < IMGW; i++) {
				// 	if (r[i] == 0)count++;
				// }
				// if (count > IMGW - Re.end.end_whitecount) {
				// 	MI.state_in_garage = garage_inside;
				// }
				MI.find_edge_point_end();
				MI.find_center();
				if(j_zebra_line(MI,MI.state_end_line)){
					break;
				}
				else{
					MI.end_count++;
				}
				if(MI.end_count > 5){
					MI.state_end_line = race_end;
				}
				break;
			}
			case race_end: {
				stop = true;
				break;
			}
			default:
				break;
			}
			break;
		}
		}
		//重补线
		if (MI.state_out != right_garage_find && MI.state_out != left_garage_find && MI.state_out != hump_find && MI.state_out!=cone_find&&MI.state_out != state_end) {
			MI.refind_edge_point();
			MI.find_center();
		}
		// if(MI.state_out == state_end){
		// 	MI.find_center();
		// }
		//运动控制
		angle_deviation = MI.AngelDeviation();
		speed_deviation = MI.SpeedDeviation();

		if (MI.state_out == right_circle)
		{
			if (MI.state_r_circle == right_circle_out_strai)
			{
				//取circle_inside时的速度和角度的平均值作为结果
				angle_result = circle_out_ag / float(circle_count - Re.r_circle.count_start);
				speed_result = circle_out_sp / float(circle_count - Re.r_circle.count_start);
			}
			else if(MI.state_r_circle == right_circle_in_find)
			{
				angle_result = AC.output(angle_deviation);
				speed_result = Re.r_circle.in_find_speed;
			}
			else
			{
				angle_result = AC.output(angle_deviation);
				speed_result = Re.r_circle.speed;
			}
		}
		else if (MI.state_out == left_circle)
		{
			if (MI.state_l_circle == left_circle_out_strai)
			{
				angle_result = circle_out_ag / float(circle_count - Re.l_circle.count_start);
				speed_result = circle_out_sp / float(circle_count - Re.l_circle.count_start);
			}
			else if(MI.state_l_circle == left_circle_in_find)
			{
				angle_result = AC.output(angle_deviation);
				speed_result = Re.l_circle.in_find_speed;
			}
			else
			{
				angle_result = AC.output(angle_deviation);
				speed_result = Re.l_circle.speed;
			}
		}
		else if (MI.state_out == right_garage_find){
			if(MI.state_right_garage == right_garage_inside)
			{
				angle_result = AC.output(angle_deviation);
				speed_result = (MI.top_cone.y - 50) / 2; //5;
				MI.garage_speed.push_back(speed_result);
				MI.garage_angle.push_back(angle_result);	
			}
			else if(MI.state_right_garage == right_garage_stoped){
				angle_result = 0;
				speed_result = 0;				
			}
			else if(MI.state_right_garage == right_garage_try_out){
				if(MI.garage_speed.size()==1||MI.garage_angle.size()==1){
					angle_result = MI.garage_angle.back();
					speed_result = -MI.garage_speed.back();						
				}
				else{
					angle_result = MI.garage_angle.back();
					speed_result = -MI.garage_speed.back();	
					MI.garage_speed.pop_back();
					MI.garage_angle.pop_back();	
				}
			}
			else if(MI.state_right_garage == right_garage_into){
			angle_result = AC.output(angle_deviation);
			speed_result = 5; //SC.output(speed_deviation);
			MI.garage_speed.push_back(speed_result);
			MI.garage_angle.push_back(angle_result);		
			}
			else if(MI.state_right_garage == right_garage_before){
			angle_result = AC.output(angle_deviation);
			speed_result = 5; //SC.output(speed_deviation);				
			}
			else{
			angle_result = AC.output(angle_deviation);
			speed_result = SC.output(speed_deviation);		
			}
		}
		else if (MI.state_out == left_garage_find){
			if(MI.state_left_garage == left_garage_inside)
			{
				angle_result = AC.output(angle_deviation);
				speed_result = (MI.top_cone.y - 50) / 2; //5;
				MI.garage_speed.push_back(speed_result);
				MI.garage_angle.push_back(angle_result);	
			}
			else if(MI.state_left_garage == left_garage_stoped){
				angle_result = 0;
				speed_result = 0;				
			}
			else if(MI.state_left_garage == left_garage_try_out){
				if(MI.garage_speed.size()==1||MI.garage_angle.size()==1){
					angle_result = MI.garage_angle.back();
					speed_result = -MI.garage_speed.back();						
				}
				else{
					angle_result = MI.garage_angle.back();
					speed_result = -MI.garage_speed.back();	
					MI.garage_speed.pop_back();
					MI.garage_angle.pop_back();	
				}
			}
			else if(MI.state_left_garage == left_garage_into){
			angle_result = AC.output(angle_deviation);
			speed_result = 5; //SC.output(speed_deviation);
			MI.garage_speed.push_back(speed_result);
			MI.garage_angle.push_back(angle_result);		
			}
			else if(MI.state_left_garage == left_garage_before){
			angle_result = AC.output(angle_deviation);
			speed_result = 5; //SC.output(speed_deviation);				
			}
			else{
			angle_result = AC.output(angle_deviation);
			speed_result = SC.output(speed_deviation);		
			}
		}
		else if (MI.state_out == cone_find){
			angle_result = AC.output(angle_deviation);
			
			if(MI.state_cone == cone_slowdown|| MI.state_cone == judge_side || MI.state_cone == right_block || MI.state_cone == left_block)
			{
				speed_result = Re.main.cone_speed;
			}
			else 
			{
				speed_result = SC.output(speed_deviation);	
			}
			
		}
		else if (MI.state_out == repair_find) {
			angle_result = AC.output(angle_deviation);
			speed_result = Re.repair.speed;
		}
		else if (MI.state_out == farm_find) {
			angle_result = AC.output(angle_deviation);
			speed_result = Re.farm.speed;
			if (MI.state_farm == farm_out_find)
			{
				speed_result = Re.farm.speed_out;
			}
		}
		else if (MI.state_out == hump_find) {
			angle_result = AC.output(angle_deviation);
			speed_result = Re.hump.speed;
		}
		else if (MI.state_out == hill_find) {
			angle_result = AC.output(angle_deviation);
			speed_result = Re.hill.speed;
		}
		else if (MI.state_out == start_state){
			angle_result = AC.output(angle_deviation);//Re.start.start_angle;
			speed_result = Re.start.start_speed;			
		}
		else if (MI.state_out == garage_out) {
			if (Re.start.left) {
				angle_result = Re.start.v_left.first;
				speed_result = Re.start.v_left.second;
			}
			else {
				angle_result = Re.start.v_right.first;
				speed_result = Re.start.v_right.second;
			}
		}
		else if ((MI.state_out == state_end && MI.state_end_line == garage_in_find) || (MI.state_out == state_end && MI.state_end_line == end_acc)) {
			// int start_left = Re.start.left;
			// if (start_left) {
			// 	angle_result = Re.zebra.v_left_zebra.first;
			// 	speed_result = Re.zebra.v_left_zebra.second;
			// }
			// else {
			// 	angle_result = Re.zebra.v_right_zebra.first;
			// 	speed_result = Re.zebra.v_right_zebra.second;
			angle_result = AC.output(angle_deviation);;//Re.end.end_angle; // Re.end.v_left_garage.first;
			speed_result = Re.end.end_speed;//Re.end.v_left_garage.second;
			// }
		}
		// else if (MI.state_out == garage_find && MI.state_in_garage == garage_in) {
		// 	// int start_left = Re.start.left;
		// 	// if (start_left) {
		// 	angle_result = Re.end.end_angle; // Re.end.v_left_garage.first;
		// 	speed_result = Re.end.end_speed;//Re.end.v_left_garage.second;
		// 	// }
		// 	// else {
		// 	// 	angle_result = Re.end.v_right_garage.first;
		// 	// 	speed_result = Re.end.v_right_garage.second;
		// 	// }
		// }
		else if(MI.state_out == turn_state)
		{
			angle_result = AC.output(Re.turn.angle_ctrl_deviation_coef * angle_deviation +
								     Re.turn.angle_ctrl_slope_coef * cur_slope);
			if(MI.state_turn_state == turn_slow_down)
			{
				// disable_motor = false;
				speed_result = Re.turn.speed_in;
				cout <<"speed in: " << Re.turn.speed_in << endl;
			}
			else if(MI.state_turn_state == turn_inside)
			{
				speed_result = SC_turn.output(Re.turn.speed_ctrl_deviation_coef * speed_deviation + 
									 	 Re.turn.speed_ctrl_slope_coef * cur_slope);
				cout <<"turn speed: " << speed_result << endl;
			}
			else 
			{
				speed_result = SC.output(Re.main.deviation_coef * speed_deviation + 
									 Re.main.slope_coef * cur_slope);
			}
		}
		else {
			disable_motor = false;
			float angle_result_tmp = 0;
			angle_result_tmp = AC.output(angle_deviation);
			if((abs(angle_result - angle_result_tmp) > 500) && (angle_result * angle_result_tmp > 0))
			{
				angle_result = 0.9 * angle_result_tmp + 0.1 * angle_result;
			}
			else if( (abs(angle_result - angle_result_tmp) > 1000) && (angle_result * angle_result_tmp < 0))
			{
				angle_result = 0.4 * angle_result_tmp + 0.6 * angle_result;
			}
			else
			{
				angle_result = angle_result_tmp;
			}
			speed_result = SC.output(Re.main.deviation_coef * abs(speed_deviation) + 
									 Re.main.slope_coef * abs(cur_slope));
		}

		//通过增量提升刹车性能
		
		speed_result = SC.output_reduced(speed_result,real_speed_enc,slow_down_kd);

		if (!Re.set.motor_use)speed_result = 0;
		//环岛计数
		if (MI.state_out == left_circle && MI.state_l_circle == left_circle_inside &&
			circle_count < (100 + Re.l_circle.count_start)) {
			//做了两帧的延时
			//盲猜是为了防止记录到进环岛时的转向动作 by reverie 24/1/30
			if (circle_count >= Re.l_circle.count_start) {
				circle_out_ag += angle_result;
				circle_out_sp += speed_result;
			}
			circle_count++;
		}
		else if (MI.state_out == right_circle && MI.state_r_circle == right_circle_inside &&
			circle_count <= (100 + Re.r_circle.count_start)) {
			if (circle_count > Re.r_circle.count_start) {
				circle_out_ag += angle_result;
				circle_out_sp += speed_result;
			}
			circle_count++;
		}

		encode_and_send();

		#pragma region 打印当前状态
		switch (MI.state_out)
		{
			case start_state: cout << "start_state" << endl; break;
			case straight: cout << "straight" << endl; break;
			case turn_state: cout << "turn_state" << endl; break;
			case garage_out: cout << "garage_out" << endl; break;
			case hill_find: cout << "hill_find" << endl; break;
			case right_garage_find:
			{
				switch (MI.state_right_garage)
				{
					case right_garage_before:{cout << "right_garage_before" << endl; break;}
					case right_garage_into:{cout << "right_garage_into" << endl; break;}
					case right_garage_inside:{cout << "right_garage_inside" << endl; break;}
				}
			}
			case right_circle:
			{
				switch (MI.state_r_circle)
				{
				case right_circle_in_find: {cout << "right_circle_in_find" << endl; break; }
				case right_circle_in_strai: {cout << "right_circle_in_strai" << endl; break; }
				case right_circle_in_circle: {cout << "right_circle_in_circle" << endl; break; }
				case right_circle_inside_before: {cout << "right_circle_inside_before" << endl; break; }
				case right_circle_inside: {cout << "right_circle_inside" << endl; break; }
				case right_circle_out_find: {cout << "right_circle_out_find" << endl; break; }
				case right_circle_out_strai: {cout << "right_circle_out_strai" << endl; break; }
				case right_circle_out: {cout << "right_circle_out" << endl; break; }
				case right_circle_out_out: {cout << "right_circle_out_out" << endl; break; }
				}
				break;
			}
			case left_circle:
			{
				switch (MI.state_l_circle)
				{
				case left_circle_in_find: {cout << "left_circle_in_find" << endl; break; }
				case left_circle_in_strai: {cout << "left_circle_in_strai" << endl; break; }
				case left_circle_in_circle: {cout << "left_circle_in_circle" << endl; break; }
				case left_circle_inside_before: {cout << "left_circle_inside_before" << endl; break; }
				case left_circle_inside: {cout << "left_circle_inside" << endl; break; }
				case left_circle_out_find: {cout << "left_circle_out_find" << endl; break; }
				case left_circle_out_strai: {cout << "left_circle_out_strai" << endl; break; }
				case left_circle_out: {cout << "left_circle_out" << endl; break; }
				case left_circle_out_out: {cout << "left_circle_out_out" << endl; break; }
				}
				break;
			}
			case repair_find:
			{
				break;
			}
			case farm_find:
			{
				switch (MI.state_farm)
				{
				case farm_in_find: {cout << "farm_in_find" << endl; break; }
				case farm_inside: {cout << "farm_inside" << endl; break; }
				case farm_out_find: {cout << "farm_out_find" << endl; break; }
				case farm_out: {cout << "farm_out" << endl; break; }
				}
				break;
			}
			case hump_find:
			{
				switch (MI.state_hump)
				{
				case hump_in_find: {cout << "hump_in_find" << endl; break; }
				case hump_on: {cout << "hump_on" << endl; break; }
				case hump_out: {cout << "hump_out" << endl; break; }
				}
				break;
			}
			case state_end:
			{
				switch (MI.state_end_line)
				{
				case end_line_find: {cout << "end_line_find" << endl; break; }
				// case garage_in_before: {cout << "garage_in_before" << endl; break; }
				case end_acc: {cout << "end_acc" << endl; break; }
				case race_end: {cout << "race_end" << endl; break; }
				}
				break;
			}
		}
		#pragma endregion

		//保存图像
		if (Re.set.video_save) MI.show(angle_deviation, angle_result, speed_result, real_speed_enc);

		MI.store.save_num++;

		//if (waitKey(5) >= 0)
		//	break;
	}
	cout << "OUT!" << endl;
	angle_result = 0;
	speed_result = 0;
	for(int i = 0; i < 10; i++)
		encode_and_send();
	shmdt(addr);
	shmdt(image_addr);

	cout << "STOP" << endl;

	MI.store.cap.release();
	if (MI.store.Writer_Exist) {
		MI.store.wri.release();
		cout << "wri_stop" << endl;
	}
	return 0;
}