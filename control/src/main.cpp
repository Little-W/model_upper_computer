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
using namespace std;
using namespace cv;

#define Re MI.re

MainImage MI;
AngleControl AC(Re.main.kp, Re.main.kd, Re.main.max_ag, Re.main.min_ag);
SpeedControl SC(Re.main.min_v_diff, Re.main.max_v_diff, Re.main.max_v, Re.main.min_v);

float kp, kd;
int dv;
float deviation = 0;
schar speed_result, run_set = 'r';
float angle_result = 0;
int circle_inside_count = 0;
int circle_last_y = IMGH - 2;
long long circle_out_ag = 0;
long long circle_out_sp = 0;

int hill_count = 0;//坡道中计数
int circle_count = 0;//环岛中计数
int zebra_count = 0;


int stop = false;

//通信端口
serial::Serial ser;

void callback(int signum) {
	cerr << "Sender received signal, quit!" << endl;
	stop = true;
}

/*
	下位机速度控制范围  +- 31
	舵机控制范围       +- 1200
*/
void encode_and_send(void)
{
	vector<unsigned char> result;
	uchar speed_result_tmp;
	uint angle_result_tmp = 0;
	angle_result *= 6;
	speed_result_tmp = speed_result < 0 ? -speed_result : speed_result;
	angle_result_tmp = angle_result < 0 ? -angle_result : angle_result;
	speed_result_tmp /= 20;
	if(angle_result_tmp > 1200) angle_result_tmp = 1200;
	uchar speed_code,servo_code_p1,servo_code_p2;
	speed_code = speed_result_tmp & 0x1f;
	if(speed_result < 0) speed_code |= 0x20;
	servo_code_p1 = (angle_result_tmp >> 6) & 0x1f;
	if(angle_result < 0) servo_code_p1 |= 0x20;
	servo_code_p1 |= 0x80;
	servo_code_p2 = angle_result_tmp & 0x3f;
	servo_code_p2 |= 0xc0;
	result.push_back(0x55); //传输开始标志
	result.push_back(speed_code);
	result.push_back(servo_code_p1);
	result.push_back(servo_code_p2);
	result.push_back(0x6a);	//传输结束标志
	ser.write(result);
	cout << "Angle: " << dec << angle_result  << "	Speed: " << (int)speed_result / 20 << endl;
	cout << "************************************************************************"<< endl;
	cout << "HEX:   Angle: "  << hex << (uint)servo_code_p1  << (uint)servo_code_p1 << "	Speed: " << (uint)speed_code <<  endl;
	cout << "************************************************************************"<< dec << endl;
}

int main()
{
	signal(SIGTSTP, callback);
	//共享内存
	int image_sem = GetSem(IMAGE_ID);
	InitSem(image_sem);
	int result_sem = GetSem(RESULT_ID);
	InitSem(result_sem);
	int result_shm = GetShm(RESULT_SHM, 4096);
	int image_shm = GetShm(IMAGE_SHM, 44 * 4096);
	usleep(1000);
	int* addr = (int*)shmat(result_shm, NULL, 0);
	addr[0] = 'N';
	addr[1] = 0;
	addr[2] = 0;
	void* image_addr = shmat(image_shm, NULL, 0);

	// auto now = std::chrono::system_clock::now();
	// std::time_t time = std::chrono::system_clock::to_time_t(now);
	// std::tm* start_tm;
	// start_tm = std::localtime(&time);

	if (!Re.main.garage_start) {
		MI.state_out = straight;
	}
	try {
		ser.setPort("/dev/ttyUSB0");
		ser.setBaudrate(115200);
		serial::Timeout out = serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(out);
		ser.open();
	}
	catch (serial::IOException& e) {
		cerr << "Unable to open port!" << endl;
		return -1;
	}
	printf("test_p1\n");
	//保存视频
	Re.set.video_save = false;
	if (Re.set.video_save)
	{
		if (Re.set.color)MI.store.wri.open("Word.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25, cv::Size(300, 200));
		else MI.store.wri.open("Word.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25, cv::Size(320, 240));
		MI.store.Writer_Exist = true;
	}
	//主循环
	// ser.write("Hello World!");
	while (!stop)
	{
		// now = std::chrono::system_clock::now();
		// time = std::chrono::system_clock::to_time_t(now);
		// std::tm* tm;
		// tm = std::localtime(&time);
		// cout << "test" << endl;
		// std::cout << "运行时间："<< tm->tm_hour - start_tm->tm_hour << ":" << tm->tm_min - start_tm->tm_min << ":" << tm->tm_sec - start_tm->tm_sec << std::flush;
		
		cout << " " << endl;
		cout << MI.store.save_num << endl;
		MI.update_image();
		semP(image_sem);
		matWrite(image_addr, MI.store.image_BGR);
		semV(image_sem);
		if (MI.state_out == straight)
		{
			semP(result_sem);
			addr[1] = 1;
			addr[2] = MI.store.save_num;
			MI.ai_bridge = false;
			MI.ai_tractor = false;
			MI.ai_corn = false;
			MI.ai_pig = false;
			if (addr[0] == 'B') { addr[0] = 'N'; MI.ai_bridge = true; }
			else if (addr[0] == 'T') { addr[0] = 'N'; MI.ai_tractor = true; }
			else if (addr[0] == 'C') { addr[0] = 'N'; MI.ai_corn = true; }
			else if (addr[0] == 'P') { addr[0] = 'N'; MI.ai_pig = true; }
			semV(result_sem);
		}
		else
		{
			semP(result_sem);
			addr[0] = 'N';
			addr[1] = 0;
			semV(result_sem);
		}

		MI.state_judge();
		MI.update_control(kp, kd, dv);

		AC.reset(kp, kd);

		switch (MI.state_out)
		{
		case straight:
		{
			MI.mend_trunk();
			break;
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
				if (MI.left_end_point.size() != 0 && MI.left_end_point[0].y > IMGH - 10 && MI.left_end_point[1].y < IMGH - 40) {
					MI.state_r_circle = right_circle_out;
				}
				break;
			}
			case right_circle_out: {
				j_right_circle_out_out(MI, MI.state_r_circle);
				if (MI.right_end_point.size() != 0) {
					ray(MI.store.image_mat, MI.right_end_point[0], Re.r_circle.out_ray_ag);
				}
				break;
			}
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
		case garage_find: {
			switch (MI.state_in_garage)
			{
			case garage_in_find: {
				if (MI.zebra_near_find) {
					MI.state_in_garage = garage_in;
				}
				break;
			}
			case garage_in: {
				int count = 0;
				uchar* r = MI.store.image_mat.ptr<uchar>(IMGH - Re.end.end_dist);
				for (int i = 0; i < IMGW; i++) {
					if (r[i] == 0)count++;
				}
				if (count > IMGW - Re.end.end_whitecount) {
					MI.state_in_garage = garage_inside;
				}
				break;
			}
			case garage_inside: {
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
		if (MI.state_out != farm_find && MI.state_out != hump_find) {
			MI.refind_edge_point();
			MI.find_center();
		}
		//运动控制
		deviation = MI.MidlineDeviation();

		if (MI.state_out == right_circle)
		{
			if (MI.state_r_circle == right_circle_out_strai)
			{
				angle_result = circle_out_ag / float(circle_count - Re.r_circle.count_start);
				speed_result = circle_out_sp / float(circle_count - Re.r_circle.count_start);
			}
			else
			{
				angle_result = AC.output(deviation);
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
			else
			{
				angle_result = AC.output(deviation);
				speed_result = Re.l_circle.speed;
			}
		}
		else if (MI.state_out == repair_find) {
			angle_result = AC.output(deviation);
			speed_result = Re.repair.speed;
		}
		else if (MI.state_out == farm_find) {
			angle_result = AC.output(deviation);
			speed_result = Re.farm.speed;
			if (MI.state_farm == farm_out_find)
			{
				speed_result = Re.farm.speed_out;
			}
		}
		else if (MI.state_out == hump_find) {
			angle_result = AC.output(deviation);
			speed_result = Re.hump.speed;
		}
		else if (MI.state_out == hill_find) {
			angle_result = AC.output(deviation);
			speed_result = Re.hill.speed;
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
		else if ((MI.state_out == garage_find && MI.state_in_garage == garage_in_find) || (MI.state_out == garage_find && MI.state_in_garage == garage_in_before)) {
			int start_left = Re.start.left;
			if (start_left) {
				angle_result = Re.zebra.v_left_zebra.first;
				speed_result = Re.zebra.v_left_zebra.second;
			}
			else {
				angle_result = Re.zebra.v_right_zebra.first;
				speed_result = Re.zebra.v_right_zebra.second;
			}
		}
		else if (MI.state_out == garage_find && MI.state_in_garage == garage_in) {
			int start_left = Re.start.left;
			if (start_left) {
				angle_result = Re.end.v_left_garage.first;
				speed_result = Re.end.v_left_garage.second;
			}
			else {
				angle_result = Re.end.v_right_garage.first;
				speed_result = Re.end.v_right_garage.second;
			}
		}
		else {
			angle_result = AC.output(deviation);
			speed_result = SC.output(deviation);
		}

		if (!Re.set.motor_use)speed_result = 0;
		//环岛计数
		if (MI.state_out == left_circle && MI.state_l_circle == left_circle_inside &&
			circle_count < (100 + Re.l_circle.count_start)) {
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

		switch (MI.state_out)
		{
		case straight: cout << "straight" << endl; break;
		case garage_out: cout << "garage_out" << endl; break;
		case hill_find: cout << "hill_find" << endl; break;
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
		case garage_find:
		{
			switch (MI.state_in_garage)
			{
			case garage_in_find: {cout << "garage_in_find" << endl; break; }
			case garage_in_before: {cout << "garage_in_before" << endl; break; }
			case garage_in: {cout << "garage_in" << endl; break; }
			case garage_inside: {cout << "garage_inside" << endl; break; }
			}
			break;
		}
		}


		if (Re.set.video_save) MI.show(deviation);

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