#include <iostream>
#include <stdlib.h>
#include "process.h"
#include <chrono>
#include "pid.h"
#include "comm.h"

#define USE_VIDEO 0

bool r_circle_use = true;
bool l_circle_use = true;
bool l_circle_big_circle = true;
bool r_circle_big_circle = true;
bool video_ok = false;

Mat get_frame(VideoCapture cap) {
	Mat frame;
	cap.read(frame);
	return frame;
}

float MainImage::dy_forward_dist_up(float kp, float kd, float coef, float exp, float bias, 
									float speed_thresh, float max)
{
	float speed_error = 0, last_speed_error = 0, new_forward_dist;
	last_speed_error = last_enc_speed - speed_thresh;
	if(enc_speed > speed_thresh)
	{
		speed_error = enc_speed - speed_thresh;
		speed_error = kp * speed_error + kd * (speed_error - last_speed_error);
		if(speed_error < 0)  speed_error = 0;
		new_forward_dist = bias + coef * pow(speed_error,exp) / 1000.0;
		if(new_forward_dist > max)
		{
			new_forward_dist = max;
		}
	}
	else
	{
		new_forward_dist = bias;
	}
	return new_forward_dist;
}

float MainImage::dy_forward_dist_up_and_down(float kp, float kd, float coef_up, float coef_down, float exp_up, 
											 float exp_down, float bias, float speed_thresh, float max, float min)
{
	float speed_error = 0, last_speed_error = 0, new_forward_dist;
	last_speed_error = last_enc_speed - speed_thresh;
	speed_error = enc_speed - speed_thresh;
	speed_error = kp * speed_error + kd * (speed_error - last_speed_error);
	if(speed_error >= 0)
	{
		new_forward_dist = bias + coef_up * pow(speed_error,exp_up) / 1000.0;
		if(new_forward_dist > max)
		{
			new_forward_dist = max;
		}
	}
	else
	{
		new_forward_dist = bias - coef_down * pow(-speed_error,exp_up) / 1000.0;
		if(new_forward_dist < min)
		{
			new_forward_dist = min;
		}
	}
	return new_forward_dist;
}

float MainImage::AngelDeviation(void)
{
	int i, j;
	int center = IMGW / 2;
	// center += 10;
	//int center=(IMGW/2+last_center)/2;
	long long sum;
	
	if (state_out == farm_find) {
		MainImage::deviation_thresh = IMGH - re.farm.dist;
		sum = 0;
		for (i = MainImage::deviation_thresh; i > MainImage::deviation_thresh - re.farm.up_scope; i--) {
			sum += center_point[i];
		}
		angle_deviation = re.main.forward_coef1 * (center - float(sum) / re.farm.up_scope);
		
		sum = 0;
		for (i = MainImage::deviation_thresh + re.farm.down_scope; i > MainImage::deviation_thresh; i--) {
			sum += center_point[i];
		}
		angle_deviation += re.main.forward_coef2 * (center - float(sum) / re.farm.down_scope);
		cout << "deviation: " << angle_deviation << endl;
	}
	else {
		//速度大于阈值时对前瞻进行动态调整:forward_dist up_scope forward_coef1
		double speed_error;

		if (state_out == right_circle) 
		{
			if (MI.state_r_circle == right_circle_inside_before ||
				MI.state_r_circle == right_circle_in_circle ||
				MI.state_r_circle == right_circle_in_strai)
			{
				MainImage::angle_new_forward_dist = 
					dy_forward_dist_up_and_down(re.r_circle.angle_forward_dist_kp, re.r_circle.angle_forward_dist_kd,
												re.r_circle.dy_forward_dist_coef_up, re.r_circle.dy_forward_dist_coef_down,
												re.r_circle.dy_forward_dist_exp_up, re.r_circle.dy_forward_dist_exp_down,
												re.r_circle.circle_dist, re.r_circle.speed * ENC_SPEED_SCALE,
												re.r_circle.max_dy_forward_dist, re.r_circle.min_dy_forward_dist);
				cout << "forward_dist is: " << MainImage::angle_new_forward_dist << endl;
				MainImage::deviation_thresh = IMGH - MainImage::angle_new_forward_dist;
			}
			else
			{
				MainImage::deviation_thresh = IMGH - re.r_circle.circle_dist;
			}
		}
		else if (state_out == left_circle) 
		{
			if (MI.state_l_circle == left_circle_inside_before ||
				MI.state_l_circle == left_circle_in_circle ||
				MI.state_l_circle == left_circle_in_strai)
			{
				MainImage::angle_new_forward_dist = 
					dy_forward_dist_up_and_down(re.l_circle.angle_forward_dist_kp, re.l_circle.angle_forward_dist_kd,
												re.l_circle.dy_forward_dist_coef_up, re.l_circle.dy_forward_dist_coef_down,
												re.l_circle.dy_forward_dist_exp_up, re.l_circle.dy_forward_dist_exp_down,
												re.l_circle.circle_dist, re.l_circle.speed * ENC_SPEED_SCALE,
												re.l_circle.max_dy_forward_dist, re.l_circle.min_dy_forward_dist);
				cout << "forward_dist is: " << MainImage::angle_new_forward_dist << endl;
				MainImage::deviation_thresh = IMGH - MainImage::angle_new_forward_dist;
			}
			else
			{
				MainImage::deviation_thresh = IMGH - re.l_circle.circle_dist;
			}
		}
		else
		{
			MainImage::angle_new_forward_dist = 
				dy_forward_dist_up(re.main.angle_dy_forward_dist_kp,re.main.angle_dy_forward_dist_kd,
								   re.main.angle_enc_forward_dist_coef,re.main.angle_enc_forward_dist_exp,
								   re.main.forward_dist, re.main.angle_enc_forward_threshold,
								   re.main.angle_max_enc_forward_dist);
			cout << "forward_dist is: " << MainImage::angle_new_forward_dist << endl;
			MainImage::deviation_thresh = IMGH - MainImage::angle_new_forward_dist;
		}
		while ( MainImage::deviation_thresh < center_lost && MainImage::deviation_thresh < IMGH - 1)
		{
			MainImage::deviation_thresh ++;
		}
		
		sum = 0;
		for (i = MainImage::deviation_thresh; i > MainImage::deviation_thresh - re.main.up_scope; i--) {
			sum += center_point[i];
		}
  		angle_deviation = re.main.forward_coef1 * (center - float(sum) / re.main.up_scope);
		sum = 0;
		for (i = MainImage::deviation_thresh + re.main.down_scope; i > MainImage::deviation_thresh; i--) {
			sum += center_point[i];
		}
		angle_deviation += re.main.forward_coef2 * (center - float(sum) / re.main.down_scope);
		// deviation += 18.0;
		//if (deviation < -15) deviation -= 4;
		//else if (deviation > 15) deviation += 4;
		cout << "deviation: " << angle_deviation << endl;
	
	}
	return angle_deviation;
}
float MainImage::SpeedDeviation(void)
{
	
	int i, j;
	float speed_deviation = 0;
	int center = IMGW / 2;
	// center += 10;
	//int center=(IMGW/2+last_center)/2;
	long long sum;

	//速度大于阈值时对前瞻进行动态调整:forward_dist forward_coef1
	double new_forward_dist =
		dy_forward_dist_up(re.main.speed_dy_forward_dist_kp, re.main.speed_dy_forward_dist_kd,
						   re.main.speed_enc_forward_dist_coef, re.main.speed_enc_forward_dist_exp,
						   re.main.speed_forward_dist, re.main.speed_enc_forward_threshold,
						   re.main.speed_max_enc_forward_dist);
						   
	cout << "speed forward_dist is: " << new_forward_dist << endl;
	MainImage::speed_deviation_thresh = IMGH - new_forward_dist;	

	sum = 0;
	// update_forward_dist();
	for (i = MainImage::speed_deviation_thresh; i > MainImage::speed_deviation_thresh - re.main.up_scope; i--) {
		sum += center_point[i];
	}
	//根据速度动态调整权重
	
	speed_deviation = re.main.forward_coef1 * (center - float(sum) / re.main.up_scope);
	sum = 0;
	for (i = MainImage::speed_deviation_thresh + re.main.down_scope; i > MainImage::speed_deviation_thresh; i--) {
		sum += center_point[i];
	}
	speed_deviation += re.main.forward_coef2 * (center - float(sum) / re.main.down_scope);
	// deviation += 18.0;
	//if (deviation < -15) deviation -= 4;
	//else if (deviation > 15) deviation += 4;
	cout << "speed deviation: " << speed_deviation << endl;
	
	return speed_deviation;
}
ImageStorage::ImageStorage()
{
#if USE_VIDEO == 1
	cap.open("./sample.avi");//这个路径就是当前路径的意思
#else
	int video_index = 0;
	while(!(video_ok || video_index > 10))
	{
		cap.open("/dev/video" + to_string(video_index), cv::CAP_V4L2);
		if (!cap.isOpened()) {
			video_index ++;
		}
		else
		{
			video_ok = true;
		}
	}
#endif
	if (!video_ok) {
		std::cout << "An error occured!!!" << std::endl;
	}
	cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));  //'M', 'J', 'P', 'G'
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	cap.set(cv::CAP_PROP_FPS, 60);

	fut = async(launch::async, get_frame, cap);
}

void ImageStorage::get_image(int x, int y, int w, int h, int ai_x, int ai_y, int ai_w, int ai_h, bool f)
{
	bool success;
	static std::chrono::time_point<std::chrono::high_resolution_clock> last_frame_ts = std::chrono::high_resolution_clock::now();
	cv::Mat frame, frame_tmp;
	vector<cv::Mat> image_split;
	auto start = std::chrono::high_resolution_clock::now();
	do {
		success = true;
#if USE_VIDEO == 1
		std::chrono::duration<double, std::milli> elapsed = std::chrono::high_resolution_clock::now() - last_frame_ts;
		if(elapsed.count() > 1000.0 / 60.0)
		{
			std::cout << "Frame Elapsed Time: " << elapsed.count() << " ms" << std::endl;
			frame = fut.get();
			fut = async(launch::async, get_frame, cap);
			if (frame.empty()) {
				success = false;
				cout << "frame is empty" << endl;
				stop = true;
				return;
			}
			else
			{
				last_frame_ts = std::chrono::high_resolution_clock::now();
			}
			if(elapsed.count() > 200)
			{
				stop = true;
				return;
			}
		}
		else
		{
			success = false;
		}
#else
		success = true;
		frame = fut.get();
		fut = async(launch::async, get_frame, cap);
		if (frame.empty()) {
			success = false;
		}
#endif
	} while (!success);
	//cv::Rect roi(50, 180, 540, 280);
	cv::Rect roi(x, y, w, h);
	image_BGR = frame;
	frame = frame(roi);
	split(frame, image_split);

	cv::Rect roi2(ai_x, ai_y, ai_w, ai_h);
	image_BGR = image_BGR(roi2);
	resize(image_BGR, image_BGR, cv::Size(300, 200));
	image_R = image_split[2];

	resize(image_R, image_R, cv::Size(IMGW, IMGH));//120*160
	threshold(image_R, image_mat, 0, 255, THRESH_BINARY | THRESH_OTSU);//大津法
	if (f)
	{
		Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
		morphologyEx(image_mat, image_mat, MORPH_CLOSE, kernel);//腐蚀膨胀
	}
}
//构造函数，进行初始化操作
MainImage::MainImage() : re("./config.yaml")
{
	init(true);
	left_end_point.reserve(16);
	right_end_point.reserve(16);
	state_out = garage_out;
	state_r_circle = right_circle_in_find;
	state_l_circle = left_circle_in_find;
	state_repair = repair_in_find;
	state_farm = farm_in_find;
	state_hump = hump_in_find;
	state_in_garage = garage_in_find;
	state_hill = hill_on;
	last_center = IMGW / 2;
	count_circle = 0;
	zebra_far_find = false;
	zebra_near_find = false;
	
	ai_bridge = false;
	ai_tractor = false;
	ai_corn = false;
	ai_pig = false;
}
/**
 * @brief 初始化特征点
*/
void MainImage::init(bool complete = true)
{
	for (int i = 0; i < IMGH; i++)
	{
		left_edge_point[i] = 0;
		right_edge_point[i] = IMGW - 1;
		center_point[i] = 0;

		exist_left_edge_point[i] = false;
		exist_right_edge_point[i] = false;
	}
	if (complete)
	{
		left_end_point.clear();
		right_end_point.clear();
		left_cone.clear();
		right_cone.clear();
		center_cone.clear();
		lost_left = false;
		lost_right = false;
		left_continue = true;
		right_continue = true;
		left_branch_num = 0;
		right_branch_num = 0;

		center_lost = -1;
	}
}
/**
 * @brief 更新处理后的图像和特征点查询结果
*/
void MainImage::update_image()
{
	if (state_out == straight)
		store.get_image(re.set.cut_point_x, re.set.cut_point_y, re.set.cut_w, re.set.cut_h, re.set.ai_cut_x, re.set.ai_cut_y, re.set.ai_w, re.set.ai_h, false);
	else
		store.get_image(re.set.cut_point_x, re.set.cut_point_y, re.set.cut_w, re.set.cut_h, re.set.ai_cut_x, re.set.ai_cut_y, re.set.ai_w, re.set.ai_h, true);
	init();
	find_edge_point();//找边界点
	find_end_point();//找断点
	if (state_out == straight) edge_filter(12);
	find_end_point();
	end_filter();
	judge_lost();
	count_branch();
	judge_continuity();
}

void MainImage::state_judge()
{
	if (ai_bridge) {
		ai_bridge = false;
		if (state_hill != hill_out)state_out = hill_find;
	}
	else if (ai_tractor) {
		ai_tractor = false;
		if (state_repair != repair_out_out)state_out = repair_find;
	}
	else if (ai_corn) {
		ai_corn = false;
		if (state_farm != farm_out)state_out = farm_find;
	}
	else if (ai_pig) {
		ai_pig = false;
		if (state_hump != hump_out)state_out = hump_find;
	}
	if (state_out == straight)
	{
		if (re.set.zebra_detect) {
			find_far_zebra();
			find_near_zebra();
		}
		if (zebra_far_find) {
			state_out = garage_find;
		}
		else if (r_circle_use && right_end_point.size() >= 6 && right_branch_num >= 2 && abs(right_end_point[1].y - right_end_point[4].y) > 35)
		{
			bool f = true;
			int i;
			for (i = right_end_point[1].y - 10; i > right_end_point[4].y - 1; i--) {
				if (!(exist_left_edge_point[i] && abs(left_edge_point[i] - left_edge_point[i + 1]) < 3)) {
					f = false;
					break;
				}
			}
			if (f) {
				state_out = right_circle;
			}
		}
		else if (l_circle_use && left_end_point.size() >= 6 && left_branch_num >= 2 && abs(left_end_point[1].y - left_end_point[4].y) > 35) {
			bool f = true;
			int i;
			for (i = left_end_point[1].y - 10; i > left_end_point[4].y - 1; i--) {
				if (!(exist_right_edge_point[i] && abs(right_edge_point[i] - right_edge_point[i + 1]) < 3)) {
					f = false;
					break;
				}
			}
			if (f) {
				state_out = left_circle;
			}
		}
	}
	else if (state_out == garage_find) {
		find_far_zebra();
		find_near_zebra();
	}
}

void MainImage::update_control(float& kp, float& kd, float& ki, int& dv)
{
	switch (state_out) {
	case garage_out: {
		kp = re.start.kp;
		kd = re.start.kd;
		ki = re.start.ki;
		dv = re.start.dv;
		return;
	}
	case straight: {
		kp = re.main.kp;
		kd = re.main.kd;
		ki = re.main.ki;
		dv = re.main.dv;
		return;
	}
	case right_circle: {
		kp = re.r_circle.kp;
		kd = re.r_circle.kd;
		ki = re.r_circle.ki;
		dv = re.r_circle.dv;
		return;
	}
	case left_circle: {
		kp = re.l_circle.kp;
		kd = re.l_circle.kd;
		ki = re.l_circle.ki;
		dv = re.l_circle.dv;
		return;
	}
	case repair_find: {
		kp = re.repair.kp;
		kd = re.repair.kd;
		ki = re.repair.ki;
		dv = re.repair.dv;
		return;
	}
	case farm_find: {
		kp = re.farm.kp;
		kd = re.farm.kd;
		ki = re.farm.ki;
		dv = re.farm.dv;
		return;
	}
	case hump_find: {
		kp = re.hump.kp;
		kd = re.hump.kd;
		ki = re.hump.ki;
		dv = re.hump.dv;
		return;
	}
	case garage_find: {
		kp = re.zebra.kp;
		kd = re.zebra.kd;
		ki = re.zebra.ki;
		dv = re.zebra.dv;
		return;
	}
	case hill_find: {
		kp = re.hill.kp;
		kd = re.hill.kd;
		ki = re.hill.ki;
		dv = re.hill.dv;
		return;
	}
	}
}

void MainImage::mend_trunk()
{
	int i;
	if (!lost_left && !lost_right) {
		if (left_end_point.size() > 1) {
			for (i = 1; i < left_end_point.size() - 1; i += 2) {
				line(store.image_mat, left_end_point[i], left_end_point[i + 1]);
			}
			if (left_end_point[0].y < IMGH - 5)
				ray(store.image_mat, left_end_point[0], -1.74f);
		}
		if (right_end_point.size() > 1) {
			for (i = 1; i < right_end_point.size() - 1; i += 2) {
				line(store.image_mat, right_end_point[i], right_end_point[i + 1]);
			}
			if (right_end_point[0].y < IMGH - 5)

				ray(store.image_mat, right_end_point[0], -1.4f);
		}
		// if (right_end_point.size() == 1 && left_end_point.size() == 1){
		// 	if(right_end_point[0].y < left_end_point[0].y){
		// 		ray(store.image_mat, right_end_point[0], re.main.right_ray);
		// 	}
		// 	if(left_end_point[0].y < left_end_point[0].y){
		// 		ray(store.image_mat, right_end_point[0], re.main.left_ray);
		// 	}
		// }
	}
	if (lost_left) {
		for (int k = right_end_point[1].y; k < IMGH - 2; k++) {
			if (right_edge_point[k + 1] > right_edge_point[k]) {
				Point p;
				p.y = k;
				p.x = right_edge_point[k];
				float angle = atan(2.0 / (right_edge_point[k] - right_edge_point[k + 1])) + 3.14f;
				ray(store.image_mat, p, angle);
				break;
			}
		}
	}
	else if (lost_right) {
		for (int k = left_end_point[1].y; k < IMGH - 2; k++) {
			if (left_edge_point[k + 1] < left_edge_point[k]) {
				Point p;
				p.y = k;
				p.x = left_edge_point[k];
				float angle = atan(2.0 / (left_edge_point[k] - left_edge_point[k + 2]));
				ray(store.image_mat, p, angle);
				break;
			}
		}
	}
}

void MainImage::mend_right_circle_in_straight()
{
	Point p;
	p.x = IMGW - 1;
	int i;
	for (i = right_end_point[0].y; i > right_end_point[1].y; i--) {
		if (right_edge_point[i] < p.x) {
			p.x = right_edge_point[i];
			p.y = i;
		}
	}
	ray(store.image_mat, p, re.r_circle.in_strai_ray_ag1);
	ray(store.image_mat, right_end_point[2], re.r_circle.in_strai_ray_ag2);
	for (int j = right_end_point[2].y; j > 0; j--){
		exist_left_edge_point[j] = false;
		exist_right_edge_point[j] = false;

	}
}

void MainImage::mend_right_circle_in_circle()
{
	Point p;
	p.x = 0;
	p.y = IMGH - 5;
	if (right_end_point.size() > 0) {
		if (right_end_point[0].x > IMGW - MINX || right_end_point[0].y > IMGH - MINY) {
			if (right_end_point[2].x > IMGW - 26) {
				ray(store.image_mat, p, re.r_circle.in_circle_ray_ag1);
			}
			else {
				Point p1;
				p1.x = 0;
				int i;
				for (i = right_end_point[0].y; i > right_end_point[1].y; i--) {
					if (right_edge_point[i] > p1.x) {
						p1.x = right_edge_point[i];
						p1.y = i;
					}
				}
				ray(store.image_mat, p1, re.r_circle.in_circle_ray_ag2);
				ray(store.image_mat, right_end_point[2], re.r_circle.in_circle_ray_ag3);
			}
		}
		else {
			ray(store.image_mat, right_end_point[0], re.r_circle.in_circle_ray_ag4);
		}
	}
	else {
		ray(store.image_mat, p, re.r_circle.in_circle_ray_ag5);
	}
}

void MainImage::mend_left_circle_in_straight() {
	Point p;
	p.x = 0;
	int i;
	for (i = left_end_point[0].y; i > left_end_point[1].y; i--) {
		if (left_edge_point[i] > p.x) {
			p.x = left_edge_point[i];
			p.y = i;
		}
	}
	ray(store.image_mat, p, re.l_circle.in_strai_ray_ag1);
	ray(store.image_mat, left_end_point[2], re.l_circle.in_strai_ray_ag2);
}

void MainImage::mend_left_circle_in_circle() {
	Point p;
	p.x = IMGW - 1;
	p.y = IMGH - 5;
	if (left_end_point.size() > 0) {
		if (left_end_point[0].x < MINX || left_end_point[0].y > IMGH - MINY) {
			if (left_end_point[2].x < 26) {
				ray(store.image_mat, p, re.l_circle.in_circle_ray_ag1);
			}
			else {
				Point p1;
				p1.x = 0;
				int i;
				for (i = left_end_point[0].y; i > left_end_point[1].y; i--) {
					if (left_edge_point[i] > p1.x) {
						p1.x = left_edge_point[i];
						p1.y = i;
					}
				}
				ray(store.image_mat, p1, re.l_circle.in_circle_ray_ag2);
				ray(store.image_mat, left_end_point[2], re.l_circle.in_circle_ray_ag3);
			}
		}
		else {
			ray(store.image_mat, left_end_point[0], re.l_circle.in_circle_ray_ag4);
		}
	}
	else {
		ray(store.image_mat, p, re.l_circle.in_circle_ray_ag5);
	}
}

void MainImage::mend_farm_in_find() {
	int i;
	Point p;
	int size = left_cone.size() < right_cone.size() ? left_cone.size() : right_cone.size();
	for (i = 0; i < size; i++)
	{
		p.x = (left_cone[i].x + right_cone[i].x) / 2;
		p.y = (left_cone[i].y + right_cone[i].y) / 2;
		center_cone.push_back(p);
	}
	if (left_cone.size() > 0)ray(store.image_mat, left_cone[0], -1.884, 255);
	if (right_cone.size() > 0)ray(store.image_mat, right_cone[0], -1.256, 255);
	if (left_cone.size() > 0 && right_cone.size() > 0)
	{
		p.x = (left_cone.back().x + right_cone.back().x) / 2;
		p.y = (left_cone.back().y + right_cone.back().y) / 2;
		center_cone.push_back(p);
	}
	if (size > 1)
	{
		for (i = 0; i < size - 1; i++)
		{
			line(store.image_mat, left_cone[i], left_cone[i + 1], 255);
			line(store.image_mat, right_cone[i], right_cone[i + 1], 255);
		}
	}
	if (center_cone.size() > 1)last_center_in_cone = center_cone[1].x;
}

void MainImage::mend_farm_inside() {
	Point p;
	int i;
	int size = left_cone.size() < right_cone.size() ? left_cone.size() : right_cone.size();
	if (left_cone.size() > 0 && right_cone.size() > 0)
	{
		for (i = 0; i < size; i++)
		{
			p.x = (left_cone[i].x + right_cone[i].x) / 2;
			p.y = (left_cone[i].y + right_cone[i].y) / 2;
			center_cone.push_back(p);
		}
	}
	if (left_cone.size() > 0)ray(store.image_mat, left_cone[0], -1.884, 255);
	if (right_cone.size() > 0)ray(store.image_mat, right_cone[0], -1.256, 255);
	if (left_cone.size() > 1)
	{
		for (i = 0; i < left_cone.size() - 1; i++)line(store.image_mat, left_cone[i], left_cone[i + 1], 255);
	}
	if (right_cone.size() > 1)
	{
		for (i = 0; i < right_cone.size() - 1; i++)line(store.image_mat, right_cone[i], right_cone[i + 1], 255);
	}
	if (center_cone.size() > 1)last_center_in_cone = center_cone[1].x;
}

void MainImage::mend_farm_out_find() {
	if (center_cone.size() > 1)
	{
		float x1 = center_cone.back().x;
		float x2 = center_cone[center_cone.size() - 2].x;
		float y1 = center_cone.back().y;
		float y2 = center_cone[center_cone.size() - 2].y;
		float x = center_cone.back().x;
		int m = center_cone.back().y;
		const uchar* r = store.image_mat.ptr<uchar>(m);
		while (r[(int)x] == 0)
		{
			x -= (x2 - x1) / (y2 - y1);
			if (x < 0)x = 0;
			else if (x > IMGW - 1)x = IMGW - 1;
			m--;
			r = store.image_mat.ptr<uchar>(m);
		}
		Point p;
		p.x = x;
		p.y = m;
		center_cone.push_back(p);
		cout << p.x << " " << p.y << endl;
	}
	else if (center_cone.size() == 1 && center_cone[0].y < IMGH - 5)
	{
		float x1 = center_cone[0].x;
		float x2 = IMGW / 2;
		float y1 = center_cone[0].y;
		float y2 = IMGH - 5;
		float x = center_cone[0].x;
		int m = center_cone[0].y;
		const uchar* r = store.image_mat.ptr<uchar>(m);
		while (r[(int)x] == 0)
		{
			x -= (x2 - x1) / (y2 - y1);
			if (x < 0)x = 0;
			else if (x > IMGW - 1)x = IMGW - 1;
			m--;
			r = store.image_mat.ptr<uchar>(m);
		}
		Point p;
		p.x = x;
		p.y = m;
		center_cone.push_back(p);
		cout << p.x << " " << p.y << endl;
	}
}

void MainImage::mend_in_hump_on() {
	int i;
	int right = IMGW - 1, left = 0, center = IMGW / 2;
	int r_up_y = IMGH - re.hump.mend_up_line;
	bool find_up = false;
	Point down, up;
	down.x = IMGW / 2; down.y = IMGH - 1;
	while (!find_up)
	{
		uchar* r_up = store.image_mat.ptr<uchar>(r_up_y);
		r_up_y++;
		for (i = 1; i < IMGW; i++)
		{
			if (r_up[i] != 0 && r_up[i - 1] == 0)
			{
				int k = 0;
				while (r_up[i + k] != 0)k++;
				if (k <= 10)continue;
				else if (k < 80) find_up = true;
				break;
			}
		}
		if (IMGH - 1 - r_up_y < 40)find_up = true;
	}
	uchar* r_up = store.image_mat.ptr<uchar>(r_up_y - 10);
	for (i = IMGW / 2; i > 0; i--)
	{
		if (r_up[i + 1] != 0 && r_up[i] != 0 && r_up[i - 1] == 0)left = i;
	}
	for (i = IMGW / 2; i < IMGW - 1; i++)
	{
		if (r_up[i - 1] != 0 && r_up[i] != 0 && r_up[i + 1] == 0)right = i;
	}
	center = (left + right) / 2;
	up.x = center; up.y = r_up_y - 10;
	float x1 = down.x;
	float y1 = down.y;
	float x2 = up.x;
	float y2 = up.y;
	for (i = down.y; i > up.y; i--)
	{
		int m = x1 + (i - down.y) * (x2 - x1) / (y2 - y1);
		center_point[i] = m;
		exist_left_edge_point[i] = true;
		exist_right_edge_point[i] = true;
	}
}



void MainImage::find_edge_point()
{
	Mat mat = store.image_mat;
	int i, j, center = last_center;
	int last_right = IMGW - 1;
	int last_left = 0;
	int count = 0;
	int m = 1;
	uchar* r = mat.ptr<uchar>(IMGH - 1);           //指向最下面一行
	for (j = last_center + 1; j < IMGW; j++) {                 //中线向右找寻右边界
		if (j == IMGW - 1) {
			last_right = IMGW - 1;
		}
		if (r[j - 1] != 0 && r[j] == 0 && r[j + 1] == 0) {
			last_right = j;
			break;
		}
	}
	for (j = last_center - 1; j > -1; j--) {                   //中线向左找寻左边界
		if (j == 0) {
			last_left = 0;
		}
		if (r[j + 1] != 0 && r[j] == 0 && r[j - 1] == 0) {
			last_left = j;
			break;
		}
	}
	for (i = IMGH - 1; i > center_lost; i--) {                     //从下向上开始巡线
		uchar* this_row = mat.ptr(i);                  //指向第i行
		if (this_row[center] == 0) {
			if (count_white(mat, i) <= 8)  //第i行白像素数量不大于8，则第i行中心位置缺失
			{
				exist_left_edge_point[i] = false;
				exist_right_edge_point[i] = false;
				center_lost = i;
				break;
			}
			while (true)
			{
				//将中心点挪到白色位置
				if (this_row[(center + m < IMGW ? center + m : IMGW - 1)] != 0)
				{
					center = (center + m < IMGW ? center + m : IMGW - 1);
					m = 1;
					break;
				}
				if (this_row[center - m >= 0 ? center - m : 0] != 0)
				{
					center = center - m >= 0 ? center - m : 0;
					m = 1;
					break;
				}
				m++;
			}
		}
		for (j = center + 1; j < IMGW; j++) {                                    //找寻右边界 
			count++;
			if (j == IMGW - 1) {
				exist_right_edge_point[i] = false;
				break;
			}
			if (this_row[j - 1] != 0 && this_row[j] == 0) {
				if (j < last_right + 2)
				{//j <= last_right + 2&&(abs(j-last_right)<20    abs(j-last_right) <=  5
					right_edge_point[i] = j;
					exist_right_edge_point[i] = true;
					last_right = j;
				}
				else {
					exist_right_edge_point[i] = false;
				}
				break;
			}
		}
		for (j = center - 1; j > -1; j--) {                                            //找寻左边界
			count++;
			if (j == 0) {
				exist_left_edge_point[i] = false;
				break;
			}
			if (this_row[j + 1] != 0 && this_row[j] == 0) {
				if (j > last_left - 2)
				{  //j >= last_left - 2abs(j - last_left) <= 5
					left_edge_point[i] = j;
					exist_left_edge_point[i] = true;
					last_left = j;
				}
				else {
					exist_left_edge_point[i] = false;
				}
				break;
			}

		}
		if (last_left > last_right) {
			center_lost = i;
			exist_left_edge_point[i] = false;
			exist_right_edge_point[i] = false;
			break;
		}
		if (count < 3 * MINX) {    //3 * MINX
			center_lost = i;
			exist_left_edge_point[i] = false;
			exist_right_edge_point[i] = false;
			break;
		}
		if ((exist_left_edge_point[i] && left_edge_point[i] > IMGW - 8) || (exist_right_edge_point[i] && right_edge_point[i] < 8)) {
			center_lost = i;
			exist_left_edge_point[i] = false;
			exist_right_edge_point[i] = false;
			break;
		}
		if (exist_left_edge_point[i]) {
			if (count_white(mat, i,
				(left_edge_point[i] + 5 < IMGW ? left_edge_point[i] + 5 : IMGW), (left_edge_point[i] - 4 > -1 ? left_edge_point[i] - 4 : 0)) == 0) {
				center_lost = i;
				exist_left_edge_point[i] = false;
				exist_right_edge_point[i] = false;
				break;
			}
		}
		if (exist_right_edge_point[i]) {
			if (count_white(mat, i,
				(right_edge_point[i] + 5 < IMGW ? right_edge_point[i] + 5 : IMGW), (right_edge_point[i] - 4 > -1 ? right_edge_point[i] - 4 : 0)) == 0) {
				center_lost = i;
				exist_left_edge_point[i] = false;
				exist_right_edge_point[i] = false;
				break;
			}
		}
		count = 0;
		center = (last_left + last_right) / 2;
		center_point[i] = center;
	}
	if (exist_left_edge_point[IMGH - MINY] && exist_right_edge_point[IMGH - MINY])
		last_center = (left_edge_point[IMGH - MINY] + right_edge_point[IMGH - MINY]) >> 1;
	else if (exist_left_edge_point[IMGH - MINY])
		last_center = (left_edge_point[IMGH - MINY] + IMGW - 1) >> 1;
	else if (exist_right_edge_point[IMGH - MINY])
		last_center = right_edge_point[IMGH - MINY] >> 1;
	for (i = 0; i < 3; i++)
	{
		exist_left_edge_point[i] = false;
		exist_right_edge_point[i] = false;
	}
	exist_left_edge_point[IMGH - 1] = false;
	exist_right_edge_point[IMGH - 1] = false;
	exist_left_edge_point[IMGH - 2] = false;
	exist_right_edge_point[IMGH - 2] = false;
}

void MainImage::refind_edge_point()
{
	Mat mat = store.image_mat;
	int i, j, center = IMGW / 2;
	int last_right = IMGW - 1;
	int last_left = 0;
	int count = 0;
	int m = 1;
	for (i = IMGH - 1; i > 0; i--) {
		exist_left_edge_point[i] = false;
		exist_right_edge_point[i] = false;
	}
	for (i = IMGH - 1; i > center_lost; i--)
	{                     //从下向上开始巡线
		uchar* this_row = mat.ptr(i);                  //指向第i行
		if (this_row[center] == 0) {
			if (count_white(store.image_mat, i) < 5) {
				exist_left_edge_point[i] = false;
				exist_right_edge_point[i] = false;
				break;
			}
			while (true) {
				if (this_row[(center + m < IMGW ? center + m : IMGW - 1)] != 0) {
					center = (center + m < IMGW ? center + m : IMGW - 1);
					m = 1;
					break;
				}
				if (this_row[center - m >= 0 ? center - m : 0] != 0) {
					center = center - m >= 0 ? center - m : 0;
					m = 1;
					break;
				}
				m++;
			}
		}
		if (this_row[center] != 0)
		{
			for (j = center + 1; j < IMGW - 1; j++) {                                    //找寻右边界 
				count++;
				if (this_row[j - 1] != 0 && this_row[j] == 0) {
					right_edge_point[i] = j;
					exist_right_edge_point[i] = true;
					last_right = j;
					break;
				}
				if (j == IMGW - 1) {
					exist_right_edge_point[i] = false;
				}
			}
			for (j = center - 1; j > 0; j--) {                                            //找寻左边界
				count++;
				if (this_row[j + 1] != 0 && this_row[j] == 0) {
					left_edge_point[i] = j;
					exist_left_edge_point[i] = true;
					last_left = j;
					break;
				}
				if (j == 1)
				{
					exist_left_edge_point[i] = false;
				}
			}
		}
		center = (last_left + last_right) / 2;
		if (last_left > last_right) {
			center_lost = i;
			break;
		}
		if (count < 3 * MINX) {    //3 * MINX
			center_lost = i;
			break;
		}
		if ((exist_left_edge_point[i] && left_edge_point[i] > IMGW - 8) || (exist_right_edge_point[i] && right_edge_point[i] < 8)) {
			center_lost = i;
			break;
		}
		if (exist_left_edge_point[i]) {
			if (count_white(mat, i,
				(left_edge_point[i] + 5 < IMGW ? left_edge_point[i] + 5 : IMGW), (left_edge_point[i] - 4 > -1 ? left_edge_point[i] - 4 : 0)) == 0) {
				center_lost = i;
				break;
			}
		}
		if (exist_right_edge_point[i]) {
			if (count_white(mat, i,
				(right_edge_point[i] + 5 < IMGW ? right_edge_point[i] + 5 : IMGW), (right_edge_point[i] - 4 > -1 ? right_edge_point[i] - 4 : 0)) == 0) {
				center_lost = i;
				break;
			}
		}
		count = 0;
	}
}

void MainImage::find_center()
{
	for (int i = IMGH - 1; i >= 0; i--) {
		if (i < center_lost)break;
		if (lost_left || lost_right) {
			if (exist_left_edge_point[i] && exist_right_edge_point[i])
				center_point[i] = (left_edge_point[i] + right_edge_point[i]) >> 1;
			else if (exist_right_edge_point[i]) {
				int j;
				for (j = i; j >= 0; j--) {
					if (!exist_right_edge_point[j] || exist_left_edge_point[j]) break;
				}
				int mov, devide;
				float wid;
				if (i > IMGH - 5 && right_edge_point[i] < 140) {
					wid = 125;//(2 * right_edge_point[i] - 100)/2;110
				}
				else wid = right_edge_point[i];

				for (int k = i; k > j; k--) {
					wid -= 0.28f;
					//devide = right_edge_point[k] / 2;
					mov = (2 * right_edge_point[k] - round(wid)) / 2;
					center_point[k] = ((2 * mov) / 2 > -1 ? (2 * mov) / 2 : 0);
				}
				i = j + 1;
			}
			else if (exist_left_edge_point[i]) {
				int j;
				for (j = i; j >= 0; j--) {
					if (!exist_left_edge_point[j] || exist_right_edge_point[j]) break;
				}
				int mov, devide;
				float wid;
				if (i > IMGH - 5 && IMGW - left_edge_point[i] < 140) {
					wid = 125;//IMGW - left_edge_point[i];110
				}
				else wid = IMGW - left_edge_point[i];
				for (int k = i; k > j; k--) {
					wid -= 0.28;//0.43
					//devide = (left_edge_point[k] + IMGW) / 2;
					mov = (2 * left_edge_point[k] + round(wid)) / 2;
					center_point[k] = ((2 * mov) / 2 < IMGW ? (2 * mov) / 2 : IMGW - 1);
				}
				i = j + 1;
			}
			else center_point[i] = IMGW / 2;
		}
		else {
			if (exist_left_edge_point[i] && exist_right_edge_point[i])
				center_point[i] = (left_edge_point[i] + right_edge_point[i]) / 2;
			else if (exist_right_edge_point[i]) {
				center_point[i] = right_edge_point[i] / 2;
			}
			else if (exist_left_edge_point[i]) {
				center_point[i] = (left_edge_point[i] + IMGW - 1) / 2;
			}
			else center_point[i] = IMGW / 2;
		    /*else if(exist_right_edge_point[i]) {
				center_point[i] =((2*center_point[i-1]-center_point[i-2])+right_edge_point[i] / 2)/2;
			}
			else if (exist_left_edge_point[i]) {
				center_point[i] =(2*center_point[i-1]-center_point[i-2]+_(left_edge_point[i] + IMGW - 1) / 2)/2;
			}
			else center_point[i] = (center_point[i-1]+IMGW / 2)/2;
			*/
		}
	}
	if (center_lost != -1) {
		if (center_lost < IMGH - 1) {
			for (int i = center_lost; i > -1; i--) {
				center_point[i] = center_point[center_lost + 1];
			}
		}
		else {
			for (int i = center_lost; i > -1; i--) {
				center_point[i] = IMGW / 2;
			}
		}
	}
}

void MainImage::find_end_point() {
	Mat mat = store.image_mat;
	Point p;
	int i;
	left_end_point.clear();
	right_end_point.clear();
	for (i = IMGH - 2; i > 0; i--) {
		if (count_white(mat, i) < 1) break;
		if (exist_left_edge_point[i]) {
			if ((!exist_left_edge_point[i - 1] && exist_left_edge_point[i + 1]) ||
				(exist_left_edge_point[i - 1] && !exist_left_edge_point[i + 1])) {
				p.y = i;
				p.x = left_edge_point[i];
				left_end_point.push_back(p);
			}
		}

		if (exist_right_edge_point[i]) {

			if ((!exist_right_edge_point[i - 1] && exist_right_edge_point[i + 1]) ||
				(exist_right_edge_point[i - 1] && !exist_right_edge_point[i + 1])) {
				p.y = i;
				p.x = right_edge_point[i];
				right_end_point.push_back(p);
			}
		}
	}
}

void MainImage::judge_lost()
{
	if (left_end_point.size() != 0 && (left_end_point[0].y > IMGH - 10 || left_end_point[0].x < 8) && left_end_point[1].y > 20 && left_end_point[1].y < IMGH - 20 && left_end_point[1].x > IMGW / 2 + 5) {
		if ((right_end_point.size() == 0) ||
			(right_end_point.size() != 0 && right_end_point[0].y < IMGH - 50) ||
			(right_end_point.size() != 0 && right_end_point[0].y > IMGH - 20 && right_end_point[1].y > IMGH - 40 && right_end_point[0].x > IMGW - 20)) {
			lost_right = true;
		}
	}
	if (right_end_point.size() != 0 && (right_end_point[0].y > IMGH - 10 || right_end_point[0].x > IMGW - 8) && right_end_point[1].y > 20 && right_end_point[1].y < IMGH - 20 && right_end_point[1].x < IMGW / 2 - 5) {
		if (left_end_point.size() == 0 ||
			(left_end_point.size() != 0 && left_end_point[0].y < IMGH - 50) ||
			(left_end_point.size() != 0 && left_end_point[0].y > IMGH - 20 && left_end_point[1].y > IMGH - 40 && left_end_point[0].x < 20)) {
			lost_left = true;
		}
	}
}

void MainImage::count_branch()
{
	Mat mat = store.image_mat;
	int i, j, k;
	int count = 0, white = 0;
	if (left_end_point.size() > 2) {
		for (i = 1; i < left_end_point.size() - 1; i += 2) {
			if (abs(left_end_point[i].y - left_end_point[i + 1].y) > 5 && left_end_point[i].x > 3) {
				for (j = left_end_point[i].y - 1; j > left_end_point[i + 1].y; j--) {
					uchar* r = mat.ptr<uchar>(j);
					for (k = left_end_point[i].x; k > -1; k--) {
						if (r[k] != 0)white++;
						else break;
					}
					if (white > 4)count++;
					white = 0;
				}
				if (count >= abs(left_end_point[i].y - left_end_point[i + 1].y) - 3)left_branch_num++;
				count = 0;
			}
		}
	}
	if (right_end_point.size() > 2) {
		for (i = 1; i < right_end_point.size() - 1; i += 2) {
			if (abs(right_end_point[i].y - right_end_point[i + 1].y) > 5 && right_end_point[i].x < IMGW - 3) {
				for (j = right_end_point[i].y - 1; j > right_end_point[i + 1].y; j--) {
					uchar* r = mat.ptr<uchar>(j);
					for (k = right_end_point[i].x; k < IMGW; k++) {
						if (r[k] != 0)white++;
						else break;
					}
					if (white > 4)count++;
					white = 0;
				}
				if (count >= abs(right_end_point[i].y - right_end_point[i + 1].y) - 3)right_branch_num++;
				count = 0;
			}
		}
	}
}

void MainImage::judge_continuity()
{
	int i, l_long = 0, r_long = 0;
	for (i = 0; i < left_end_point.size(); i += 2) {
		if (abs(left_end_point[i].y - left_end_point[i + 1].y) > l_long)
			l_long = abs(left_end_point[i].y - left_end_point[i + 1].y);
	}
	for (i = 0; i < right_end_point.size(); i += 2) {
		if (abs(right_end_point[i].y - right_end_point[i + 1].y) > r_long)
			r_long = abs(right_end_point[i].y - right_end_point[i + 1].y);
	}
	if ((left_end_point.size() > 2 && l_long > 0.6 * IMGH) || (left_end_point.size() == 2 && abs(left_end_point[0].y - left_end_point[1].y) > 0.3 * IMGH))
		left_continue = true;
	else left_continue = false;
	if ((right_end_point.size() > 2 && r_long > 0.6 * IMGH) || (right_end_point.size() == 2 && abs(right_end_point[0].y - right_end_point[1].y) > 0.3 * IMGH))
		right_continue = true;
	else right_continue = false;
}

void MainImage::edge_filter(int wid, int side)
{
	int i;
	for (i = IMGH - 2; i > 0; i--) {
		if (side == 0 || side == 2)
		{
			if (exist_left_edge_point[i] && exist_left_edge_point[i + 1]) {
				if (abs(left_edge_point[i] - left_edge_point[i + 1]) >= wid) {
					exist_left_edge_point[i + 1] = false;
				}
			}
		}
		if (side == 1 || side == 2)
			if (exist_right_edge_point[i] && exist_right_edge_point[i + 1]) {
				if (abs(right_edge_point[i] - right_edge_point[i + 1]) >= wid) {
					exist_right_edge_point[i + 1] = false;
				}
			}
	}
}

void MainImage::end_filter(int side) {
	int i, j;
	if (side == 0 || side == 2)
	{
		if (left_end_point.size() > 0) {
			for (i = 0; i < left_end_point.size(); i += 2) {
				if (abs(left_end_point[i + 1].y - left_end_point[i].y) < 3) {
					for (j = left_end_point[i + 1].y; j <= left_end_point[i].y; j++) {
						exist_left_edge_point[j] = false;
					}
					left_end_point.erase(left_end_point.begin() + i, left_end_point.begin() + i + 2);
				}
			}
		}
	}
	if (side == 1 || side == 2)
	{
		if (right_end_point.size() > 0) {
			for (i = 0; i < right_end_point.size(); i += 2) {
				if (abs(right_end_point[i + 1].y - right_end_point[i].y) < 3) {
					for (j = right_end_point[i + 1].y; j <= right_end_point[i].y; j++) {
						exist_right_edge_point[j] = false;
					}
					right_end_point.erase(right_end_point.begin() + i, right_end_point.begin() + i + 2);
				}
			}
		}
	}

}
/**
 * @brief 保存图像
*/

void MainImage::show(float dev, float angle_result, float speed, int current_speed, bool c, bool l, bool r, bool l_e, bool r_e, bool l_c, bool r_c, bool c_c)
{
    Mat channels[3];
    store.image_mat += 128;
    store.image_mat.copyTo(channels[0]);
    store.image_mat.copyTo(channels[1]);
    store.image_mat.copyTo(channels[2]);

    // 在图像上添加边缘、端点和圆锥标记
    merge(channels, 3, store.image_show);

    // 在图像上添加边缘、端点和圆锥标记
    for (int i = 0; i < IMGH; i++) {
        if (exist_left_edge_point[i] || exist_right_edge_point[i])
        {
            if (exist_left_edge_point[i]) store.image_show.at<Vec3b>(i, left_edge_point[i]) = Vec3b(255, 0, 0);
            if (exist_right_edge_point[i]) store.image_show.at<Vec3b>(i, right_edge_point[i]) = Vec3b(0, 255, 0);
            store.image_show.at<Vec3b>(i, center_point[i]) = Vec3b(0, 255, 255);
        }
    }
    if (l_e) for (int i = 0; i < left_end_point.size(); i++) store.image_show.at<Vec3b>(left_end_point[i].y, left_end_point[i].x) = Vec3b(0, 0, 255);
    if (r_e) for (int i = 0; i < right_end_point.size(); i++) store.image_show.at<Vec3b>(right_end_point[i].y, right_end_point[i].x) = Vec3b(255, 0, 255);
    if (l_c) for (int i = 0; i < left_cone.size(); i++) store.image_show.at<Vec3b>(left_cone[i].y, left_cone[i].x) = Vec3b(19, 78, 39);
    if (r_c) for (int i = 0; i < right_cone.size(); i++) store.image_show.at<Vec3b>(right_cone[i].y, right_cone[i].x) = Vec3b(121, 77, 166);
    if (c_c) for (int i = 0; i < center_cone.size(); i++) store.image_show.at<Vec3b>(center_cone[i].y, center_cone[i].x) = Vec3b(0, 153, 255);

	store.image_show.at<Vec3b>(MainImage::deviation_thresh, 80) = Vec3b(0, 0, 255);
	store.image_show.at<Vec3b>((MainImage::deviation_thresh - re.main.up_scope), 80) = Vec3b(0, 255, 0);
	store.image_show.at<Vec3b>((MainImage::deviation_thresh + re.main.down_scope),80) = Vec3b(255, 0, 0);

    // 在图像顶部添加变量文本
    int fontFace = FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.25;
    int thickness = 1;
    int baseline = 0;
	int interval = 5;//行间距
    Size textSize = getTextSize("Text", fontFace, fontScale, thickness, &baseline);
    baseline += thickness;

    // 文本位置的起始点
    Point textOrg(2, textSize.height + 5);
	// 主文本颜色
	cv::Scalar mainColor = Scalar(60,20,220,0.75);
    // 绘制每个变量

    putText(store.image_show, "state: " + 
		string(((state_out == straight) ? "straight" :
				(state_out == right_circle_in_find) ? "right_circle_in_find" :
				(state_out == right_circle_in_strai) ? "right_circle_in_strai" :
				(state_out == right_circle_in_circle) ? "right_circle_in_circle" :
				(state_out == right_circle_inside_before) ? "right_circle_inside_before" :
				(state_out == right_circle_inside) ? "right_circle_inside" :
				(state_out == right_circle_out) ? "right_circle_out" :
				(state_out == right_circle_out_find) ? "right_circle_out_find" :
				(state_out == right_circle_out_strai) ? "right_circle_out_strai" :
				(state_out == right_circle_out_out) ? "right_circle_out_out" :
				(state_out == left_circle_in_find) ? "left_circle_in_find" :
				(state_out == left_circle_in_strai) ? "left_circle_in_strai" :
				(state_out == left_circle_in_circle) ? "left_circle_in_circle" :
				(state_out == left_circle_inside_before) ? "left_circle_inside_before" :
				(state_out == left_circle_inside) ? "left_circle_inside" :
				(state_out == left_circle_out) ? "left_circle_out" :
				(state_out == left_circle_out_find) ? "left_circle_out_find" :
				(state_out == left_circle_out_strai) ? "left_circle_out_strai" :
	 			"other")),
				textOrg, fontFace, fontScale, mainColor, thickness, 8);
	textOrg.y += textSize.height + interval;

    putText(store.image_show, "kp: " + to_string(cur_kp), textOrg, fontFace, fontScale, mainColor, thickness, 8);
    textOrg.y += textSize.height + interval;

    putText(store.image_show, "Angle: " + to_string(angle_result), textOrg, fontFace, fontScale, mainColor, thickness, 8);
    textOrg.y += textSize.height + interval;

    putText(store.image_show, "speed: " + to_string(speed), textOrg, fontFace, fontScale, mainColor, thickness, 8);
    textOrg.y += textSize.height + interval;

    putText(store.image_show, "current_speed: " + to_string(current_speed), textOrg, fontFace, fontScale, mainColor, thickness, 8);
	textOrg.y += textSize.height + interval;

    putText(store.image_show, "deviation: " + to_string(deviation), textOrg, fontFace, fontScale, mainColor, thickness, 8);
	textOrg.y += textSize.height + interval;

    putText(store.image_show, "end point size: " + to_string(right_end_point.size()), textOrg, fontFace, fontScale, mainColor, thickness, 8);

    resize(store.image_show, store.image_show, cv::Size(IMGW * 2, IMGH * 2));
    if (re.set.color)store.wri << store.image_BGR;
    else store.wri << store.image_show;
}

void MainImage::count_cone(int begin) {
	left_cone.clear(); right_cone.clear(); center_cone.clear();
	Point p;
	int i;
	int center;
	int l_wid, l_wid_last = 0;
	int r_wid, r_wid_last = 0;
	bool l_rise = true;
	bool r_rise = true;
	center = last_center_in_cone > 0 ? last_center_in_cone : IMGW / 2;
	for (i = begin; i > 10; i--)
	{
		const uchar* r = store.image_mat.ptr<uchar>(i);
		for (int j = center - 1; j > 0; j--)
		{
			if (r[j] != 0 && r[j + 1] == 0)
			{
				int k = j - 1;
				while ((r[k] != 0) && k > 0)k--;
				if (begin < IMGH - 5 && k == 0)break;
				l_wid = j - k;
				if (l_wid < 6 || l_wid>20)break;
				if (l_wid <= l_wid_last && l_rise)
				{
					l_rise = false;
					p.x = (j + k) / 2;
					p.y = i;
					if (left_cone.size() > 0)
					{
						if (left_cone.back().y - p.y > 55 || left_cone.back().y - p.y < i / 5)break;
					}
					else
					{
						if (begin - p.y > 50)break;
					}
					bool h = true;
					for (int m = i; m > i - 3; m--) {
						if (store.image_mat.ptr<uchar>(m)[p.x] == 0)h = false;
					}
					if (!h)break;
					if (p.x < center)left_cone.push_back(p);

				}
				else if (l_wid > l_wid_last)
				{
					l_rise = true;
				}
				l_wid_last = l_wid;
				break;
			}
		}
		for (int j = center + 1; j < IMGW; j++)
		{
			if (r[j] != 0 && r[j - 1] == 0)
			{
				int k = j + 1;
				while ((r[k] != 0) && k < IMGW)k++;
				if (begin < IMGH - 5 && k == IMGW - 1)break;
				r_wid = k - j;
				if (r_wid < 6 || r_wid>20)break;
				if (r_wid <= r_wid_last && r_rise)
				{
					r_rise = false;
					p.x = (j + k) / 2;
					p.y = i;
					if (right_cone.size() > 0)
					{
						if (right_cone.back().y - p.y > 55 || right_cone.back().y - p.y < i / 5)break;
					}
					else
					{
						if (begin - p.y > 50)break;
					}
					bool h = true;
					for (int m = i; m > i - 3; m--) {
						if (store.image_mat.ptr<uchar>(m)[p.x] == 0)h = false;
					}
					if (!h)break;
					if (p.x > center)right_cone.push_back(p);
				}
				else if (r_wid > r_wid_last)
				{
					r_rise = true;
				}
				r_wid_last = r_wid;
				break;
			}
		}
		if (left_cone.size() > 0 && right_cone.size() > 0)center = (left_cone.back().x + right_cone.back().x) / 2;
		else if (left_cone.size() > 0)
		{
			if (begin >= IMGH - 5)center = (left_cone.back().x + IMGW - 1) / 2;
		}
		else if (right_cone.size() > 0)
		{
			if (begin >= IMGH - 5)center = right_cone.back().x / 2;
		}

	}

	Point p0;
	int l_num = left_cone.size();
	int r_num = right_cone.size();
	if (l_num == 0) {
		l_num = 1; p0.x = (IMGH - begin) * 0.75; p0.y = begin; left_cone.push_back(p0);
	}
	if (r_num == 0) {
		r_num = 1; p0.x = IMGW - (IMGH - begin) * 0.75; p0.y = begin; right_cone.push_back(p0);
	}
	int num = l_num > r_num ? l_num : r_num;
	for (int i = 0; i < num; i++)
	{
		if (l_num > i && r_num > i)
		{
			p.x = (left_cone[i].x + right_cone[i].x) / 2;
			p.y = (left_cone[i].y + right_cone[i].y) / 2;
			center_cone.push_back(p);
		}
		else if (l_num > i)
		{
			p.x = (left_cone[i].x + right_cone.back().x) / 2;
			p.y = (left_cone[i].y + right_cone.back().y) / 2;
			center_cone.push_back(p);
		}
		else if (r_num > i)
		{
			p.x = (left_cone.back().x + right_cone[i].x) / 2;
			p.y = (left_cone.back().y + right_cone[i].y) / 2;
			center_cone.push_back(p);
		}
	}
	if (center_cone.size() > 1)last_center_in_cone = center_cone[1].x;
}

void MainImage::find_center_in_farm(int begin) {
	if (center_cone.size() > 0)
	{
		for (int i = center_cone.size() - 1; i > 0; i--)
		{
			float x1 = center_cone[i].x;
			float x2 = center_cone[i - 1].x;
			float y1 = center_cone[i].y;
			float y2 = center_cone[i - 1].y;
			for (int j = center_cone[i].y; j <= center_cone[i - 1].y; j++)
			{
				center_point[j] = x1 + (j - y1) * (x2 - x1) / (y2 - y1);
				exist_left_edge_point[j] = true;
				exist_right_edge_point[j] = true;
			}
		}
		for (int j = center_cone[0].y; j <= begin; j++)
		{
			float x = center_cone[0].x;
			int bottom;
			if (begin < IMGH - 5)bottom = begin + 8 < IMGH - 5 ? center_point[begin + 8] : IMGW / 2;
			else bottom = IMGW / 2;
			center_point[j] = x + (float)(j - center_cone[0].y) * (bottom - (float)center_cone[0].x) / (center_lost - (float)center_cone[0].y);
			exist_left_edge_point[j] = true;
			exist_right_edge_point[j] = true;
		}
	}
}

void MainImage::refind_edge_in_farm_out(Point start) {
	int i, j;
	int last_right = IMGW - 1;
	int last_left = 0;
	center_lost = 0;
	int center = start.x;
	for (i = start.y; i > center_lost; i--)
	{
		uchar* r = store.image_mat.ptr(i);
		if (r[center] == 0) { center_lost = i; break; }
		for (j = center + 1; j < IMGW - 1; j++) {                                    //找寻右边界 
			if (r[j - 1] != 0 && r[j] == 0) {
				right_edge_point[i] = j;
				exist_right_edge_point[i] = true;
				last_right = j;
				break;
			}
			if (j == IMGW - 1) {
				exist_right_edge_point[i] = false;
			}
		}
		for (j = center - 1; j > 0; j--) {                                            //找寻左边界
			if (r[j + 1] != 0 && r[j] == 0) {
				left_edge_point[i] = j;
				exist_left_edge_point[i] = true;
				last_left = j;
				break;
			}
			if (j == 1)
			{
				exist_left_edge_point[i] = false;
			}
		}
		center = (last_left + last_right) / 2;
		center_point[i] = center;
	}
}

void MainImage::produce_dv(int error) {
	int i;
	int dv = 0;
	float delta = 0;
	if (abs(error) < 6) {
		for (i = IMGH - 40; i > 30; i--) {
			if (abs(center_point[i] - IMGW / 2) < 6) {
				delta += 1;
			}
			else {
				break;
			}
		}
		dv += round(delta);
	}
	return;
}



bool j_right_circle_in_circle(const MainImage& mi, uchar& state_in)
{
	/*if (mi.right_end_point[0].y < IMGH - 5) return false;
	if (mi.right_end_point.size() == 4 && mi.right_end_point[1].y > IMGH - 30) {
		//state_in = right_circle_in_circle;
	return true;
	}*/
	int i, max = IMGW - 1;
	if (mi.right_end_point.size() > 2)
	{
		for (i = mi.right_end_point[0].y; i > mi.right_end_point[1].y; i--)
		{
			if (mi.right_edge_point[i] < max)
			{
				max = mi.right_edge_point[i];
			}
		}
		if ((max != IMGW - 1 && max > IMGW - 30) || mi.right_end_point[1].y > IMGH - 30)return true;
	}
	else return true;
	return false;
}

bool j_right_circle_inside_before(const MainImage& mi, uchar& state_in)
{
	if (mi.left_end_point.size() > 0 && mi.left_end_point[1].x > IMGW - 40) {
		state_in = right_circle_inside_before;
		return true;
	}
	return false;
}

bool j_right_circle_inside(const MainImage& mi, uchar& state_in)
{
	if ((mi.right_end_point.size() == 0 && mi.left_end_point.size() == 2 && mi.left_end_point[1].x > IMGW / 2 + 20 &&
		(mi.left_end_point[0].y > IMGH - 5 || mi.left_end_point[0].x < 5)) ||
		(mi.right_end_point.size() == 2 && mi.right_end_point[1].y > IMGH / 2 + 10 && mi.left_end_point.size() == 2 && mi.left_end_point[1].x > IMGW / 2 + 20 &&
			(mi.left_end_point[0].y > IMGH - 5 || mi.left_end_point[0].x < 5)))
	{
		return true;
	}
	return false;
}

void j_right_circle_out_find(const MainImage& mi, uchar& state_in)
{
	int i, count = 0;
	const uchar* r = mi.store.image_mat.ptr<uchar>(mi.left_end_point[1].y - 1);
	for (i = mi.left_end_point[1].x - 3; i < mi.left_end_point[1].x + 4; i++) {
		if (r[i] != 0)count++;
	}
	if (mi.left_end_point.size() == 2 && (mi.left_end_point[0].y > IMGH - MINY || mi.left_end_point[0].x < MINX) && count > 2) {
		state_in = right_circle_out_find;
	}
}

//Re
void j_right_circle_out_strai(const MainImage& mi, uchar& state_in)
{
	int count = 0;
	//const uchar* r = mi.store.image_mat.ptr<uchar>(IMGH - mi.re.r_circle.out_strai_find_pos);
	const uchar* r = mi.store.image_mat.ptr<uchar>(mi.left_end_point[1].y - 20);
	for (int i = 0; i < IMGW; i++) {
		if (r[i] != 0)count++;
		else break;
	}
	if (count > IMGW - 10) {
		state_in = right_circle_out_strai;
	}
}

void j_right_circle_out_out(const MainImage& mi, uchar& state_in)
{
	if (mi.left_end_point.size() != 0 && abs(mi.left_end_point[0].y - mi.left_end_point[1].y) > 30 &&
		mi.right_end_point.size() != 0 && mi.right_end_point[0].y < IMGH - 30) {
		state_in = right_circle_out_out;
	}
}

bool j_left_circle_in_circle(const MainImage& mi, uchar& state_in) {
	/*if (mi.left_end_point[0].y < IMGH - 5) return false;
	if (mi.left_end_point.size() == 4 && mi.left_end_point[1].y > IMGH - 30) {
		//state_in = left_circle_in_circle;
	return true;
	}*/
	int i, max = 0;
	if (mi.left_end_point.size() > 2)
	{
		for (i = mi.left_end_point[0].y; i > mi.left_end_point[1].y; i--)
		{
			if (mi.left_edge_point[i] > max)
			{
				max = mi.left_edge_point[i];
			}
		}
		if ((max != 0 && max < 30) || mi.left_end_point[1].y > IMGH - 30)return true;
	}
	else return true;
	return false;
}

bool j_left_circle_inside_before(const MainImage& mi, uchar& state_in) {
	if (mi.right_end_point.size() > 0 && mi.right_end_point[1].x < 40) {
		state_in = left_circle_inside_before;
		return true;
	}
	return false;
}

bool j_left_circle_inside(const MainImage& mi, uchar& state_in) {
	if ((mi.left_end_point.size() == 0 && mi.right_end_point.size() == 2 && mi.right_end_point[1].x < IMGW / 2 - 20 &&
		(mi.right_end_point[0].y > IMGH - 5 || mi.right_end_point[0].x > IMGW - 5)) ||
		(mi.left_end_point.size() == 2 && mi.left_end_point[1].y > IMGH / 2 + 10 && mi.right_end_point.size() == 2 && mi.right_end_point[1].x < IMGW / 2 - 20 &&
			(mi.right_end_point[0].y > IMGH - 5 || mi.right_end_point[0].x > IMGW - 5)))
	{
		return true;
	}
	return false;
}

void j_left_circle_out_find(const MainImage& mi, uchar& state_in) {
	int i, count = 0;
	const uchar* r = mi.store.image_mat.ptr<uchar>(mi.right_end_point[1].y - 1);
	for (i = mi.right_end_point[1].x - 3; i < mi.right_end_point[1].x + 4; i++) {
		if (r[i] != 0)count++;
	}
	if (mi.right_end_point.size() == 2 && (mi.right_end_point[0].y > IMGH - MINY || mi.right_end_point[0].x > IMGW - MINX) && count > 2) {
		state_in = left_circle_out_find;
	}
}

void j_left_circle_out_strai(const MainImage& mi, uchar& state_in) {
	int count = 0;
	//const uchar* r = mi.store.image_mat.ptr<uchar>(IMGH - mi.re.l_circle.out_strai_find_pos);
	const uchar* r = mi.store.image_mat.ptr<uchar>(mi.right_end_point[1].y - 20);
	for (int i = IMGW - 1; i > -1; i--) {
		if (r[i] != 0)count++;
		else break;
	}
	if (count > IMGW - 10) {
		state_in = left_circle_out_strai;
	}
}

void j_left_circle_out_out(const MainImage& mi, uchar& state_in) {
	if (mi.right_end_point.size() != 0 && abs(mi.right_end_point[0].y - mi.right_end_point[1].y) > 30 &&
		mi.left_end_point.size() != 0 && mi.left_end_point[0].y < IMGH - 30) {
		state_in = left_circle_out_out;
	}
}

bool j_farm_in_find(const MainImage& mi, uchar& state_in) {
	int i;
	int start = IMGW / 2;
	int y = IMGH - 1;
	int count = 0;
	int error = 0;
	const uchar* r = mi.store.image_mat.ptr<uchar>(y);
	while (r[start] != 0)
	{
		y--;
		const uchar* r = mi.store.image_mat.ptr<uchar>(y);
	}
	if (y < 50 || y > 70)return false;
	r = mi.store.image_mat.ptr<uchar>(y);
	for (i = 0; i < 30; i++)
	{
		int k;
		int c = 0;
		bool left = false;
		bool right = false;
		y--;
		if (r[start] != 0)return false;
		const uchar* r = mi.store.image_mat.ptr<uchar>(y - 2);
		/*for (k = start - 1; k >= start - 35; k--) {
			if (r[k] == 0)c++;
			else {
				left = true;
			}
		}
		for (k = start + 1; k <= start + 35; k++) {
			if (r[k] == 0)c++;
			else {
				right = true;
			}
		}
		if (!left || !right)return false;
		if (c < 20)error++;
		if (error > 2)return false;*/
	}
	return true;
}

void j_farm_inside(const MainImage& mi, uchar& state_in) {
	int c1, c2, c3;
	c1 = count_white(mi.store.image_mat, IMGH - 10);
	c2 = count_white(mi.store.image_mat, IMGH - 11);
	c3 = count_white(mi.store.image_mat, IMGH - 12);
	if (c1 < 50 && c2 < 50 && c3 < 50)
	{
		state_in = farm_inside;
	}
}

void j_farm_out_find(const MainImage& mi, uchar& state_in) {
	/*if (mi.right_cone.back().y > IMGH / 2 && mi.right_cone.back().x < IMGW - 40)
	{
		int count = 0;
		int start = mi.right_cone.back().x;
	if(mi.right_cone.back().y>40)
	{
	  const uchar* r = mi.store.image_mat.ptr<uchar>(mi.right_cone.back().y + 15);
		for (int i = start - 1; i > 0; i--)
		  {
			  if (r[i] != 0)count++;
		}
		if (count > mi.re.farm.out_find_count)state_in = farm_out_find;
	}
	}
	if (mi.left_cone.back().y > IMGH / 2 && mi.left_cone.back().x > 40)
	{
		int count = 0;
		int start = mi.left_cone.back().x;
	if(mi.left_cone.back().y>40)
	{
	  const uchar* r = mi.store.image_mat.ptr<uchar>(mi.left_cone.back().y + 15);
		  for (int i = start + 1; i < IMGW - 1; i++)
		  {
			if (r[i] != 0)count++;
		  }
		  if (count > mi.re.farm.out_find_count)state_in = farm_out_find;
	}
	}*/

	/*int i;
	int count = 0;
	int h=mi.left_cone.back().y<mi.right_cone.back().y?mi.left_cone.back().y:mi.right_cone.back().y;
	h=h>30?h-10:h;
	const uchar* r = mi.store.image_mat.ptr<uchar>(h);
	for(i=mi.left_cone.back().x;i<mi.right_cone.back().x;i++)
	{
	  if(r[i]!=0)count++;
	  else{count=0;break;}
	}
	if(count > mi.re.farm.out_find_count)state_in=farm_out_find;
	*/

	//********************

	/*int i;
	int count = 0;
	const uchar* r = mi.store.image_mat.ptr<uchar>(IMGH - mi.re.farm.out_find_line);
	for(i=32;i<IMGW-32;i++)
	{
	  if(r[i]!=0)count++;
	}
	if(count > mi.re.farm.out_find_count)state_in=farm_out_find;*/

	//********************

	int i;
	int count = 0;
	int length;
	if (mi.left_cone.size() > 0 && mi.right_cone.size() > 0)
	{
		int y = mi.left_cone.back().y < mi.right_cone.back().y ? mi.left_cone.back().y : mi.right_cone.back().y;
		y = y >= 40 ? y - 10 : 30;
		length = mi.right_cone.back().y - mi.left_cone.back().y;
		const uchar* r = mi.store.image_mat.ptr<uchar>(y);
		for (i = mi.left_cone.back().y; i < mi.right_cone.back().y; i++)
		{
			if (r[y] != 0)count++;
		}
		if (count / length > 0.8)state_in = farm_out_find;
	}

}

void j_farm_out(const MainImage& mi, uchar& state_in) {
	//if (mi.center_cone.back().y >= IMGH - 5)state_in = farm_out;
	int i;
	int count = 0;
	const uchar* r = mi.store.image_mat.ptr<uchar>(IMGH - mi.re.farm.out_line);
	for (i = 0; i < IMGW - 32; i++)
	{
		if (r[i] != 0)count++;
	}
	if (count > mi.re.farm.out_count)state_in = farm_out;
}



void j_hump_on(const MainImage& mi, uchar& state_in) {
	int i, k;
	int count = 0;
	bool find_down = false;
	for (k = 0; k < 4; k++)
	{
		const uchar* r_down = mi.store.image_mat.ptr<uchar>(IMGH - mi.re.hump.on_line - k);
		for (i = 1; i < IMGW; i++)
		{
			if (r_down[i] != 0 && r_down[i - 1] == 0)
			{
				int k = 0;
				while (r_down[i + k] != 0)k++;
				if (k <= 15)continue;
				else if (k < 70) count++;
				else count = 0;
				break;
			}
		}
	}
	if (count > 2)state_in = hump_on;
}

void j_hump_out(const MainImage& mi, uchar& state_in) {
	int i, k;
	int count = 0;
	bool find_down = false;
	for (k = 0; k < 50; k++)
	{
		const uchar* r_down = mi.store.image_mat.ptr<uchar>(IMGH - 1 - k);
		for (i = 1; i < IMGW; i++)
		{
			if (r_down[i] != 0 && r_down[i - 1] == 0)
			{
				int k = 0;
				while (r_down[i + k] != 0)k++;
				if (k <= 15)continue;
				else if (k > 60) count++;
				else count = 0;
				break;
			}
		}
	}
	if (count == 50)state_in = hump_out;
}



int count_white(const Mat& img, int row, int end, int start)    //第row行白点数统计
{
	int i, count = 0;
	const uchar* this_row = img.ptr<uchar>(row);
	for (i = start; i < end; i++) {
		if (this_row[i] != 0)
			count++;
	}
	return count;
}
int count_wid(const Mat& src, Point seed) {
	int i, count = 1;
	const uchar* this_row = src.ptr<uchar>(seed.y);
	if (this_row[seed.x] != 0) {
		for (i = seed.x - 1; i > -1; i--) {
			if (this_row[i] != 0)count++;
			else break;
		}
		for (i = seed.x + 1; i < IMGW; i++) {
			if (this_row[i] != 0)count++;
			else break;
		}
		return count;
	}
	else return(-1);
}

void line(Mat& img, Point A, Point B, int color)
{
	float x0 = A.x;
	float y0 = A.y;
	float x1 = B.x;
	float y1 = B.y;
	int x, y;
	float k = (y1 - y0) / (x1 - x0);
	if (x1 < x0)
	{
		while (true)
		{
			if (k > 0)
			{
				if (k > 1)
				{
					x0 -= 1 / k;
					y0--;
				}
				else
				{
					x0--;
					y0 -= k;
				}
				x = round(x0);
				y = round(y0);
				if (x <= x1 && y <= y1)break;
				img.ptr<uchar>(y)[x] = color;
			}
			else
			{
				if (k < -1)
				{
					x0 += 1 / k;
					y0++;
				}
				else
				{
					x0--;
					y0 -= k;
				}
				x = round(x0);
				y = round(y0);
				if (x <= x1 && y >= y1)break;
				img.ptr<uchar>(y)[x] = color;
			}
		}
	}
	else
	{
		while (true)
		{
			if (k > 0)
			{
				if (k > 1)
				{
					x0 += 1 / k;
					y0++;
				}
				else
				{
					x0++;
					y0 += k;
				}
				x = round(x0);
				y = round(y0);
				if (x >= x1 && y >= y1)break;
				img.ptr<uchar>(y)[x] = color;
			}
			else
			{
				if (k < -1)
				{
					x0 -= 1 / k;
					y0--;
				}
				else
				{
					x0++;
					y0 += k;
				}
				x = round(x0);
				y = round(y0);
				if (x >= x1 && y <= y1)break;
				img.ptr<uchar>(y)[x] = color;
			}
		}
	}
}
void ray(Mat& img, Point start, float angle, int color)
{
	float x = start.x;
	float y = start.y;
	int x0, y0;
	float k = tan(angle);
	if (abs(angle) > 1.5707)
	{
		while (true)
		{
			if (k > 1) {
				x -= 1 / k;
				y++;
			}
			else if (k > 0 && k <= 1) {
				x--;
				y += k;
			}
			else if (k > -1 && k <= 0) {
				x--;
				y += k;
			}
			else if (k <= -1) {
				x += 1 / k;
				y--;
			}
			x0 = round(x);
			y0 = round(y);
			if (y0 > IMGH - 1 || y0 < 1 || x0 < 1 || x0 > IMGW - 1) break;
			img.ptr<uchar>(y0)[x0] = color;
		}
	}
	else
	{
		while (true)
		{
			if (k > 1) {
				x += 1 / k;
				y--;
			}
			else if (k > 0 && k <= 1) {
				x++;
				y -= k;
			}
			else if (k > -1 && k <= 0) {
				x++;
				y -= k;
			}
			else if (k <= -1) {
				x -= 1 / k;
				y++;
			}
			x0 = round(x);
			y0 = round(y);
			if (y0 > IMGH - 1 || y0 < 1 || x0 < 1 || x0 > IMGW - 1) break;
			img.ptr<uchar>(y0)[x0] = color;
		}
	}
}


void MainImage::find_far_zebra() {
	int zebra_dist = re.zebra.zebra_far_dist;
	int thresh = IMGH - zebra_dist;
	int i, j;
	int count = 0, r_count = 0;
	bool f = false;
	uchar* r;
	for (i = thresh; i >= thresh - 8; i--) {
		r = store.image_mat.ptr<uchar>(i);
		for (j = 10; j < IMGW - 10; j++) {
			if (r[j - 2] != 0 && r[j - 1] != 0 && r[j] != 0) {
				f = true;
				break;
			}
		}
		if (!f) return;
		for (; j < IMGW; j++) {
			if (r[j] == 0 && r[j + 1] == 0 && r[j + 2] == 0 && r[j + 3] == 0 && r[j + 4] == 0 && r[j + 5] == 0 && r[j + 6] == 0 && r[j + 7] == 0 && r[j + 8] == 0) break;
			if (r[j] != r[j + 1]) count++;
			if ((r[j] != 0 && r[j + 1] == 0 && r[j + 2] != 0) || (r[j] != 0 && r[j + 1] == 0 && r[j + 2] == 0 && r[j + 3] != 0)) count = count - 2;
		}
		if (count >= 10) {
			r_count++;
			if (r_count >= 3) {
				zebra_far_find = true;
				return;
			}
		}
		else {
			count = 0;
		}
	}
	zebra_far_find = false;
}

void MainImage::find_near_zebra() {
	int zebra_dist = re.zebra.zebra_near_dist;
	int thresh = IMGH - zebra_dist;
	int i, j;
	int count = 0, r_count = 0;
	bool f = false;
	uchar* r;
	for (i = thresh; i >= thresh - 8; i--) {
		r = store.image_mat.ptr<uchar>(i);
		for (j = 10; j < IMGW - 10; j++) {
			if (r[j - 2] != 0 && r[j - 1] != 0 && r[j] != 0) {
				f = true;
				break;
			}
		}
		if (!f) return;
		for (; j < IMGW; j++) {
			if (r[j] == 0 && r[j + 1] == 0 && r[j + 2] == 0 && r[j + 3] == 0 && r[j + 4] == 0 && r[j + 5] == 0 && r[j + 6] == 0 && r[j + 7] == 0 && r[j + 8] == 0) break;
			if (r[j] != r[j + 1]) count++;
		}
		if (count >= 10) {
			r_count++;
			if (r_count >= 2) {
				zebra_near_find = true;
				return;
			}
		}
		else {
			count = 0;
		}
	}
	zebra_near_find = false;

}