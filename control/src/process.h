#ifndef __TANGENT_HEADER__
#define __TANGENT_HEADER__
#define Re MI.re

#include <opencv2/opencv.hpp>
#include <future>
#include"reader.h"

using namespace cv;
using namespace std;

class ImageStorage  //图片处理
{
public:
	int save_num = 1;
	cv::VideoCapture cap;
	cv::VideoWriter wri;
 	bool Writer_Exist=false;
	future<cv::Mat> fut;
	Mat image_BGR;  //彩色图
	Mat image_R;    //红色图
	Mat image_mat;	//处理图
	Mat image_show;	//展示图
	ImageStorage();
	void get_image(int x, int y, int w, int h, int ai_x, int ai_y, int ai_w, int ai_h, bool f = true);
};

class MainImage    //寻线
{
private:
	struct CurvaturePoint {
    	int x, y;
	};
	float calc_curvature(CurvaturePoint p1, CurvaturePoint p2, CurvaturePoint p3);
public:
	uchar left_edge_point[IMGH];        //左边界
	uchar right_edge_point[IMGH];       //右边界
	uchar center_point[IMGH];           //中线
	bool exist_left_edge_point[IMGH];   //是否找到左边界点
	bool exist_right_edge_point[IMGH];  //是否找到右边界点
	int last_center;                    //上一段找寻的中点位置
	int center_lost;                    //无法搜索到中心点位置
	vector<Point> left_end_point;       //左断点
	vector<Point> right_end_point;      //右断点
	bool lost_left;						//左丢边
	bool lost_right;					//右丢边
	int left_branch_num;				//左支路数
	int right_branch_num;				//右支路数
	bool left_continue;					//左连续
	bool right_continue;				//右连续
	int count_circle;					//圈数

	int deviation_thresh;
	int angle_new_forward_dist;
	int speed_deviation_thresh;
	int slope_thresh_near;
	int slope_thresh_far;
	int slop_direction_thresh;
	float cur_kp_show;

	bool zebra_far_find;
	bool zebra_near_find;

	vector<Point> left_cone;
	vector<Point> right_cone;
	vector<Point> center_cone;
	int last_center_in_cone;

	Point repair_cone;
	
	//ai元素
	bool ai_bridge;
	bool ai_tractor;
	bool ai_corn;
	bool ai_pig;
  
	uchar state_out;
	uchar state_l_circle;
	uchar state_r_circle;
	uchar state_farm;
	uchar state_repair;
	uchar state_barn;
  	uchar state_hump;
	uchar state_in_garage;
	uchar state_last;
	uchar state_hill;

	Reader re;
	ImageStorage store;

	int enc_speed;
	int last_enc_speed;
	//较近前瞻位置附近的曲率
	float curvature_near;
	//较远前瞻位置附近的曲率
	float curvature_far;
	float smoothed_curvature_near;
	float smoothed_curvature_far;
	float slope;
	float angle_deviation;

	MainImage();
	void init(bool complete);//初始化
	void update_image();//帧处理
	void state_judge();	//状态判断
	void update_control(float& kp, float& kd, float& ki, int& dv);

	void find_edge_point();    //搜索边界点
	void find_end_point(); //搜索断点
	void judge_lost();	//判断丢边
	void count_branch();	//判断支路数
	void judge_continuity();	//判断连续性
	void edge_filter(int wid, int side = 2);	//边缘过滤，side=0表示左边，side=1表示右边， 不输入或side=2表示同时
	void end_filter(int side = 2);	//断点过滤，side=0表示左边，side=1表示右边， 不输入或side=2表示同时
	void count_cone(int begin = IMGH - 5);

	void mend_trunk();	//straight修补
	void mend_right_circle_in_straight();	//右环岛直线段修补
	void mend_right_circle_in_circle();		//右环岛弧线段修补
	void mend_left_circle_in_straight();
	void mend_left_circle_in_circle();
	void mend_farm_in_find();
	void mend_farm_inside();
	void mend_farm_out_find();
  	void mend_in_hump_on();

	void refind_edge_point();
	void find_center();
	void find_center_in_farm(int begin = IMGH - 5);
	float get_curvature_near(void);
	float get_slope_near(void);
	float get_curvature_far(void);
	void refind_edge_in_farm_out(Point start);
	void find_far_zebra();
	void find_near_zebra();

	void produce_dv(int deviation);

	void show(float dev, float angle_result, float speed, int current_speed, bool c = true, bool l = false, bool r = false, bool l_e = true, bool r_e = true, bool l_c = false, bool r_c = false, bool c_c = false);//总显示方案

	float AngelDeviation(void);
	float SpeedDeviation(void);
	float dy_forward_dist_up(float kp, float kd, float coef, float exp, float bias, float speed_thresh, float max);
	float dy_forward_dist_up_and_down(float kp, float kd, float coef_up, float coef_down, float exp_up, 
									  float exp_down, float bias, float speed_thresh, float max, float min);
};

bool j_right_circle_in_circle(const MainImage& mi, uchar& state_in);
bool j_right_circle_inside_before(const MainImage& mi, uchar& state_in);
bool j_right_circle_inside(const MainImage& mi, uchar& state_in);
void j_right_circle_out_find(const MainImage& mi, uchar& state_in);
void j_right_circle_out_strai(const MainImage& mi, uchar& state_in);
void j_right_circle_out_out(const MainImage& mi, uchar& state_in);
bool j_left_circle_in_circle(const MainImage& mi, uchar& state_in);
bool j_left_circle_inside_before(const MainImage& mi, uchar& state_in);
bool j_left_circle_inside(const MainImage& mi, uchar& state_in);
void j_left_circle_out_find(const MainImage& mi, uchar& state_in);
void j_left_circle_out_strai(const MainImage& mi, uchar& state_in);
void j_left_circle_out_out(const MainImage& mi, uchar& state_in);

bool j_farm_in_find(const MainImage& mi, uchar& state_in);

void j_farm_inside(const MainImage& mi, uchar& state_in);
void j_farm_out_find(const MainImage& mi, uchar& state_in);
void j_farm_out(const MainImage& mi, uchar& state_in);
void j_hump_on(const MainImage& mi, uchar& state_in);
void j_hump_out(const MainImage& mi, uchar& state_in);

int count_white(const Mat& img, int row, int end = IMGW, int start = 0);
int count_wid(const Mat& src, Point seed);
void line(Mat& img, Point A, Point B, int color = 0);	//补线段
void ray(Mat& img, Point start, float angle, int color = 0);	//补射线

extern int stop;
extern MainImage MI;
#endif