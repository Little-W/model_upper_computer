
#ifndef MY_READER_HEADER
#define MY_READER_HEADER

#include <yaml-cpp/yaml.h>
#include <opencv2/core.hpp>
#include <iostream>
#include <string>
#include <utility>

#define IMGH 120
#define IMGW 160
#define MINX 4
#define MINY 4

#define garage_out 40
#define zebra_on 38
#define garage_find 39
#define straight 1
#define right_circle 2
#define left_circle 3
#define cross_find 4
#define repair_find 5
#define hump_find 6
#define barn_find 7
#define farm_find 8
#define repair_in_find 11
#define repair_out_out 12
#define farm_in_find 14
#define farm_inside 15
#define farm_out_find 16
#define farm_out 17

#define hump_in_find 43
#define hump_on 44
#define hump_out 45

#define right_circle_in_find 18
#define right_circle_in_strai 19
#define right_circle_in_circle 20
#define right_circle_inside_before 21
#define right_circle_inside 22
#define right_circle_out_find 23
#define right_circle_out_strai 24
#define right_circle_out 25
#define right_circle_out_out 26

#define left_circle_in_find 27
#define left_circle_in_strai 28
#define left_circle_in_circle 29
#define left_circle_inside_before 30
#define left_circle_inside 31
#define left_circle_out_find 32
#define left_circle_out_strai 33
#define left_circle_out 34
#define left_circle_out_out 35

#define hill_find 36
#define hill_on 37
#define hill_out 46

#define garage_in 41
#define garage_in_find 42
#define garage_inside 44
#define garage_in_before 45

typedef std::pair<int, int> pii;

float expr2f(const std::string& expr);//字符串转浮点
int inline expr2i(const std::string& expr);//字符串转整型

class BaseReader {
public:
    BaseReader() {}
    BaseReader(YAML::Node node);
    bool is_null(std::string tag);
    template <typename T>
    T get(std::string tag) {
        return node[tag].as<T>();
    }
    template <typename T>
    T get_bi(std::string tag1, std::string tag2) {
        BaseReader temp(node[tag1]);
        return temp.get<T>(tag2);
    }
    float get_expr(std::string tag);
private:
    YAML::Node node;
};

class Reader
{
private:
    BaseReader n_set;
    
    BaseReader n_main;
    BaseReader n_right_circle;
    BaseReader n_left_circle;
    BaseReader n_repair;
    BaseReader n_farm;
    BaseReader n_hump;
    BaseReader n_start;
    BaseReader n_end;
    BaseReader n_zebra;
    BaseReader n_hill;
public:
    struct pidv
    {
        float kp;
        float kd;
        float ki;
        int dv;
    };

    Reader(std::string path);
    Reader() : Reader("./config.yaml") {}
    struct pidv get_pidv(int state_out, int state_in);
    struct {
        bool video_save;
        bool color;
        bool motor_use;
        bool zebra_detect;
        int cut_point_x;
        int cut_point_y;
        int cut_w;
        int cut_h;
        int ai_cut_x;
        int ai_cut_y;
        int ai_w;
        int ai_h;
    }set;
    struct {
        bool garage_start;
        int angle_enc_forward_threshold;
        float angle_enc_forward_dist_coef;
        float angle_enc_forward_dist_exp;
        int angle_max_enc_forward_dist;
        float angle_dy_forward_dist_kp;
        float angle_dy_forward_dist_kd;

        int speed_enc_forward_threshold;
        float speed_enc_forward_dist_coef;
        float speed_enc_forward_dist_exp;
        int speed_max_enc_forward_dist;
        float speed_dy_forward_dist_kp;
        float speed_dy_forward_dist_kd;
        float dy_kp_threshold;
        float dy_kp_coef;
        float dy_kp_exp;
        float dy_kp_max;
        float inc_dy_kp_threshold;
        float inc_dy_kp_coef;
        float inc_dy_kp_exp;
        float dy_kd_threshold;
        float dy_kd_coef;
        float dy_speed_coef;
        float dy_speed_exp;
        float dy_speed_bezier_p0_ctrl_x;
        float dy_speed_bezier_p0_ctrl_y;
        float dy_speed_bezier_p1_ctrl_x;
        float dy_speed_bezier_p1_ctrl_y;
        float kp;
        float kd;
        float ki;
        int dv;
        float sc_kp;
        float sc_ki;
        float sc_kd;
        int max_ag;
        int min_ag;
        float max_v;
        float min_v;
        float max_v_diff;
        float min_v_diff;
        bool use_dv;
        bool second_lap;
        float center_coef;
        int forward_dist;
        int speed_forward_dist;
        int up_scope;
        int down_scope;
        float forward_coef1;
        float forward_coef2;
        pii v_right_circle;
        pii v_left_circle;
        float left_ray;
        float right_ray;
        float speed_delta_bezier_p0_ctrl_x;
        float speed_delta_bezier_p0_ctrl_y;
        float speed_delta_bezier_p1_ctrl_x;
        float speed_delta_bezier_p1_ctrl_y;
        float slow_down_kd;
        //斜率和曲率暂时共用配置
        int curvature_up_scope;
	    int curvature_down_scope;
        int slope_calc_dist;
        int slope_forward_dist_near;
        int slope_forward_dist_far;
        int slope_direction_forward_dist;
        float slope_coef;
        float deviation_coef;
    }main;
    struct {
        float kp;
        float ki;
        float kd;
        int dv;
        float speed;
        int circle_dist;
        int max_dy_forward_dist;
        int min_dy_forward_dist;
        float angle_forward_dist_kp;
        float angle_forward_dist_kd;
        float dy_forward_dist_coef_up;
        float dy_forward_dist_exp_up;
        float dy_forward_dist_coef_down;
        float dy_forward_dist_exp_down;
        float circle_slow_down_kd;
        bool use;
        bool big_circle;
        int count_start;
        float in_find_ray_ag;
        float in_strai_ray_ag1;
        float in_strai_ray_ag2;
        float in_circle_ray_ag1;
        float in_circle_ray_ag2;
        float in_circle_ray_ag3;
        float in_circle_ray_ag4;
        float in_circle_ray_ag5;

        float in_find_kp;
        float in_find_kd;
        float in_find_ki;
        float in_find_speed;

        float in_strai_kp;
        float in_strai_kd;
        float in_strai_ki;

        float in_circle_kp;
        float in_circle_kd;
        float in_circle_ki;

        float inside_before_kp;
        float inside_before_kd;
        float inside_before_ki;

        float inside_kp;
        float inside_kd;
        float inside_ki;

        float out_find_kp;
        float out_find_kd;
        float out_find_ki;

        float out_strai_kp;
        float out_strai_kd;
        float out_strai_ki;

        float out_kp;
        float out_kd;
        float out_ki;

        float out_out_kp;
        float out_out_kd;
        float out_out_ki;

        cv::Point inside_before_p;
        cv::Point out_find_p;
        int out_strai_find_pos;
        float out_ray_ag;
    }r_circle, l_circle;
    struct {
        float kp;
        float ki;
        float kd;
        int dv;
        float speed;
    }repair;
    struct {
        float kp;
        float kd;
        float ki;
        int dv;
        float speed;
        float speed_out;
        int dist;
        int up_scope;
        int down_scope;
        int out_find_line;
        int out_find_count;
        int out_line;
        int out_count;
    }farm;
    struct {
        float kp;
        float kd;
        float ki;
        int dv;
        float speed;
        int mend_up_line;
        int on_line;
    }hump;
    struct {
        float kp;
        float ki;
        float kd;
        int dv;
        bool left;
        int start_dist;
        int x_thresh;
        pii v_left;
        pii v_right;
    }start;
    struct {
        int end_dist;
        int end_whitecount;
        pii v_left_garage;
        pii v_right_garage;  
    }end;
    struct {
        float speed;
        float kp;
        float kd;
        float ki;
        int dv;
        int zebra_far_dist;
        int zebra_near_dist;
        pii v_left_zebra;
        pii v_right_zebra;
    }zebra;
    struct {
        float kp;
        float kd;
        float ki;
        int dv;
        float speed;
        int mid_bot_y;
        int mid_top_y;
        int frame;
    }hill;
};

#endif