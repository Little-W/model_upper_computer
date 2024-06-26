#include  "reader.h"
#include <iostream>
#include <cmath>

using namespace std;
using cv::Point;
using namespace YAML;

enum optr { none, add, sub, mul, dev };

float eval(float val1, float val2, optr op) {
    switch (op)
    {
    case add:
        return val1 + val2;
    case sub:
        return val1 - val2;
    case mul:
        return val1 * val2;
    case dev:
        return val1 / val2;
    default:
        cerr << "Invaild operator: " << op << endl;
        throw op;
    }
}
float mystof(string str) {
    if (str == string("IMGH")) return IMGH;
    else if (str == string("IMGW")) return IMGW;
    else return stof(str);
}
float expr2f(const string& expr) {

    if (expr.size() == 1) return mystof(expr);
    unsigned int start = 0;
    float val = 0;
    optr op = none;
    unsigned int pos = 1;
    string temp;
    for (; pos < expr.size(); pos++) {
        if (expr[pos] == '+') {
            temp = expr.substr(start, pos - start);
            val = mystof(temp);
            start = ++pos;
            op = add;
            break;
        }
        else if (expr[pos] == '-') {
            temp = expr.substr(start, pos - start);
            val = mystof(temp);
            start = ++pos;
            op = sub;
            break;
        }
        else if (expr[pos] == '*') {
            temp = expr.substr(start, pos - start);
            val = mystof(temp);
            start = ++pos;
            op = mul;
            break;
        }
        else if (expr[pos] == '/') {
            temp = expr.substr(start, pos - start);
            val = mystof(temp);
            start = ++pos;
            op = dev;
            break;
        }
    }
    if (pos == expr.size()) return mystof(expr);
    for (; pos < expr.size(); pos++) {
        if (expr[pos] == '+') {
            temp = expr.substr(start, pos - start);
            val = eval(val, mystof(temp), op);
            start = ++pos;
            op = add;
        }
        else if (expr[pos] == '-') {
            temp = expr.substr(start, pos - start);
            val = eval(val, mystof(temp), op);
            start = ++pos;
            op = sub;
        }
        else if (expr[pos] == '*') {
            temp = expr.substr(start, pos - start);
            val = eval(val, mystof(temp), op);
            start = pos + 1;
            op = mul;
        }
        else if (expr[pos] == '/') {
            temp = expr.substr(start, pos - start);
            val = eval(val, mystof(temp), op);
            start = pos + 1;
            op = dev;
        }
    }
    temp = expr.substr(start, pos - start);
    val = eval(val, mystof(temp), op);
    return val;
}
int inline expr2i(const string& expr) {
    return int(round(expr2f(expr)));
}

BaseReader::BaseReader(YAML::Node node_) {
    this->node = node_;
}
bool BaseReader::is_null(std::string tag) {
    return node[tag].IsNull();
}
template <>
Point BaseReader::get<Point>(std::string tag) {
    YAML::const_iterator iter = node[tag].begin();
    const std::string sx = iter->as<std::string>();
    iter++;
    const std::string sy = iter->as<std::string>();
    Point p;
    p.x = expr2i(sx);
    p.y = expr2i(sy);
    return p;
}
template <>
pii BaseReader::get<pii>(std::string tag) {
    pii p;
    YAML::const_iterator iter = node[tag].begin();
    p.first = iter->as<int>();
    iter++;
    p.second = iter->as<int>();
    return p;
}
float BaseReader::get_expr(std::string tag) {
    const auto expr = node[tag].as<string>();
    return expr2f(expr);
}

Reader::Reader(string path) {
    auto root = YAML::LoadFile(path);
    n_set=root["set"];
    n_main = root["main"];
    n_right_circle = root["right_circle"];
    n_left_circle = root["left_circle"];
    n_repair = root["repair"];
    n_farm = root["farm"];
    n_hump = root["hump"];
    n_start=root["start"];
    n_end=root["end"];
    n_zebra=root["zebra"];
    n_hill=root["hill"];
    n_turn=root["turn"];
    n_cone=root["cone"];
    n_garage=root["garage"];
    //set
    set.video_save=n_set.get<bool>("video_save");
    set.color=n_set.get<bool>("color");
    set.motor_use=n_set.get<bool>("motor_use");
    set.zebra_detect=n_set.get<bool>("zebra_detect");
    set.cut_point_x=n_set.get<int>("cut_point_x");
    set.cut_point_y=n_set.get<int>("cut_point_y");
    set.cut_w=n_set.get<int>("cut_w");
    set.cut_h=n_set.get<int>("cut_h");
    set.ai_cut_x=n_set.get<int>("ai_cut_x");
    set.ai_cut_y=n_set.get<int>("ai_cut_y");
    set.ai_w=n_set.get<int>("ai_w");
    set.ai_h=n_set.get<int>("ai_h");
    //main
    main.garage_start = n_main.get<bool>("garage_start");
    main.angle_enc_forward_threshold = n_main.get<int>("angle_enc_forward_threshold");
    main.angle_enc_forward_dist_coef = n_main.get<float>("angle_enc_forward_dist_coef");
    main.angle_enc_forward_dist_exp = n_main.get<float>("angle_enc_forward_dist_exp");
    main.angle_max_enc_forward_dist = n_main.get<int>("angle_max_enc_forward_dist");
    main.angle_dy_forward_dist_kp = n_main.get<float>("angle_dy_forward_dist_kp");
    main.angle_dy_forward_dist_kd = n_main.get<float>("angle_dy_forward_dist_kd");

    main.speed_enc_forward_threshold = n_main.get<int>("speed_enc_forward_threshold");
    main.speed_enc_forward_dist_coef = n_main.get<float>("speed_enc_forward_dist_coef");
    main.speed_enc_forward_dist_exp = n_main.get<float>("speed_enc_forward_dist_exp");
    main.speed_max_enc_forward_dist_far = n_main.get<int>("speed_max_enc_forward_dist_far");
    main.speed_max_enc_forward_dist_near = n_main.get<int>("speed_max_enc_forward_dist_near");
    main.speed_dy_forward_dist_kp = n_main.get<float>("speed_dy_forward_dist_kp");
    main.speed_dy_forward_dist_kd = n_main.get<float>("speed_dy_forward_dist_kd");

    main.dy_kp_threshold = n_main.get<float>("dy_kp_threshold");
    main.dy_kp_coef = n_main.get<float>("dy_kp_coef");
    main.dy_kp_exp = n_main.get<float>("dy_kp_exp");
    main.dy_kp_max = n_main.get<float>("dy_kp_max");
    main.inc_dy_kp_threshold = n_main.get<float>("inc_dy_kp_threshold");
    main.inc_dy_kp_coef = n_main.get<float>("inc_dy_kp_coef");
    main.inc_dy_kp_exp = n_main.get<float>("inc_dy_kp_exp");
    main.dy_kd_threshold = n_main.get<float>("dy_kd_threshold");
    main.dy_kd_coef = n_main.get<float>("dy_kd_coef");
    main.dy_speed_coef = n_main.get<float>("dy_speed_coef");
    main.dy_speed_exp = n_main.get<float>("dy_speed_exp");
    main.dy_speed_bezier_p0_ctrl_x = n_main.get<float>("dy_speed_bezier_p0_ctrl_x");
    main.dy_speed_bezier_p0_ctrl_y = n_main.get<float>("dy_speed_bezier_p0_ctrl_y");
    main.dy_speed_bezier_p1_ctrl_x = n_main.get<float>("dy_speed_bezier_p1_ctrl_x");
    main.dy_speed_bezier_p1_ctrl_y = n_main.get<float>("dy_speed_bezier_p1_ctrl_y");
    
    main.slowdown_enhance_bezier_p0_ctrl_x = n_main.get<float>("slowdown_enhance_bezier_p0_ctrl_x");
    main.slowdown_enhance_bezier_p0_ctrl_y = n_main.get<float>("slowdown_enhance_bezier_p0_ctrl_y");
    main.slowdown_enhance_bezier_p1_ctrl_x = n_main.get<float>("slowdown_enhance_bezier_p1_ctrl_x");
    main.slowdown_enhance_bezier_p1_ctrl_y = n_main.get<float>("slowdown_enhance_bezier_p1_ctrl_y");
    main.slow_down_kd = n_main.get<float>("slow_down_kd");
    main.slowdown_smooth_bezier_p0_ctrl_x = n_main.get<float>("slowdown_smooth_bezier_p0_ctrl_x");
    main.slowdown_smooth_bezier_p0_ctrl_y = n_main.get<float>("slowdown_smooth_bezier_p0_ctrl_y");
    main.slowdown_smooth_bezier_p1_ctrl_x = n_main.get<float>("slowdown_smooth_bezier_p1_ctrl_x");
    main.slowdown_smooth_bezier_p1_ctrl_y = n_main.get<float>("slowdown_smooth_bezier_p1_ctrl_y");
    main.slow_down_smooth_thresh = n_main.get<float>("slow_down_smooth_thresh");
    main.kp = n_main.get<float>("kp");
    main.kd = n_main.get<float>("kd");
    main.ki = n_main.get<float>("ki");
    main.dv = n_main.get<int>("dv");
    main.sc_kp = n_main.get<float>("sc_kp");
    main.sc_ki = n_main.get<float>("sc_ki");
    main.sc_kd = n_main.get<float>("sc_kd");

    main.max_ag = n_main.get<int>("max_ag");
    main.min_ag = n_main.get<int>("min_ag");
    main.max_v = n_main.get<float>("max_v");
    main.min_v = n_main.get<float>("min_v");
    main.max_v_diff = n_main.get<float>("max_v_diff");
    main.min_v_diff = n_main.get<float>("min_v_diff");
    main.use_dv = n_main.get<bool>("use_dv");
    main.garage_start = n_main.get<bool>("garage_start");
    main.second_lap = n_main.get<bool>("second_lap");
    main.center_coef = n_main.get<float>("center_coef");
    main.forward_dist = n_main.get<int>("forward_dist");
    main.speed_forward_dist_far = n_main.get<int>("speed_forward_dist_far");
    main.speed_forward_dist_near = n_main.get<int>("speed_forward_dist_near");
    main.up_scope = n_main.get<int>("up_scope");
    main.down_scope = n_main.get<int>("down_scope");
    main.curvature_up_scope = n_main.get<int>("curvature_up_scope");
    main.curvature_down_scope = n_main.get<int>("curvature_down_scope");
    main.slope_forward_dist_near = n_main.get<int>("slope_forward_dist_near");
    main.slope_forward_dist_far = n_main.get<int>("slope_forward_dist_far");
    main.slope_direction_forward_dist = n_main.get<int>("slope_direction_forward_dist");
    main.forward_coef1 = n_main.get<float>("forward_coef1");
    main.forward_coef2 = n_main.get<float>("forward_coef2");
    main.v_right_circle = n_main.get<pii>("v_right_circle");
    main.v_left_circle = n_main.get<pii>("v_left_circle");
    main.left_ray = n_main.get<float>("left_ray");
    main.right_ray = n_main.get<float>("right_ray");
    main.deviation_coef = n_main.get<float>("deviation_coef");
    main.slope_coef = n_main.get<float>("slope_coef");
    main.cone_speed = n_main.get<int>("cone_speed");
    main.cone_slowdown_thresh = n_main.get<int>("cone_slowdown_thresh");
    main.cone_trapezium_long = n_main.get<float>("cone_trapezium_long");
    main.cone_trapezium_slope=n_main.get<float>("cone_trapezium_slope");
    main.cone_trapezium_slope_with_y=n_main.get<float>("cone_trapezium_slope_with_y");


    //right_circle
    r_circle.kp = n_right_circle.get<float>("kp");
    r_circle.kd = n_right_circle.get<float>("kd");
    r_circle.dv = n_right_circle.get<int>("dv");
    r_circle.speed = n_right_circle.get<float>("speed"); 
    r_circle.circle_dist = n_right_circle.get<int>("circle_dist");
    r_circle.max_dy_forward_dist = n_right_circle.get<int>("max_dy_forward_dist");
    r_circle.min_dy_forward_dist = n_right_circle.get<int>("min_dy_forward_dist");
    r_circle.angle_forward_dist_kp = n_right_circle.get<float>("angle_forward_dist_kp");  
    r_circle.angle_forward_dist_kd = n_right_circle.get<float>("angle_forward_dist_kd");
    r_circle.dy_forward_dist_coef_up = n_right_circle.get<float>("dy_forward_dist_coef_up");
    r_circle.dy_forward_dist_exp_up = n_right_circle.get<float>("dy_forward_dist_exp_up");
    r_circle.dy_forward_dist_coef_down = n_right_circle.get<float>("dy_forward_dist_coef_down");
    r_circle.dy_forward_dist_exp_down = n_right_circle.get<float>("dy_forward_dist_exp_down");
    r_circle.circle_slow_down_kd = n_right_circle.get<float>("circle_slow_down_kd");
    r_circle.slowdown_enhance_bezier_p0_ctrl_x = n_right_circle.get<float>("slowdown_enhance_bezier_p0_ctrl_x");
    r_circle.slowdown_enhance_bezier_p0_ctrl_y = n_right_circle.get<float>("slowdown_enhance_bezier_p0_ctrl_y");
    r_circle.slowdown_enhance_bezier_p1_ctrl_x = n_right_circle.get<float>("slowdown_enhance_bezier_p1_ctrl_x");
    r_circle.slowdown_enhance_bezier_p1_ctrl_y = n_right_circle.get<float>("slowdown_enhance_bezier_p1_ctrl_y");
    r_circle.use = n_right_circle.get<bool>("use");
    r_circle.big_circle = n_right_circle.get<bool>("big_circle");
    r_circle.count_start = n_right_circle.get<int>("count_start");
    r_circle.in_find_ray_ag = n_right_circle.get_bi<float>("in_find", "ray_ag");
    r_circle.in_strai_ray_ag1 = n_right_circle.get_bi<float>("in_strai", "ray_ag1");
    r_circle.in_strai_ray_ag2 = n_right_circle.get_bi<float>("in_strai", "ray_ag2");
    r_circle.in_circle_ray_ag1 = n_right_circle.get_bi<float>("in_circle", "ray_ag1");
    r_circle.in_circle_ray_ag2 = n_right_circle.get_bi<float>("in_circle", "ray_ag2");
    r_circle.in_circle_ray_ag3 = n_right_circle.get_bi<float>("in_circle", "ray_ag3");
    r_circle.in_circle_ray_ag4 = n_right_circle.get_bi<float>("in_circle", "ray_ag4");
    r_circle.in_circle_ray_ag5 = n_right_circle.get_bi<float>("in_circle", "ray_ag5");
    r_circle.inside_before_p = n_right_circle.get_bi<Point>("inside_before", "p");
    r_circle.out_find_p = n_right_circle.get_bi<Point>("out_find", "p");
    r_circle.out_strai_find_pos = n_right_circle.get_bi<int>("out_strai", "find_pos");
    r_circle.out_ray_ag = n_right_circle.get_bi<float>("out", "ray_ag");

    r_circle.in_find_kp = n_right_circle.get_bi<float>("in_find", "kp");
    r_circle.in_find_kd = n_right_circle.get_bi<float>("in_find", "kd");
    r_circle.in_find_ki = n_right_circle.get_bi<float>("in_find", "ki");

    r_circle.in_strai_kp = n_right_circle.get_bi<float>("in_strai", "kp");
    r_circle.in_strai_kd = n_right_circle.get_bi<float>("in_strai", "kd");
    r_circle.in_strai_ki = n_right_circle.get_bi<float>("in_strai", "ki");

    r_circle.in_circle_kp = n_right_circle.get_bi<float>("in_circle", "kp");
    r_circle.in_circle_kd = n_right_circle.get_bi<float>("in_circle", "kd");
    r_circle.in_circle_ki = n_right_circle.get_bi<float>("in_circle", "ki");

    r_circle.inside_before_kp = n_right_circle.get_bi<float>("inside_before", "kp");
    r_circle.inside_before_kd = n_right_circle.get_bi<float>("inside_before", "kd");
    r_circle.inside_before_ki = n_right_circle.get_bi<float>("inside_before", "ki");

    r_circle.inside_kp = n_right_circle.get_bi<float>("inside", "kp");
    r_circle.inside_kd = n_right_circle.get_bi<float>("inside", "kd");
    r_circle.inside_ki = n_right_circle.get_bi<float>("inside", "ki");
    r_circle.in_find_speed = n_right_circle.get_bi<float>("in_find", "speed");

    r_circle.out_find_kp = n_right_circle.get_bi<float>("out_find", "kp");
    r_circle.out_find_kd = n_right_circle.get_bi<float>("out_find", "kd");
    r_circle.out_find_ki = n_right_circle.get_bi<float>("out_find", "ki");

    r_circle.out_strai_kp = n_right_circle.get_bi<float>("out_strai", "kp");
    r_circle.out_strai_kd = n_right_circle.get_bi<float>("out_strai", "kd");
    r_circle.out_strai_ki = n_right_circle.get_bi<float>("out_strai", "ki");

    r_circle.out_kp = n_right_circle.get_bi<float>("out", "kp");
    r_circle.out_kd = n_right_circle.get_bi<float>("out", "kd");
    r_circle.out_ki = n_right_circle.get_bi<float>("out", "ki");

    r_circle.out_out_kp = n_right_circle.get_bi<float>("out_out", "kp");
    r_circle.out_out_kd = n_right_circle.get_bi<float>("out_out", "kd");
    r_circle.out_out_ki = n_right_circle.get_bi<float>("out_out", "ki");

    
    //left_circle
    l_circle.kp = n_left_circle.get<float>("kp");
    l_circle.kd = n_left_circle.get<float>("kd");
    l_circle.dv = n_left_circle.get<int>("dv");
    l_circle.speed = n_left_circle.get<float>("speed");
    l_circle.circle_dist = n_left_circle.get<int>("circle_dist");
    l_circle.max_dy_forward_dist = n_left_circle.get<int>("max_dy_forward_dist");
    l_circle.min_dy_forward_dist = n_left_circle.get<int>("min_dy_forward_dist");
    l_circle.angle_forward_dist_kp = n_left_circle.get<float>("angle_forward_dist_kp");  
    l_circle.angle_forward_dist_kd = n_left_circle.get<float>("angle_forward_dist_kd");
    l_circle.dy_forward_dist_coef_up = n_left_circle.get<float>("dy_forward_dist_coef_up");
    l_circle.dy_forward_dist_exp_up = n_left_circle.get<float>("dy_forward_dist_exp_up");
    l_circle.dy_forward_dist_coef_down = n_left_circle.get<float>("dy_forward_dist_coef_down");
    l_circle.dy_forward_dist_exp_down = n_left_circle.get<float>("dy_forward_dist_exp_down");
    l_circle.circle_slow_down_kd = n_left_circle.get<float>("circle_slow_down_kd");
    l_circle.slowdown_enhance_bezier_p0_ctrl_x = n_left_circle.get<float>("slowdown_enhance_bezier_p0_ctrl_x");
    l_circle.slowdown_enhance_bezier_p0_ctrl_y = n_left_circle.get<float>("slowdown_enhance_bezier_p0_ctrl_y");
    l_circle.slowdown_enhance_bezier_p1_ctrl_x = n_left_circle.get<float>("slowdown_enhance_bezier_p1_ctrl_x");
    l_circle.slowdown_enhance_bezier_p1_ctrl_y = n_left_circle.get<float>("slowdown_enhance_bezier_p1_ctrl_y");
    l_circle.use = n_left_circle.get<bool>("use");
    l_circle.big_circle = n_left_circle.get<bool>("big_circle");
    l_circle.count_start = n_left_circle.get<int>("count_start");
    l_circle.in_find_ray_ag = n_left_circle.get_bi<float>("in_find", "ray_ag");
    l_circle.in_strai_ray_ag1 = n_left_circle.get_bi<float>("in_strai", "ray_ag1");
    l_circle.in_strai_ray_ag2 = n_left_circle.get_bi<float>("in_strai", "ray_ag2");
    l_circle.in_circle_ray_ag1 = n_left_circle.get_bi<float>("in_circle", "ray_ag1");
    l_circle.in_circle_ray_ag2 = n_left_circle.get_bi<float>("in_circle", "ray_ag2");
    l_circle.in_circle_ray_ag3 = n_left_circle.get_bi<float>("in_circle", "ray_ag3");
    l_circle.in_circle_ray_ag4 = n_left_circle.get_bi<float>("in_circle", "ray_ag4");
    l_circle.in_circle_ray_ag5 = n_left_circle.get_bi<float>("in_circle", "ray_ag5");
    l_circle.inside_before_p = n_left_circle.get_bi<Point>("inside_before", "p");
    l_circle.out_find_p = n_left_circle.get_bi<Point>("out_find", "p");
    l_circle.out_strai_find_pos = n_left_circle.get_bi<int>("out_strai", "find_pos");
    l_circle.out_ray_ag = n_left_circle.get_bi<float>("out", "ray_ag");

    l_circle.in_find_kp = n_left_circle.get_bi<float>("in_find", "kp");
    l_circle.in_find_kd = n_left_circle.get_bi<float>("in_find", "kd");
    l_circle.in_find_ki = n_left_circle.get_bi<float>("in_find", "ki");
    l_circle.in_find_speed = n_left_circle.get_bi<float>("in_find", "speed");

    l_circle.in_strai_kp = n_left_circle.get_bi<float>("in_strai", "kp");
    l_circle.in_strai_kd = n_left_circle.get_bi<float>("in_strai", "kd");
    l_circle.in_strai_ki = n_left_circle.get_bi<float>("in_strai", "ki");

    l_circle.in_circle_kp = n_left_circle.get_bi<float>("in_circle", "kp");
    l_circle.in_circle_kd = n_left_circle.get_bi<float>("in_circle", "kd");
    l_circle.in_circle_ki = n_left_circle.get_bi<float>("in_circle", "ki");

    l_circle.inside_before_kp = n_left_circle.get_bi<float>("inside_before", "kp");
    l_circle.inside_before_kd = n_left_circle.get_bi<float>("inside_before", "kd");
    l_circle.inside_before_ki = n_left_circle.get_bi<float>("inside_before", "ki");

    l_circle.inside_kp = n_left_circle.get_bi<float>("inside", "kp");
    l_circle.inside_kd = n_left_circle.get_bi<float>("inside", "kd");
    l_circle.inside_ki = n_left_circle.get_bi<float>("inside", "ki");

    l_circle.out_find_kp = n_left_circle.get_bi<float>("out_find", "kp");
    l_circle.out_find_kd = n_left_circle.get_bi<float>("out_find", "kd");
    l_circle.out_find_ki = n_left_circle.get_bi<float>("out_find", "ki");

    l_circle.out_strai_kp = n_left_circle.get_bi<float>("out_strai", "kp");
    l_circle.out_strai_kd = n_left_circle.get_bi<float>("out_strai", "kd");
    l_circle.out_strai_ki = n_left_circle.get_bi<float>("out_strai", "ki");

    l_circle.out_kp = n_left_circle.get_bi<float>("out", "kp");
    l_circle.out_kd = n_left_circle.get_bi<float>("out", "kd");
    l_circle.out_ki = n_left_circle.get_bi<float>("out", "ki");

    l_circle.out_out_kp = n_left_circle.get_bi<float>("out_out", "kp");
    l_circle.out_out_kd = n_left_circle.get_bi<float>("out_out", "kd");
    l_circle.out_out_ki = n_left_circle.get_bi<float>("out_out", "ki");

    //repair
    repair.kp = n_repair.get<float>("kp");
    repair.kd = n_repair.get<float>("kd");
    repair.dv = n_repair.get<float>("dv");
    repair.speed = n_repair.get<float>("speed");
    //farm
    farm.kp=n_farm.get<float>("kp");
    farm.kd=n_farm.get<float>("kd");
    farm.dv=n_farm.get<int>("dv");
    farm.speed=n_farm.get<float>("speed");
    farm.speed_out=n_farm.get<float>("speed_out");
    farm.dist=n_farm.get<int>("dist");
    farm.up_scope=n_farm.get<int>("up_scope");
    farm.down_scope=n_farm.get<int>("down_scope");
    farm.out_find_line=n_farm.get<int>("out_find_line");
    farm.out_find_count=n_farm.get<int>("out_find_count");
    farm.out_line=n_farm.get<int>("out_line");
    farm.out_count=n_farm.get<int>("out_count");
    //hump
    hump.kp=n_hump.get<float>("kp");
    hump.kd=n_hump.get<float>("kd");
    hump.dv=n_hump.get<int>("dv");
    hump.speed=n_hump.get<float>("speed");
    hump.mend_up_line=n_hump.get<int>("mend_up_line");
    hump.on_line=n_hump.get<int>("on_line");
    //start
    start.kp = n_start.get<float>("kp");
    start.kd = n_start.get<float>("kd");
    start.dv = n_start.get<int>("dv");
    start.left = n_start.get<bool>("left");
    start.start_dist = n_start.get<int>("start_dist");
    start.x_thresh = n_start.get<int>("x_thresh");
    start.v_left = n_start.get<pii>("v_left");
    start.v_right = n_start.get<pii>("v_right");
    start.start_speed = n_start.get<int>("start_speed");
    start.start_angle = n_start.get<int>("start_angle");
    //end
    end.end_dist = n_end.get<int>("end_dist");
    end.end_whitecount = n_end.get<int>("end_whitecount");
    end.v_left_garage = n_end.get<pii>("v_left_garage");
    end.v_right_garage = n_end.get<pii>("v_right_garage");  
    end.end_speed = n_end.get<int>("end_speed");
    end.end_angle = n_end.get<int>("end_angle");
    //zebra
    zebra.kp = n_zebra.get<float>("kp");
    zebra.kd = n_zebra.get<float>("kd");
    zebra.dv = n_zebra.get<int>("dv");
    zebra.zebra_far_dist = n_zebra.get<int>("zebra_far_dist");
    zebra.zebra_near_dist = n_zebra.get<int>("zebra_near_dist");
    zebra.v_left_zebra = n_zebra.get<pii>("v_left_zebra");
    zebra.v_right_zebra = n_zebra.get<pii>("v_right_zebra");
    //hill
    hill.kp = n_hill.get<float>("kp");
    hill.kd = n_hill.get<float>("kd");
    hill.dv = n_hill.get<int>("dv");
    hill.speed = n_hill.get<float>("speed");
    hill.mid_bot_y = n_hill.get<int>("mid_bot_y");
    hill.mid_top_y = n_hill.get<int>("mid_top_y");
    hill.frame = n_hill.get<int>("frame");

    //turn
    turn.turn_curvature_thresh = n_turn.get<float>("turn_curvature_thresh");
    turn.turn_deviation_thresh = n_turn.get<float>("turn_deviation_thresh");
    turn.turn_slope_thresh = n_turn.get<float>("turn_slope_thresh");
    turn.turn_out_slope_thresh = n_turn.get<float>("turn_out_slope_thresh");
    turn.inside_centerlost_thresh = n_turn.get<int>("inside_centerlost_thresh");
    turn.speed_ceiling = n_turn.get<float>("speed_ceiling");
    turn.speed_ground = n_turn.get<float>("speed_ground");
    turn.speed_in = n_turn.get<float>("speed_in");
    turn.motor_power = n_turn.get<float>("motor_power");
    turn.speed_ctrl_deviation_coef = n_turn.get<float>("speed_ctrl_deviation_coef");
    turn.speed_ctrl_slope_coef = n_turn.get<float>("speed_ctrl_slope_coef");
    turn.angle_ctrl_deviation_coef = n_turn.get<float>("angle_ctrl_deviation_coef");
    turn.angle_ctrl_slope_coef = n_turn.get<float>("angle_ctrl_slope_coef");
    turn.sc_kp = n_turn.get<float>("sc_kp");
    turn.sc_ki = n_turn.get<float>("sc_ki");    
    turn.sc_kd = n_turn.get<float>("sc_kd");    
    turn.dy_speed_bezier_p0_ctrl_x = n_turn.get<float>("dy_speed_bezier_p0_ctrl_x");    
    turn.dy_speed_bezier_p0_ctrl_y = n_turn.get<float>("dy_speed_bezier_p0_ctrl_y");
    turn.dy_speed_bezier_p1_ctrl_x = n_turn.get<float>("dy_speed_bezier_p1_ctrl_x");    
    turn.dy_speed_bezier_p1_ctrl_y = n_turn.get<float>("dy_speed_bezier_p1_ctrl_y");

    turn.slow_down_kd = n_turn.get<float>("slow_down_kd"); 
    turn.max_v_diff = n_turn.get<float>("max_v_diff");
    turn.min_v_diff = n_turn.get<float>("min_v_diff");

    turn.slowdown_enhance_bezier_p0_ctrl_x = n_turn.get<float>("slowdown_enhance_bezier_p0_ctrl_x");
    turn.slowdown_enhance_bezier_p0_ctrl_y = n_turn.get<float>("slowdown_enhance_bezier_p0_ctrl_y");
    turn.slowdown_enhance_bezier_p1_ctrl_x = n_turn.get<float>("slowdown_enhance_bezier_p1_ctrl_x");
    turn.slowdown_enhance_bezier_p1_ctrl_y = n_turn.get<float>("slowdown_enhance_bezier_p1_ctrl_y");
    turn.slowdown_smooth_bezier_p0_ctrl_x = n_turn.get<float>("slowdown_smooth_bezier_p0_ctrl_x");
    turn.slowdown_smooth_bezier_p0_ctrl_y = n_turn.get<float>("slowdown_smooth_bezier_p0_ctrl_y");
    turn.slowdown_smooth_bezier_p1_ctrl_x = n_turn.get<float>("slowdown_smooth_bezier_p1_ctrl_x");
    turn.slowdown_smooth_bezier_p1_ctrl_y = n_turn.get<float>("slowdown_smooth_bezier_p1_ctrl_y");
    turn.slow_down_smooth_thresh = n_turn.get<float>("slow_down_smooth_thresh");

    cone.kp = n_cone.get<float>("cone_kp");
    cone.kd = n_cone.get<float>("cone_kd");
    cone.ki = n_cone.get<float>("cone_ki");
    cone.dist = n_cone.get<int>("dist");
    cone.up_scope = n_cone.get<int>("up_scope");
    cone.down_scope = n_cone.get<int>("down_scope");
    cone.garage_dist = n_cone.get<int>("garage_dist");
    cone.judge_up_scope = n_cone.get<int>("judge_up_scope");
    cone.judge_down_scope = n_cone.get<int>("judge_down_scope");
    cone.b_cone_count = n_cone.get<int>("b_cone_count");
    cone.cone_state_count = n_cone.get<int>("cone_state_count");


    garage.slowdown_enhance_bezier_p0_ctrl_x = n_garage.get<float>("slowdown_enhance_bezier_p0_ctrl_x");
    garage.slowdown_enhance_bezier_p0_ctrl_y = n_garage.get<float>("slowdown_enhance_bezier_p0_ctrl_y");
    garage.slowdown_enhance_bezier_p1_ctrl_x = n_garage.get<float>("slowdown_enhance_bezier_p1_ctrl_x");
    garage.slowdown_enhance_bezier_p1_ctrl_y = n_garage.get<float>("slowdown_enhance_bezier_p1_ctrl_y");
    garage.slow_down_kd = n_garage.get<float>("slow_down_kd");
    garage.find_speed = n_garage.get<int>("find_speed");
    garage.out_speed = n_garage.get<int>("out_speed");
    garage.right_out_angle = n_garage.get<int>("right_out_angle");
    garage.left_out_angle = n_garage.get<int>("left_out_angle");
    garage.into_thresh = n_garage.get<int>("into_thresh");
    garage.top_y = n_garage.get<int>("top_y");
    garage.into_speed = n_garage.get<int>("into_speed");

}

struct Reader::pidv Reader::get_pidv(int state_out, int state_in) {
    float kp, kd, ki;
    int dv;
    switch (state_out)
    {
    case left_circle: 
    {
        switch (state_in)
        {
        case left_circle_out_out: {
            kp = n_left_circle.get_bi<float>("out_out", "kp");
            ki = n_left_circle.get_bi<float>("out_out", "ki");
            kd = n_left_circle.get_bi<float>("out_out", "kd");
            dv = n_left_circle.get_bi<int>("out_out", "dv");
            break;
        }
        case left_circle_out: {
            kp = n_left_circle.get_bi<float>("out", "kp");
            ki = n_left_circle.get_bi<float>("out", "ki");
            kd = n_left_circle.get_bi<float>("out", "kd");
            dv = n_left_circle.get_bi<int>("out", "dv");
            break;
        }
        case left_circle_out_strai: {
            kp = n_left_circle.get_bi<float>("out_strai", "kp");
            ki = n_left_circle.get_bi<float>("out_strai", "ki");
            kd = n_left_circle.get_bi<float>("out_strai", "kd");
            dv = n_left_circle.get_bi<int>("out_strai", "dv");
            break;
        }
        case left_circle_out_find: {
            kp = n_left_circle.get_bi<float>("out_find", "kp");
            ki = n_left_circle.get_bi<float>("out_find", "ki");
            kd = n_left_circle.get_bi<float>("out_find", "kd");
            dv = n_left_circle.get_bi<int>("out_find", "dv");
            break;
        }
        case left_circle_inside: {
            kp = n_left_circle.get_bi<float>("inside", "kp");
            ki = n_left_circle.get_bi<float>("inside", "ki");
            kd = n_left_circle.get_bi<float>("inside", "kd");
            dv = n_left_circle.get_bi<int>("inside", "dv");
            break;
        }
        case left_circle_inside_before: {
            kp = n_left_circle.get_bi<float>("inside_before", "kp");
            ki = n_left_circle.get_bi<float>("inside_before", "ki");
            kd = n_left_circle.get_bi<float>("inside_before", "kd");
            dv = n_left_circle.get_bi<int>("inside_before", "dv");
            break;
        }
        case left_circle_in_circle: {
            kp = n_left_circle.get_bi<float>("in_circle", "kp");
            ki = n_left_circle.get_bi<float>("in_circle", "ki");
            kd = n_left_circle.get_bi<float>("in_circle", "kd");
            dv = n_left_circle.get_bi<int>("in_circle", "dv");
            break;
        }
        case left_circle_in_strai: {
            kp = n_left_circle.get_bi<float>("in_strai", "kp");
            ki = n_left_circle.get_bi<float>("in_strai", "ki");
            kd = n_left_circle.get_bi<float>("in_strai", "kd");
            dv = n_left_circle.get_bi<int>("in_strai", "dv");
            break;
        }
        case left_circle_in_find: {
            kp = n_left_circle.get_bi<float>("in_find", "kp");
            ki = n_left_circle.get_bi<float>("in_find", "ki");
            kd = n_left_circle.get_bi<float>("in_find", "kd");
            dv = n_left_circle.get_bi<int>("in_find", "dv");
            break;
        }
        }
        break;
    }
    case right_circle: 
    {
        switch (state_in)
        {
        case right_circle_out_out: {
            kp = n_right_circle.get_bi<float>("out_out", "kp");
            ki = n_right_circle.get_bi<float>("out_out", "ki");
            kd = n_right_circle.get_bi<float>("out_out", "kd");
            dv = n_right_circle.get_bi<int>("out_out", "dv");
            break;
        }
        case right_circle_out: {
            kp = n_right_circle.get_bi<float>("out", "kp");
            ki = n_right_circle.get_bi<float>("out", "ki");
            kd = n_right_circle.get_bi<float>("out", "kd");
            dv = n_right_circle.get_bi<int>("out", "dv");
            break;
        }
        case right_circle_out_strai: {
            kp = n_right_circle.get_bi<float>("out_strai", "kp");
            ki = n_right_circle.get_bi<float>("out_strai", "ki");
            kd = n_right_circle.get_bi<float>("out_strai", "kd");
            dv = n_right_circle.get_bi<int>("out_strai", "dv");
            break;
        }
        case right_circle_out_find: {
            kp = n_right_circle.get_bi<float>("out_find", "kp");
            ki = n_right_circle.get_bi<float>("out_find", "ki");
            kd = n_right_circle.get_bi<float>("out_find", "kd");
            dv = n_right_circle.get_bi<int>("out_find", "dv");
            break;
        }
        case right_circle_inside: {
            kp = n_right_circle.get_bi<float>("inside", "kp");
            ki = n_right_circle.get_bi<float>("inside", "ki");
            kd = n_right_circle.get_bi<float>("inside", "kd");
            dv = n_right_circle.get_bi<int>("inside", "dv");
            break;
        }
        case right_circle_inside_before: {
            kp = n_right_circle.get_bi<float>("inside_before", "kp");
            ki = n_right_circle.get_bi<float>("inside_before", "ki");
            kd = n_right_circle.get_bi<float>("inside_before", "kd");
            dv = n_right_circle.get_bi<int>("inside_before", "dv");
            break;
        }
        case right_circle_in_circle: {
            kp = n_right_circle.get_bi<float>("in_circle", "kp");
            ki = n_right_circle.get_bi<float>("in_circle", "ki");
            kd = n_right_circle.get_bi<float>("in_circle", "kd");
            dv = n_right_circle.get_bi<int>("in_circle", "dv");
            break;
        }
        case right_circle_in_strai: {
            kp = n_right_circle.get_bi<float>("in_strai", "kp");
            ki = n_right_circle.get_bi<float>("in_strai", "ki");
            kd = n_right_circle.get_bi<float>("in_strai", "kd");
            dv = n_right_circle.get_bi<int>("in_strai", "dv");
            break;
        }
        case right_circle_in_find: {
            kp = n_right_circle.get_bi<float>("in_find", "kp");
            ki = n_right_circle.get_bi<float>("in_find", "ki");
            kd = n_right_circle.get_bi<float>("in_find", "kd");
            dv = n_right_circle.get_bi<int>("in_find", "dv");
            break;
        }
        }
        break;
    }
    default: 
    {
        kp = n_main.get<float>("kp");
        ki = n_main.get<float>("ki");
        kd = n_main.get<float>("kd");
        dv = n_main.get<int>("dv");
        break;
    }
    }
    struct pidv PIDV;
    PIDV.kp = kp;
    PIDV.ki = ki;
    PIDV.kd = kd;
    PIDV.dv = dv;
    return PIDV;
}
