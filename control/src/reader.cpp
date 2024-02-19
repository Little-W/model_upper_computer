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
    main.enc_forward_threshold = n_main.get<int>("enc_forward_threshold");
    main.enc_forward_dist_coef = n_main.get<float>("enc_forward_dist_coef");
    main.enc_forward_dist_exp = n_main.get<float>("enc_forward_dist_exp");
    main.max_enc_forward_dist = n_main.get<int>("max_enc_forward_dist");
    main.enc_up_scope_coef = n_main.get<float>("enc_up_scope_coef");
    main.enc_up_scope_exp = n_main.get<float>("enc_up_scope_exp");
    main.max_enc_up_scope = n_main.get<int>("max_enc_up_scope");
    main.enc_forward_coef1_coef = n_main.get<float>("enc_forward_coef1_coef");
    main.enc_forward_coef1_exp = n_main.get<float>("enc_forward_coef1_exp");
    main.max_enc_forward_coef1 = n_main.get<float>("max_enc_forward_coef1");
    main.dy_kp_threshold = n_main.get<float>("dy_kp_threshold");
    main.dy_kp_coef = n_main.get<float>("dy_kp_coef");
    main.dy_kp_exp = n_main.get<float>("dy_kp_exp");
    main.dy_kp_max = n_main.get<float>("dy_kp_max");
    main.dy_kd_threshold = n_main.get<float>("dy_kd_threshold");
    main.dy_kd_coef = n_main.get<float>("dy_kd_coef");
    main.dy_speed_coef = n_main.get<float>("dy_speed_coef");
    main.dy_speed_exp = n_main.get<float>("dy_speed_exp");
    main.dy_speed_bezier_p0_ctrl_x = n_main.get<float>("dy_speed_bezier_p0_ctrl_x");
    main.dy_speed_bezier_p0_ctrl_y = n_main.get<float>("dy_speed_bezier_p0_ctrl_y");
    main.dy_speed_bezier_p1_ctrl_x = n_main.get<float>("dy_speed_bezier_p1_ctrl_x");
    main.dy_speed_bezier_p1_ctrl_y = n_main.get<float>("dy_speed_bezier_p1_ctrl_y")    ;
    
    main.kp = n_main.get<float>("kp");
    main.kd = n_main.get<float>("kd");
    main.ki = n_main.get<float>("ki");
    main.dv = n_main.get<int>("dv");
    main.max_ag = n_main.get<int>("max_ag");
    main.min_ag = n_main.get<int>("min_ag");
    main.max_v = n_main.get<int>("max_v");
    main.min_v = n_main.get<int>("min_v");
    main.max_v_diff = n_main.get<int>("max_v_diff");
    main.min_v_diff = n_main.get<int>("min_v_diff");
    main.use_dv = n_main.get<bool>("use_dv");
    main.garage_start = n_main.get<bool>("garage_start");
    main.second_lap = n_main.get<bool>("second_lap");
    main.center_coef = n_main.get<float>("center_coef");
    main.forward_dist = n_main.get<int>("forward_dist");
    main.up_scope = n_main.get<int>("up_scope");
    main.down_scope = n_main.get<int>("down_scope");
    main.forward_coef1 = n_main.get<float>("forward_coef1");
    main.forward_coef2 = n_main.get<float>("forward_coef2");
    main.v_right_circle = n_main.get<pii>("v_right_circle");
    main.v_left_circle = n_main.get<pii>("v_left_circle");
    main.left_ray = n_main.get<float>("left_ray");
    main.right_ray = n_main.get<float>("right_ray");

    //right_circle
    r_circle.kp = n_right_circle.get<float>("kp");
    r_circle.kd = n_right_circle.get<float>("kd");
    r_circle.dv = n_right_circle.get<int>("dv");
    r_circle.speed = n_right_circle.get<int>("speed");
    r_circle.circle_dist = n_right_circle.get<int>("circle_dist");
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
    //left_circle
    l_circle.kp = n_left_circle.get<float>("kp");
    l_circle.kd = n_left_circle.get<float>("kd");
    l_circle.dv = n_left_circle.get<int>("dv");
    l_circle.speed = n_left_circle.get<int>("speed");
    l_circle.circle_dist = n_left_circle.get<int>("circle_dist");
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
    //repair
    repair.kp = n_repair.get<float>("kp");
    repair.kd = n_repair.get<float>("kd");
    repair.dv = n_repair.get<float>("dv");
    repair.speed = n_repair.get<int>("speed");
    //farm
    farm.kp=n_farm.get<float>("kp");
    farm.kd=n_farm.get<float>("kd");
    farm.dv=n_farm.get<int>("dv");
    farm.speed=n_farm.get<int>("speed");
    farm.speed_out=n_farm.get<int>("speed_out");
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
    hump.speed=n_hump.get<int>("speed");
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
    //end
    end.end_dist = n_end.get<int>("end_dist");
    end.end_whitecount = n_end.get<int>("end_whitecount");
    end.v_left_garage = n_end.get<pii>("v_left_garage");
    end.v_right_garage = n_end.get<pii>("v_right_garage");  
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
    hill.speed = n_hill.get<int>("speed");
    hill.mid_bot_y = n_hill.get<int>("mid_bot_y");
    hill.mid_top_y = n_hill.get<int>("mid_top_y");
    hill.frame = n_hill.get<int>("frame");
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
