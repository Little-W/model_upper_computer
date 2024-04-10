#include "pid.h"
#include<iostream>
#include "process.h"
#include "comm.h"
using namespace std;

float cur_kp;

PartPdCtrl::PartPdCtrl(float kp_, float kd_, float ki_) {	
	this->kp = kp_;
	this->kd = kd_;
	this->ki = ki_;
	this->count = 0;
	this->last_error = 0;
	this->integrade = 0;
	this->dy_kp_bias = 0;
}
data_t PartPdCtrl::dynamic_pid(data_t error){
	data_t now_out_diff;

	//kd部分计算
	if (abs(error) > Re.main.dy_kd_threshold) {
		count++;
		// out += (1 + count * 0.05) * kp * error;//偏差过大增大kp
		now_out_diff += (1 + count * Re.main.dy_kd_coef) * kd * (error - last_error);//偏差过大增大kd
	}
	else {
		count = 0;
		now_out_diff = kd * (error - last_error);//微分部分
	}

	
	// now_out_diff = kd * (error - last_error);//微分部分
	data_t out;
	
	out = now_out_diff;
	integrade = integrade * 0.9 + error * 0.1;
	out += ki * integrade;

	// 位置式动态kp
	if (abs(error) > Re.main.dy_kp_threshold) {
		cur_kp = Re.main.dy_kp_coef / 100000 * pow(abs(error),Re.main.dy_kp_exp) + kp;
		// out += (1 + count * 0.05) * kp * error;//偏差过大增大kp
		//偏差过大增大kp
	}
	else {
		cur_kp = kp;
	}

	// 增量式动态kp
	if (abs(error) > Re.main.inc_dy_kp_threshold) {
		dy_kp_bias += Re.main.inc_dy_kp_coef / 100000 * pow(abs(error),Re.main.inc_dy_kp_exp);
		cur_kp += dy_kp_bias;
	}
	else {
		dy_kp_bias = 0;
	}

	if(cur_kp > Re.main.dy_kp_max)
	{
		cur_kp = Re.main.dy_kp_max;
	}

	out += cur_kp * error;
	return out;
}
data_t PartPdCtrl::output(data_t error) {
	data_t out = 0;
	if(MI.state_out == right_circle || MI.state_out == left_circle)
	{
		out = kp * error + kd * (error - last_error);
		cur_kp = kp;
	}
	else
	{
		out = dynamic_pid(error);
	}
	last_error = error;
	return out;
}

// AngleControl 类

AngleControl::AngleControl(float kp, float kd, float ki, data_t max, data_t min) {
	this->pid = PartPdCtrl(kp, kd, ki);
	this->maximum = max;
	this->minimum = min;
}
/**
 * 返回根据PID的输出值
 * 输入角度
 * @return float 输出量，未映射，已限制范围，可直接发送到下位机
 */
out_t AngleControl::output(data_t dist_error) {
	data_t now_error = dist_error;
	int out = round(this->pid.output(now_error));  //舵机中线值为0，左转为正，右转为负
	if (out > this->maximum) out = this->maximum;
	else if (out < this->minimum) out = this->minimum;
	return out_t(out);
}

// SpeedControl 类
// 对应config.yaml: Re.main.min_v_diff, Re.main.max_v_diff, Re.main.max_v, Re.main.min_v
SpeedControl::SpeedControl(data_t _start_error, data_t _end_error, data_t _max, data_t _min,
							float _kp ,float _ki ,float _kd,
							float bezier_p0_ctrl_x,float bezier_p0_ctrl_y,float bezier_p1_ctrl_x,float bezier_p1_ctrl_y,
							float slow_down_enhance_bezier_p0_ctrl_x,float slow_down_enhance_bezier_p0_ctrl_y,
							float slow_down_enhance_bezier_p1_ctrl_x,float slow_down_enhance_bezier_p1_ctrl_y,
							float slow_smooth_bezier_p0_ctrl_x,float slow_smooth_bezier_p0_ctrl_y,
							float slow_smooth_bezier_p1_ctrl_x,float slow_smooth_bezier_p1_ctrl_y
							)
{
	this->start_error = _start_error;
	this->end_error = _end_error;
	this->k = float(_max - _min) / (_end_error - _start_error);
	this->maximum = _max;
	this->minimum = _min;
	
	this->dy_speed_bezier_p0_ctrl_x = bezier_p0_ctrl_x;
	this->dy_speed_bezier_p1_ctrl_x = bezier_p1_ctrl_x;
	this->dy_speed_bezier_p0_ctrl_y = bezier_p0_ctrl_y;
	this->dy_speed_bezier_p1_ctrl_y = bezier_p1_ctrl_y;

	this->kp = _kp;
	this->ki = _ki;
	this->kd = _kd;
	this->last_error = 0;
	this->integrade = 0;

	this->speed_slow_down_enhance_bezier_p0_ctrl_x = slow_down_enhance_bezier_p0_ctrl_x;
	this->speed_slow_down_enhance_bezier_p1_ctrl_x = slow_down_enhance_bezier_p1_ctrl_x;
	this->speed_slow_down_enhance_bezier_p0_ctrl_y = slow_down_enhance_bezier_p0_ctrl_y;
	this->speed_slow_down_enhance_bezier_p1_ctrl_y = slow_down_enhance_bezier_p1_ctrl_y;

	this->speed_slow_down_smooth_bezier_p0_ctrl_x = slow_smooth_bezier_p0_ctrl_x;
	this->speed_slow_down_smooth_bezier_p1_ctrl_x = slow_smooth_bezier_p1_ctrl_x;
	this->speed_slow_down_smooth_bezier_p0_ctrl_y = slow_smooth_bezier_p0_ctrl_y;
	this->speed_slow_down_smooth_bezier_p1_ctrl_y = slow_smooth_bezier_p1_ctrl_y;	
}
//二分查找解出三阶贝塞尔曲线当前x对应的参数t
float SpeedControl::bezier_get_t(float x, float t_head, float t_tail,float p0_x,float p0_ctrl_x,float p1_ctrl_x,float p1_x) {
	float t = (t_head + t_tail) / 2.0;
	float t2x = (pow(1.0 - t, 3) * p0_x) + (3.0 * t * pow(1.0 - t, 2) * p0_ctrl_x) + (3.0 * pow(t, 2) * (1 - t) * p1_ctrl_x) + (pow(t, 3) * p1_x);
	//可接受误差范围内返回结果
	if (abs(t2x - x) < 0.001) return t;
	if (t2x < x) return bezier_get_t(x,t, t_tail,p0_x,p0_ctrl_x,p1_ctrl_x,p1_x);
	return bezier_get_t(x,t_head, t,p0_x,p0_ctrl_x,p1_ctrl_x,p1_x);
}
float limit2range(float target,float head,float tail){
	target = max(head,target);
	return min(tail,target);
}

data_t SpeedControl::pid_ctrl(data_t error){
	float out = kp * error + kd * (error - last_error);

	integrade = integrade * 0.9 + error * 0.1;
	out += ki * integrade;
	
	last_error = error;
	return out;
}
out_t SpeedControl::output(data_t input) {
	input = abs(pid_ctrl(input));
	data_t out;
	if (input < start_error) {
		out = maximum;
	}
	else if (input > end_error) {
		out = minimum;
	}
	else {
		//18th使用线性变化，过弯时减速效果并不好
		//this->k = float(max_ - min_) / (end_error_ - start_error_);
		// out = maximum - k * (input - start_error);

		//幂函数deviation-speed曲线
		//out = maximum - dy_speed_coef * pow(input - start_error,dy_speed_exp);

		//为保证曲线在边界处连续，并且可以更加自由调节曲线形状，
		//使用三阶贝塞尔曲线限制条件来拟合一条斜率逐渐增加的deviation-speed曲线
		//限制范围，确保配置文件中赋值符合规定
		float p0_x = start_error ,p0_y = maximum;
		float p1_x = end_error ,p1_y = minimum;
		// float p0_ctrl_x = limit2range(dy_speed_bezier_p0_ctrl_x,start_error,end_error);
		// float p1_ctrl_x = limit2range(dy_speed_bezier_p1_ctrl_x,start_error,end_error);
		// //限制p1_ctrl_x不小于p0_ctrl_x
		// p1_ctrl_x = max(p0_ctrl_x,p1_ctrl_x);

		// float p0_ctrl_y = limit2range(dy_speed_bezier_p0_ctrl_y,minimum,maximum);
		// float p1_ctrl_y = limit2range(dy_speed_bezier_p1_ctrl_y,minimum,maximum);
		// //限制p1_ctrl_y不超过p0_ctrl_y
		// p1_ctrl_y = min(p0_ctrl_y,p1_ctrl_y);
		float p0_ctrl_x = dy_speed_bezier_p0_ctrl_x ,p0_ctrl_y = dy_speed_bezier_p0_ctrl_y;
		float p1_ctrl_x = dy_speed_bezier_p1_ctrl_x ,p1_ctrl_y = dy_speed_bezier_p1_ctrl_y;


		float t = bezier_get_t(input,0,1,p0_x,p0_ctrl_x,p1_ctrl_x,p1_x);
		out = (pow(1.0 - t, 3) * p0_y) + (3.0 * t * pow(1.0 - t, 2) * p0_ctrl_y) + (3.0 * pow(t, 2) * (1 - t) * p1_ctrl_y) + (pow(t, 3) * p1_y);
	}
	return out_t(out);
}
/// @brief 提升减速性能，只有在减速时起作用
/// @param speed_result 计算得到的目标速度
/// @param real_speed_enc 下位机传来的当前真实速度
/// @return speed_result
out_t SpeedControl::output_reduced(data_t speed_result,data_t real_speed_enc,float slow_down_kd){
	// 非减速状态，返回原值
	if(real_speed_enc - (speed_result * ENC_SPEED_SCALE) < 20)
	{
		return speed_result;
	} 
	// 初始化边界值和控制点
	float p0_x = 0 ,p0_y = 0;
	float p1_x = maximum * ENC_SPEED_SCALE ,p1_y = 100;
	float p0_ctrl_x = speed_slow_down_enhance_bezier_p0_ctrl_x ,p0_ctrl_y = speed_slow_down_enhance_bezier_p0_ctrl_y;
	float p1_ctrl_x = speed_slow_down_enhance_bezier_p1_ctrl_x ,p1_ctrl_y = speed_slow_down_enhance_bezier_p1_ctrl_y;

	float bezier_x = abs(real_speed_enc) > p1_x ? p1_x : abs(real_speed_enc);
	// cerr << "p1" << endl;
	float bezier_t = bezier_get_t(bezier_x,0,1,p0_x,p0_ctrl_x,p1_ctrl_x,p1_x);

	// cerr << "p2" << endl;
	float bezier_out = (pow(1.0 - bezier_t, 3) * p0_y) + (3.0 * bezier_t * pow(1.0 - bezier_t, 2) * p0_ctrl_y) + (3.0 * pow(bezier_t, 2) * (1 - bezier_t) * p1_ctrl_y) + (pow(bezier_t, 3) * p1_y);

	float speed_bias = bezier_out * slow_down_kd / 100.0 * (real_speed_enc - speed_result * ENC_SPEED_SCALE) / ENC_SPEED_SCALE;

	if(speed_bias < 0)
	{
		speed_bias = 0;
	}
	return data_t(speed_result - speed_bias);
}

out_t SpeedControl::slow_down_smooth(data_t speed_result,data_t real_speed_enc,float slow_down_smooth_thresh)
{
	data_t out;
	// 非减速状态，返回原值
	if(real_speed_enc <= speed_result * ENC_SPEED_SCALE){
		return speed_result;
	} 
	// 初始化边界值和控制点
	float p0_x = 0 ,p0_y = 0;
	float p1_x = maximum * ENC_SPEED_SCALE ,p1_y = 1;
	float p0_ctrl_x = speed_slow_down_smooth_bezier_p0_ctrl_x ,p0_ctrl_y = speed_slow_down_smooth_bezier_p0_ctrl_y;
	float p1_ctrl_x = speed_slow_down_smooth_bezier_p1_ctrl_x ,p1_ctrl_y = speed_slow_down_smooth_bezier_p1_ctrl_y;

	float bezier_x = abs(real_speed_enc) > p1_x ? p1_x : abs(real_speed_enc);
	float bezier_t = bezier_get_t(bezier_x,0,1,p0_x,p0_ctrl_x,p1_ctrl_x,p1_x);

	float bezier_out = (pow(1.0 - bezier_t, 3) * p0_y) + (3.0 * bezier_t * pow(1.0 - bezier_t, 2) * p0_ctrl_y) + (3.0 * pow(bezier_t, 2) * (1 - bezier_t) * p1_ctrl_y) + (pow(bezier_t, 3) * p1_y);

	float speed_bias = bezier_out * slow_down_smooth_thresh;

	out = real_speed_enc / ENC_SPEED_SCALE - speed_bias;

	if(out < speed_result)
	{
		out = speed_result;
	}
	return out;
}