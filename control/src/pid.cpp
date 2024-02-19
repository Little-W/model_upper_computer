#include "pid.h"
#include<iostream>
#include "process.h"
using namespace std;

float cur_kp;

PartPdCtrl::PartPdCtrl(float kp_, float kd_, float ki_) {
	this->last_error = 0;
	this->kp = kp_;
	this->kd = kd_;
	this->ki = ki_;
	this->count = 0;
}

data_t PartPdCtrl::output(data_t error) {
	static data_t integrade;
	data_t now_out_diff;

	if (abs(error) > Re.main.dy_kd_threshold) {
		this->count++;
		// out += (1 + count * 0.05) * kp * error;//偏差过大增大kp
		now_out_diff += (1 + count * Re.main.dy_kd_coef) * kd * (error - last_error);//偏差过大增大kp
	}
	else {
		this->count = 0;
		now_out_diff = kd * (error - last_error);//微分部分
	}

	
	// now_out_diff = kd * (error - last_error);//微分部分
	data_t out;
	last_error = error;
	out = now_out_diff;
	integrade = integrade * 0.9 + error * 0.1;
	out += ki * integrade;
	// if (abs(error) > 62) {
	if (abs(error) > Re.main.dy_kp_threshold) {
		cur_kp = Re.main.dy_kp_coef * pow(abs(error),Re.main.dy_kp_exp) + kp;
		if(cur_kp > 15)
		{
			cur_kp = 15;
		}
		// out += (1 + count * 0.05) * kp * error;//偏差过大增大kp
		out +=  cur_kp * error;//偏差过大增大kp
	}
	else {
		cur_kp = kp;
		out += kp * error;//微分+比例部分，out=kd*(error-lasterror)+(kp - delta)*error
	}

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
SpeedControl::SpeedControl(data_t start_error_, data_t end_error_, data_t max_, data_t min_) {
	this->start_error = start_error_;
	this->end_error = end_error_;
	this->k = float(max_ - min_) / (end_error_ - start_error_);
	this->maximum = max_;
	this->minimum = min_;

	this->p0_x = start_error_;
	this->p0_y = max_;
	this->p1_x = end_error_;
	this->p1_y = min_;
	this->p0_ctrl_x = Re.main.dy_speed_bezier_p0_ctrl_x;
	this->p0_ctrl_y = Re.main.dy_speed_bezier_p0_ctrl_y;
	this->p1_ctrl_x = Re.main.dy_speed_bezier_p1_ctrl_x;
	this->p1_ctrl_y = Re.main.dy_speed_bezier_p1_ctrl_y;
}
//二分查找解出三阶贝塞尔曲线当前x对应的参数t
float SpeedControl::bezier_get_t(float x, float t_head, float t_tail) {
	float t = (t_head + t_tail) / 2.0;
	float t2x = (pow(1.0 - t, 3) * p0_x) + (3.0 * t * pow(1.0 - t, 2) * p0_ctrl_x) + (3.0 * pow(t, 2) * (1 - t) * p1_ctrl_x) + (pow(t, 3) * p1_x);
	//可接受误差范围内返回结果
	if (abs(t2x - x) < 0.001) return t;
	if (t2x < x) return bezier_get_t(x,t, t_tail);
	return bezier_get_t(x,t_head, t);
}
float limit2range(float target,float head,float tail){
	target = max(head,target);
	return min(tail,target);
}

out_t SpeedControl::output(data_t input) {
	input = abs(input);
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
		p0_ctrl_x = limit2range(p0_ctrl_x,start_error,end_error);
		p1_ctrl_x = limit2range(p1_ctrl_x,start_error,end_error);
		//限制p1_ctrl_x不小于p0_ctrl_x
		p1_ctrl_x = max(p0_ctrl_x,p1_ctrl_x);

		p0_ctrl_y = limit2range(p0_ctrl_y,minimum,maximum);
		p1_ctrl_y = limit2range(p1_ctrl_y,minimum,maximum);
		//限制p1_ctrl_y不超过p0_ctrl_y
		p1_ctrl_y = min(p0_ctrl_y,p1_ctrl_y);

		float t = bezier_get_t(input,0,1);
		out = (pow(1.0 - t, 3) * p0_y) + (3.0 * t * pow(1.0 - t, 2) * p0_ctrl_y) + (3.0 * pow(t, 2) * (1 - t) * p1_ctrl_y) + (pow(t, 3) * p1_y);
	}
	return out_t(out);
}