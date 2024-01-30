#include "pid.h"
#include<iostream>
#include "process.h"
using namespace std;

#define Re MI.re
extern MainImage MI;

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
		this->count++;
		// out += (1 + count * 0.05) * kp * error;//偏差过大增大kp
		out += (1 + count * Re.main.dy_kp_coef) * kp * error;//偏差过大增大kp
	}
	else {
		this->count = 0;
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
 *返回根据PID的输出值
 * 输入角度
 * @return uint8_t 输出量，未映射，已限制范围，可直接发送到下位机
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
}


/**
 * @brief 速度控制
 * @param input 输入中线偏离
 * @return unsigned char 返回速度绝对值
 * @note 根据偏移量来决定速度，偏移量越大速度越小，范围为[start_error,end_error]
 */
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
		out = maximum - k * (input - start_error);
	}
	return out_t(out);
}