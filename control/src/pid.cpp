#include "pid.h"
#include<iostream>
using namespace std;
PartPdCtrl::PartPdCtrl(float kp_, float kd_) {
	this->last_error = 0;
	this->kp = kp_;
	this->kd = kd_;
	this->count = 0;
}

data_t PartPdCtrl::output(data_t error) {
	data_t now_out_diff;
	now_out_diff = kd * (error - last_error);//΢�ֲ���
	data_t out;
	last_error = error;
	out = now_out_diff;

	if (abs(error) > 62) {
		this->count++;
		out += (1 + count * 0.05) * kp * error;//ƫ���������kp
	}
	else {
		this->count = 0;
		out += kp * error;//΢��+�������֣�out=kd*(error-lasterror)+(kp - delta)*error
	}

	return out;
}

// AngleControl ��

AngleControl::AngleControl(float kp, float kd, data_t max, data_t min) {
	this->pid = PartPdCtrl(kp, kd);
	this->maximum = max;
	this->minimum = min;
}
/**
 *���ظ���PID�����ֵ
 * ����Ƕ�
 * @return uint8_t �������δӳ�䣬�����Ʒ�Χ����ֱ�ӷ��͵���λ��
 */
out_t AngleControl::output(data_t dist_error) {
	data_t now_error = dist_error;
	int out = round(this->pid.output(now_error)) + 125;//125:�������ֵ
	if (out > this->maximum) out = this->maximum;
	else if (out < this->minimum) out = this->minimum;
	return out_t(out);
}

// SpeedControl ��
SpeedControl::SpeedControl(data_t start_error_, data_t end_error_, data_t max_, data_t min_) {
	this->start_error = start_error_;
	this->end_error = end_error_;
	this->k = float(max_ - min_) / (end_error_ - start_error_);
	this->maximum = max_;
	this->minimum = min_;
}
//��������ƫ��
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