#ifndef PID
#define PID

// ���� uint_8
//#include <bits/stdint-uintn.h>
#include <cmath>
#include <iostream>

// ʹ�� float ��Ϊ���롢�������
typedef float data_t;
// ʹ�� uint8 ��Ϊ�������
typedef unsigned char out_t;

class PartPdCtrl {
private:
	float kp;//��ȡ
	float kd;//��ȡ
	data_t last_error;//��һƫ��
	int count;//ƫ�����ʱ��kp
public:
	PartPdCtrl(float kp_ = 0, float kd_ = 0);
	data_t output(data_t error);
	void reset(float kp, float kd) { this->kp = kp; this->kd = kd; }
  float show(){return this->kp;}
};


/**
 * ����ǶȵĿ��ƣ����������֣�
 * С������������֮��ļнǣ�Ŀ����ʹ��Ϊ0�����������Ұ�;
 * С��������֮��ľ��룬Ŀ����ʹ��Ϊ0������ƫ������
 */
class AngleControl {
private:
	PartPdCtrl pid;
	data_t maximum;	// ��������ֵ
	data_t minimum;	// �������Сֵ

public:
	//@param kp ����ϵ��	@param kd ΢��ϵ��	@param a_c �н����ϵ��	@param d_c �������ϵ��	@param max ������ֵ	@param min �����Сֵ
	AngleControl(float kp, float kd, data_t max, data_t min);
	//@brief �������ֵ	@param angel_error С������������֮��ļн�	@param dist_error С��������֮��ľ���	@return out_t ���ֵ����ֱ�ӷ�����λ��
	out_t output(data_t dist_error);
	void reset(float kp, float kd) { this->pid.reset(kp, kd); }
  float show(){return this->pid.show();}
};

// ���ٶȿ���
class SpeedControl {
private:

	data_t start_error;//��С�������
	data_t end_error;//����������
	float k;		// �ٶȵı���ϵ����б��
	data_t minimum;// �������Сֵ	
	data_t maximum;// ��������ֵ
public:
	//@param k �ٶȵı���ϵ��	@param max ��������ֵ	@param min �������Сֵ
	SpeedControl(data_t start_error, data_t end_error, data_t max, data_t min);
	//@brief �������ֵ	@param input ��@return out_t ���ֵ����ֱ�ӷ�����λ��
	out_t output(data_t input);
};

#endif
