#ifndef PID
#define PID

// 引入 uint_8
//#include <bits/stdint-uintn.h>
#include <cmath>
#include <iostream>

// 使用 float 作为输入、误差类型
typedef float data_t;
// 使用 float 作为输出类型
typedef float out_t;
extern float cur_kp;
extern float deviation;

class PartPdCtrl {
private:
	float kp;//读取
	float kd;//读取
	float ki;//读取
	data_t last_error;//上一偏差
	int count;//偏差过大时改kp
public:
	PartPdCtrl(float kp_ = 0, float kd_ = 0, float ki_ = 0);
	data_t output(data_t error);
	void reset(float kp, float kd, float ki) { this->kp = kp; this->kd = kd; this->ki = ki;}
  float show(){return this->kp;}
};


/**
 * 舵机角度的控制，基于两部分：
 * 小车方向与中线之间的夹角，目标是使其为0，避免左右乱摆;
 * 小车与中线之间的距离，目标是使其为0，避免偏离赛道
 */
class AngleControl {
private:
	PartPdCtrl pid;
	data_t maximum;	// 输出的最大值
	data_t minimum;	// 输出的最小值

public:
	//@param kp 比例系数	@param kd 微分系数	@param a_c 夹角误差系数	@param d_c 距离误差系数	@param max 输出最大值	@param min 输出最小值
	AngleControl(float kp, float kd, float ki, data_t max, data_t min);
	/**
	 * @brief 计算输出值	
	 * @param dist_error 小车与中线之间的距离	
	 * @return out_t 输出值，可直接发至下位机
	*/
	out_t output(data_t dist_error);
	void reset(float kp, float kd, float ki) { this->pid.reset(kp, kd, ki); }
  	float show(){return this->pid.show();}
};

// 线速度控制
class SpeedControl {
private:

	data_t start_error;//最小允许误差
	data_t end_error;//最大允许误差
	float k;		// 速度的比例系数，斜坡
	data_t minimum;// 输出的最小值	
	data_t maximum;// 输出的最大值

	float p0_x;float p0_y;float p1_x;float p1_y;
	float p0_ctrl_x;float p0_ctrl_y;float p1_ctrl_x;float p1_ctrl_y;
public:
	//@param k 速度的比例系数	@param max 输出的最大值	@param min 输出的最小值
	SpeedControl(data_t start_error, data_t end_error, data_t max, data_t min);
	/**
	 * @brief 速度控制
	 * @param input 输入中线偏离deviation
	 * @return float 返回速度绝对值
	 * @note 根据偏移量来决定速度，在范围内偏移量越大速度越小，并随偏移量增大减速幅度增加
	*/
	out_t output(data_t input);
	float bezier_get_t(float t_head,float t_tail,float x);
};

#endif
