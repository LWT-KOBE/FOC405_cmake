
#include "MyProject.h"

/****************************************************************************/
bool anticogging_valid_ = false;
Anticogging_t  anticogging;
/****************************************************************************/
void anticogging_init(void)
{
	anticogging.index = 0;
	anticogging.pre_calibrated = false;
	anticogging.calib_anticogging = false;
	anticogging.calib_pos_threshold = ANTIcogging_pos_threshold;
	anticogging.calib_vel_threshold = ANTIcogging_vel_threshold;
	anticogging.cogging_ratio = 1.0f;    //没用到
	anticogging.anticogging_enabled = true;
}
/****************************************************************************/
void start_anticogging_calibration(void)
{
	if(motor_error == ERROR_NONE)
	{
		anticogging.index = 0;
		anticogging.calib_anticogging = true;
	}
}
/****************************************************************************/
float remove_anticogging_bias(void)
{
	uint32_t  i;
	float sum=0;
	float average;
	
	for(i=0; i<COG_num; i++)
	{
		sum += anticogging.cogging_map[i];
	}
	average = sum / COG_num;
	
	for(i=0; i<COG_num; i++)
	{
		anticogging.cogging_map[i] = anticogging.cogging_map[i] - average;
	}
	
	return average;
}
/****************************************************************************/
/*
 * This anti-cogging implementation iterates through each encoder position,
 * waits for zero velocity & position error,
 * then samples the current required to maintain that position.
 * 
 * This holding current is added as a feedforward term in the control loop.

   以下代码移植自ODrive v0.5.6 controller.cpp文件 第79行
 */
bool anticogging_calibration(float pos_estimate, float vel_estimate)
{
	float pos_err = input_pos_ - pos_estimate;
	if ((fabsf(pos_err) <= anticogging.calib_pos_threshold / (float)encoder_config.cpr) && (fabsf(vel_estimate) < anticogging.calib_vel_threshold / (float)encoder_config.cpr))
	{
		anticogging.cogging_map[anticogging.index] = vel_integrator_torque_;
		if(++anticogging.index > COG_num)anticogging.index = COG_num;
	}
	if (anticogging.index < COG_num)
	{
		ctrl_config.control_mode = CONTROL_MODE_POSITION_CONTROL;
		ctrl_config.input_mode = INPUT_MODE_PASSTHROUGH;    //设置为直通模式，否则电机转不到位
		input_pos_ = (float)anticogging.index / COG_num;
		input_vel_ = 0.0f;
		input_torque_ = 0.0f;
		input_pos_updated_ = true;
		return false;
	}
	else
	{
		//remove_anticogging_bias();   //校准后的数据优化处理。0.5.1和0.5.6都没有调用，多一事不如少一事
		anticogging.index = 0;
		ctrl_config.control_mode = CONTROL_MODE_POSITION_CONTROL;
		ctrl_config.input_mode = INPUT_mode;    //校准完成后恢复输入模式
		input_pos_ = 0.0f;  // Send the motor home
		input_vel_ = 0.0f;
		input_torque_ = 0.0f;
		input_pos_updated_ = true;
		anticogging_valid_ = true;
		anticogging.calib_anticogging = false;
		return true;
	}
}
/****************************************************************************/




