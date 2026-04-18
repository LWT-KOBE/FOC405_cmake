
#include "MyProject.h"


/****************************************************************************/
MOTOR_CONFIG  motor_config;

bool  is_armed_ = false;
bool  is_calibrated_ = false; // Set in apply_config()
uint8_t armed_state_ = 0;
Iph_ABC_t  current_meas_;
Iph_ABC_t  DC_calib_;
float dc_calib_running_since_ = 0.0f;

float effective_current_lim_ = 10.0f; // [A]
float max_allowed_current_ = 0.0f;    // [A] set in setup()
float max_dc_calib_ = 0.0f;           // [A] set in setup()

float *motor_torque_setpoint_src_; // Usually points to the Controller object's output
float *motor_phase_vel_src_;       // Usually points to the Encoder object's output
float2D  motor_Vdq_setpoint_;
float2D  motor_Idq_setpoint_;

bool  meas_resis = false;   //当前正在测量电阻
bool  meas_induc = false;   //当前正在测量电感
/****************************************************************************/
void arm(void);
void disarm(void);
/****************************************************************************/
/****************************************************************************/
//参数初始化，官方代码中，直接在定义的时候赋值
void motor_para_init(void)
{
	motor_config.pre_calibrated = false; // can be set to true to indicate that all values here are valid
	motor_config.pole_pairs = MOTOR_pole_pairs;  //电机极对数，参数设置宏定义在MyProject.h文件中
	motor_config.calibration_current = MOTOR_calibration_current;    // [A]
	motor_config.resistance_calib_max_voltage = MOTOR_resistance_calib_max_voltage; // [V] - You may need to increase this if this voltage isn't sufficient to drive calibration_current through the motor.
	motor_config.phase_inductance = 0.0f;        // to be set by measure_phase_inductance
	motor_config.phase_resistance = 0.0f;        // to be set by measure_phase_resistance
	motor_config.torque_constant = 0.04f;        // [Nm/A] for PM motors, [Nm/A^2] for induction motors. Equal to 8.27/Kv of the motor
	motor_config.motor_type = MOTOR_type;  
	// Read out max_allowed_current to see max supported value for current_lim.
	motor_config.current_lim = MOTOR_current_lim; //[A] 电机最大运行电流
	motor_config.current_lim_margin = 8.0f;      // Maximum violation of current_lim
//	motor_config.torque_lim = std::numeric_limits<float>::infinity();           //[Nm]. 
	// Value used to compute shunt amplifier gains
	//float requested_current_range = 60.0f; //  [A]1mΩ采样电阻对应60A，0.5mΩ采样电阻对应120A
	// motor_config.current_control_bandwidth = 1000.0f;  // [rad/s]
	// motor_config.current_control_bandwidth = 1000.0f;  // [rad/s] //修改

	motor_config.current_control_bandwidth = CONTROL_CURRENT_CONTROL_BANDWIDTH;  // [rad/s] //修改
	motor_config.inverter_temp_limit_lower = 100;
	motor_config.inverter_temp_limit_upper = 120;
	
	motor_config.R_wL_FF_enable = false; // Enable feedforwards for R*I and w*L*I terms
	motor_config.bEMF_FF_enable = false; // Enable feedforward for bEMF
	
	motor_config.I_bus_hard_min = -INFINITY;
	motor_config.I_bus_hard_max = INFINITY;
	motor_config.I_leak_max = 0.1f;
	
	motor_config.dc_calib_tau = 0.2f;
}
/****************************************************************************/
void motor_setup(void)
{
	// Solve for exact gain, then snap down to have equal or larger range as requested
	// or largest possible range otherwise
	// 为了获得精确的增益，然后向下调整到与请求范围相等或更大的范围
    // 或者如果没有请求范围，则采用最大可能范围
    const float kMargin = 0.90f;  // 安全裕度系数，90%的余量，确保不饱和
    const float max_output_swing = 1.35f;  // 放大器最大输出电压摆幅 [V]（运放最大输出范围）
    
    // 计算运放能处理的最大电流信号（在未放大前的原始信号）
    // 公式：最大输出摆幅 × 安全系数 ÷ 采样电阻
    // 单位：[V] × 无量纲 ÷ [Ω] = [A]
    // 这表示运放能线性放大的最大电流对应的原始信号值
    float max_unity_gain_current = kMargin * max_output_swing * (1/SHUNT_RESISTANCE); // [A] 1215
    
    // 计算请求的增益：最大运放处理能力 ÷ 配置的电流范围
    // 这条代码被注释掉了，可能是因为采用固定增益方案
    //float requested_gain = max_unity_gain_current / config_.requested_current_range; // [V/V]
    
    const float actual_gain = 20.0f;  // 实际使用的固定放大倍数 20倍（电流采样放大倍数）
    
    // 电流控制相关参数
    // 相位电流反向增益：将ADC读值转换回实际电流值的系数
    // 公式：1 ÷ 实际放大倍数 = 1/20 = 0.05
    // 用途：ADC值 × 此系数 = 实际电流值(A)
    float phase_current_rev_gain_ = 1.0f / actual_gain;  // 0.05 [A/ADC_LSB]或类似单位
    
    // 计算系统允许的最大电流（考虑所有限制后的实际最大值）
    // 公式：运放能处理的最大电流信号 ÷ 系统总增益
    // 物理意义：考虑运放饱和、ADC量程、安全裕度后，系统能测量的最大电流
    // 1215A ÷ 20 = 60.75A（这里的1215A是经过采样电阻但未放大的"虚拟"电流值）
    max_allowed_current_ = max_unity_gain_current * phase_current_rev_gain_;  // = 1215 * 0.05 = 60.75 A
    
    // 设置直流校准的最大电流值（用于电流传感器偏置校准）
    // 使用最大允许电流的10%作为校准电流
    // 校准通常在小电流下进行以避免电机运动
    max_dc_calib_ = 0.1f * max_allowed_current_;  // 约等于 6A
}
/****************************************************************************/
float  cali_target_current_;
float  cali_max_voltage_;
float  actual_current_;
float  test_voltage_;
float  I_beta_; // [A] low pass filtered Ibeta response
float  test_mod_;
/****************************************************************************/
void Resistance_reset(void)
{
	test_voltage_ = 0;
	I_beta_ = 0;
}
/*************************************/
void Resistance_on_measurement(void)
{
	const float kI = 1.0f; // [(V/s)/A]
	const float kIBetaFilt = 80.0f;
	
	actual_current_ = Ialpha_beta[0];    //Ialpha_beta 是foc.c中clark变换后的值
	test_voltage_ += (kI * current_meas_period) * (cali_target_current_ - actual_current_);
	I_beta_ += (kIBetaFilt * current_meas_period) * (Ialpha_beta[1] - I_beta_);
	if(fabsf(test_voltage_) > cali_max_voltage_)
	{
		disarm();
		set_error(ERROR_PHASE_RESISTANCE_OUT_OF_RANGE);
	}
	else if(vbus_voltage <= 6)   //限制电源电压不能低于6V
	{
		disarm();
		set_error(ERROR_UNKNOWN_VBUS_VOLTAGE);
	}
	else
	{
		float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
		test_mod_ = test_voltage_ * vfactor;
	}
}
/*************************************/
void Resistance_get_alpha_beta_output(void)
{
	float mod_alpha = test_mod_;
	float mod_beta = 0;
	
	enqueue_modulation_timings(mod_alpha, mod_beta);
}
/****************************************************************************/
uint32_t  start_timestamp_;
uint32_t  last_input_timestamp_;
float last_Ialpha_;
float deltaI_;
bool  attached_ = false;
/****************************************************************************/
void Inductance_reset(void)
{
	attached_ = 0;
	last_Ialpha_ = 0;
	deltaI_ = 0;
}
/*************************************/
void Inductance_on_measurement(uint32_t input_timestamp)
{
	if(attached_)
	{
		float sign = test_voltage_ >= 0.0f ? 1.0f : -1.0f;
		deltaI_ += -sign * (Ialpha_beta[0] - last_Ialpha_);
	}
	else
	{
		start_timestamp_ = input_timestamp;   //第一次进入这个函数时的时间
		attached_ = 1;
	}
	
	last_Ialpha_ = Ialpha_beta[0];
	last_input_timestamp_ = input_timestamp;  //每进入一次记录一次，最后一次记录的就是最后一次的时间
}
/*************************************/
void Inductance_get_alpha_beta_output(void)
{
	test_voltage_ *= -1.0f;
	float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
	float mod_alpha = test_voltage_ * vfactor;
	float mod_beta = 0.0f;
	
	enqueue_modulation_timings(mod_alpha, mod_beta);
}
/****************************************************************************/
void update_current_controller_gains(void)
{
	// Calculate current control gains
//	float p_gain = motor_config.current_control_bandwidth * motor_config.phase_inductance;
//	float plant_pole = motor_config.phase_resistance / motor_config.phase_inductance;
	
//	if(motor_config.phase_inductance == 0)plant_pole=0;  //针对云台电机，不测量电阻电感，防止I参数无穷大，printf不好看
//	pi_gains_[0] = p_gain;
//	pi_gains_[1] = plant_pole * p_gain;
	
	pi_gains_[0] = motor_config.current_control_bandwidth * motor_config.phase_inductance;
	pi_gains_[1] = motor_config.current_control_bandwidth * motor_config.phase_resistance;
}
/****************************************************************************/
bool measure_phase_resistance(float test_current, float max_voltage)
{
	uint32_t i;
	
	cali_target_current_ = test_current;
	cali_max_voltage_ = max_voltage;
	Resistance_reset();
	meas_resis = 1;
	
	arm();
	
	for(i = 0; i < 3000; ++i)
	{
		//if (!((axis_->requested_state_ == Axis::AXIS_STATE_UNDEFINED) && axis_->motor_.is_armed_))
		if(!is_armed_)break;
		delay_us(1000);   //1ms
	}
	
	bool success = is_armed_;
	disarm();
	
	if(success)motor_config.phase_resistance = test_voltage_ / test_current;
	
	if(motor_config.phase_resistance > 2)    //MOTOR_TYPE_HIGH_CURRENT的相电阻要小于2Ω，否则就是gimbal
	{
		set_error(ERROR_NOT_HIGH_CURRENT_MOTOR);
		success = 0;
	}
	
	if((fabsf(I_beta_) / test_current) > 0.2f)
	{
		set_error(ERROR_UNBALANCED_PHASES);
		success = 0;
	}
	
	meas_resis = 0;
	return success;
}
/****************************************************************************/
// 修复后的安全时间差计算函数
static uint32_t safe_time_delta(uint32_t current, uint32_t previous) {
    if(current >= previous) {
        return current - previous; // 正常情况
    } else {
        // 发生溢出：current + (MAX - previous) + 1
        return (UINT32_MAX - previous) + current + 1;
    }
}

// 测量电机的相电感
bool measure_phase_inductance(float test_voltage)
{
    uint32_t i;
    
    // 存储用户设定的测试电压值（用于后续计算）
    test_voltage_ = test_voltage;
    // 重置电感测量相关的内部状态和变量（如清零电流采样值）
    Inductance_reset();
    // 设置电感测量标志位，告知系统当前正在测量电感（可能影响控制策略）
    meas_induc = 1;
    
    // “上电”或“使能”电机驱动器（解锁PWM输出，使能功率管）
    arm();
    
    // 等待最多1250毫秒（1250次*1000微秒），同时检查电机是否保持“使能”状态
    for(i = 0; i < 1250; ++i)
    {
        // 检查电机是否仍处于使能状态，如果不是则提前退出等待
        if(!is_armed_)break;
        // 每次循环延时1毫秒（1000微秒）
        delay_us(1000);
    }
    
    // 判断电机是否成功保持使能状态超过1.25秒（测量条件满足）
    bool success = is_armed_;
    // 测量结束，关闭电机驱动器（禁用PWM输出）
    disarm();
    
    // 如果测量条件满足（电机成功使能并保持），则计算电感值
    if(success)
    {
        // 计算电感测量过程的总时间（秒）:
        // 1. (last_input_timestamp_ - start_timestamp_)：获取两个32位时间戳的差值（单位：定时器时钟周期数）
        // 2. 除以时钟频率（TIM_1_8_CLOCK_HZ），将周期数转换为时间（秒）
        // 3. 注释警告：在216MHz时钟下，32位时间戳约19秒后溢出，但G474为170MHz，溢出时间会更长
        // float dt = (float)(last_input_timestamp_ - start_timestamp_) / (float)TIM_1_8_CLOCK_HZ;
        float dt = (float)safe_time_delta(last_input_timestamp_, start_timestamp_) / (float)TIM_1_8_CLOCK_HZ;
        // 使用电感基本公式 L = V / (di/dt) 计算相电感：
        // 1. deltaI_ 是在 dt 时间内的电流变化量（由Inductance_reset和测量过程记录）
        // 2. deltaI_ / dt 是电流变化率 (di/dt)
        // 3. fabsf(test_voltage_) 是施加测试电压的绝对值
        motor_config.phase_inductance = fabsf(test_voltage_) / (deltaI_ / dt);
    }
    
    // 校验计算出的电感值是否在合理范围内（2uH 到 4000uH）
    // 如果超出此范围，则认为是无效测量或故障
    if (!(motor_config.phase_inductance >= 2e-6f && motor_config.phase_inductance <= 4000e-6f))
    {
        // 设置错误码，标识相位电感值超出预期范围
        set_error(ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE);
        // 将本次测量标记为失败
        success = 0;
    }
    
    // 清除电感测量标志位，恢复正常控制模式
    meas_induc = 0;
    // 返回测量是否成功的布尔值
    return success;
}
/****************************************************************************/
bool run_calibration(void)
{
	float R_calib_max_voltage = motor_config.resistance_calib_max_voltage;
	
	if(motor_config.motor_type == MOTOR_TYPE_HIGH_CURRENT)
	{
		if(!measure_phase_resistance(motor_config.calibration_current, R_calib_max_voltage))return 0;
		if(!measure_phase_inductance(R_calib_max_voltage))return 0;
	}
	else if(motor_config.motor_type == MOTOR_TYPE_GIMBAL)
	{
		// __nop();// no calibration needed
	}
	else    //unknown motor type
	{
		return 0;
	}
	
	update_current_controller_gains();
	
	is_calibrated_ = 1;
	return 1;
}
/****************************************************************************/
void arm(void)
{
	armed_state_ = 1;
	is_armed_ = 1;
	controller_reset();
	foc_reset();
	TIM_CtrlPWMOutputs(TIM1, ENABLE);   //使能输出
}
/*************************************/
void disarm(void)
{
	is_armed_ = 0;
	armed_state_ = 0;
	TIM_CtrlPWMOutputs(TIM1, DISABLE);  //停止输出
}
/****************************************************************************/
bool dc_calib_valid;
/****************************************************************************/
void current_meas_cb(uint32_t timestamp, Iph_ABC_t *current)
{
	dc_calib_valid = (dc_calib_running_since_ >= motor_config.dc_calib_tau * 7.5f)    //默认7.5-需1.5秒
												&& (fabsf(DC_calib_.phA) < max_dc_calib_)
												&& (fabsf(DC_calib_.phB) < max_dc_calib_)
												&& (fabsf(DC_calib_.phC) < max_dc_calib_);
	if (armed_state_ == 1 || armed_state_ == 2)   //arm()后先空闲2次，大概是为了先让控制运算运行2次
	{
		current_meas_.phA = 0;
		current_meas_.phB = 0;
		current_meas_.phC = 0;
		armed_state_ += 1;
	}
	else if(dc_calib_valid)
	{
		current_meas_.phA = current->phA - DC_calib_.phA;
		current_meas_.phB = current->phB - DC_calib_.phB;
		current_meas_.phC = current->phC - DC_calib_.phC;
	}
	else
	{
		current_meas_.phA = 0;
		current_meas_.phB = 0;
		current_meas_.phC = 0;
	}
	
	on_measurement(timestamp, &current_meas_);
}
/****************************************************************************/
void dc_calib_cb(Iph_ABC_t *current)
{
	const float dc_calib_period = (float)(2 * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1)) / TIM_1_8_CLOCK_HZ;  //0.000125
	const float calib_filter_k = dc_calib_period / motor_config.dc_calib_tau;    //0.000625
	
	DC_calib_.phA += (current->phA - DC_calib_.phA) * calib_filter_k;
	DC_calib_.phB += (current->phB - DC_calib_.phB) * calib_filter_k;
	DC_calib_.phC += (current->phC - DC_calib_.phC) * calib_filter_k;
	dc_calib_running_since_ += dc_calib_period;
}
/****************************************************************************/
//0.5.6官方代码中，此函数被do_check()函数调用，最终在TIM1更新中断中被调用，
//移植后的代码，放在motor_update()函数中，最终在TIM1更新中断中被调用，
//计算电流限制值
float motor_effective_current_lim(void)
{
	// Configured limit
	float current_lim = motor_config.current_lim;
	// Hardware limit
	if (motor_config.motor_type == MOTOR_TYPE_GIMBAL)
		current_lim = min(current_lim, 0.98f*one_by_sqrt3*vbus_voltage); //gimbal motor is voltage control，云台电机的电压不超过 12V/1.732=6.9，防止超调
	else
		current_lim = min(current_lim, max_allowed_current_);
	
	// Apply thermistor current limiters，
	//根据电机和MOS的温度限电流，此功能有很高实用价值，但没有移植
	// current_lim = min(current_lim, motor_thermistor_.get_current_limit(config_.current_lim));
	// current_lim = min(current_lim, fet_thermistor_.get_current_limit(config_.current_lim));
	effective_current_lim_ = current_lim;
	
	return effective_current_lim_;
}
/****************************************************************************/
//return the maximum available torque for the motor.
//Note - for ACIM motors, available torque is allowed to be 0.
float motor_max_available_torque(void)
{
//	float max_torque = effective_current_lim_ * motor_config.torque_constant;
//	max_torque = clamp(max_torque, 0.0f, motor_config.torque_lim);
//	return max_torque;
	
	return effective_current_lim_ * motor_config.torque_constant;
}
/****************************************************************************/
/****************************************************************************/
void motor_update(void)
{
	// 加载转矩设定值，并转换为电机方向
	// maybe_torque 是一个指针，指向转矩设定值的来源
	float *maybe_torque;
	// 获取转矩设定值的源指针（可能来自位置环或速度环的输出）
	maybe_torque = motor_torque_setpoint_src_;
	
	// 检查是否有有效的转矩设定值
	// has_value() 函数检查指针是否指向有效数据
	if (!has_value(maybe_torque))
	{
		// 如果没有有效设定值，直接返回，不执行本次更新
		return;
	}
	
	// 获取转矩值，并根据编码器方向调整符号
	// encoder_config.direction 通常是 ±1，用于处理电机安装方向
	// *maybe_torque 解引用获取实际的转矩值
	float torque = encoder_config.direction * *maybe_torque;
	
	// 加载上一次迭代的电流设定值（dq坐标系）
	// id: 直轴电流（通常用于弱磁控制）
	// iq: 交轴电流（直接产生转矩的电流）
	float id = motor_Idq_setpoint_.d;
	float iq = motor_Idq_setpoint_.q;
	
	// 计算有效的电流限制值（考虑温度、电压限制等因素）
	// 调用函数更新 effective_current_lim_ 变量
	motor_effective_current_lim();    // 计算电流限制值
	// 获取计算后的电流限制值
	float ilim = effective_current_lim_;
	
	// 对直轴电流进行限幅（id 优先）
	// 限制 id 在 [-ilim*0.99, ilim*0.99] 范围内
	// 保留1%的空间给 iq，避免数值计算问题（当 id 达到最大值时，iq 还有空间）
	id = clamp(id, -ilim*0.99f, ilim*0.99f); // 1% space reserved for Iq to avoid numerical issues
	
	// 计算交轴电流设定值：转矩 ÷ 转矩常数
	// 转矩常数 Kt：单位电流产生的转矩（N·m/A）
	// 这是基本的转矩-电流关系式：τ = Kt × iq
	iq = torque / motor_config.torque_constant;
	
	// 对电流矢量进行2-范数（欧几里得范数）限幅，id 有优先权
	// 计算 iq 的最大允许值的平方：总电流限制的平方减去 id 的平方
	// SQ() 可能是平方宏定义，如 #define SQ(x) ((x)*(x))
	float iq_lim_sqr = SQ(ilim) - SQ(id);
	// 如果计算结果小于等于0，说明 id 已经用完了所有电流预算，iq 只能为0
	// 否则取平方根得到 iq 的最大允许值
	float Iq_lim = (iq_lim_sqr <= 0.0f) ? 0.0f : sqrt(iq_lim_sqr);
	// 对 iq 进行限幅，确保电流矢量总长度不超过 ilim
	iq = clamp(iq, -Iq_lim, Iq_lim);
	
	// 如果不是云台电机类型，更新电流设定值
	// 云台电机可能有特殊的控制模式
	if (motor_config.motor_type != MOTOR_TYPE_GIMBAL)
	{
		// 将计算后的 id, iq 保存回设定值结构体
		motor_Idq_setpoint_.d = id;
		motor_Idq_setpoint_.q = iq;
	}
	
	// 初始化电压设定值（dq坐标系）
	float vd = 0.0f;  // 直轴电压
	float vq = 0.0f;  // 交轴电压
	
	// 获取电机电角速度（电气角速度，rad/s）
	// phase_vel 是电气角速度，等于机械角速度乘以极对数
	float *phase_vel;
	phase_vel = motor_phase_vel_src_;
	
	// 如果启用了电阻-电感前馈补偿（解耦前馈）
	// R_wL_FF: Resistance and ωL FeedForward
	if (motor_config.R_wL_FF_enable)  // 电流环解耦前馈补偿
	{
		// 检查是否有有效的电角速度
		if (!has_value(phase_vel))
		{
			// 如果没有，设置错误标志并返回
			motor_error |= ERROR_UNKNOWN_PHASE_VEL;
			return;
		}
		// 解耦前馈补偿公式（基于电机电压方程）:
		// vd = R*id - ω*L*iq  （这里减号改为加号是因为前面有负号）
		// vq = R*iq + ω*L*id  （这里加号改为减号是因为前面有加号）
		// 注意：代码中的符号可能需要根据坐标系定义确认
		
		// vd 补偿项：-ωL * iq（交轴电流在直轴产生的耦合电压）
		vd -= *phase_vel * motor_config.phase_inductance * iq;
		// vq 补偿项：+ωL * id（直轴电流在交轴产生的耦合电压）
		vq += *phase_vel * motor_config.phase_inductance * id;
		// vd 补偿项：+R * id（电阻压降）
		vd += motor_config.phase_resistance * id;
		// vq 补偿项：+R * iq（电阻压降）
		vq += motor_config.phase_resistance * iq;
	}
	
	// 如果启用了反电动势前馈补偿
	if (motor_config.bEMF_FF_enable)  // 反电动势补偿
	{
		// 检查是否有有效的电角速度
		if (!has_value(phase_vel))
		{
			// 如果没有，设置错误标志并返回
			motor_error |= ERROR_UNKNOWN_PHASE_VEL;
			return;
		}
		// 反电动势补偿公式：
		// 反电动势常数 Ke = Kt * (2/3) / pole_pairs （单位转换）
		// vq 需要补偿反电动势：+ω * Ke
		// 注意：这里直接加到 vq，因为反电动势主要在 q 轴
		vq += *phase_vel * (2.0f/3.0f) * (motor_config.torque_constant / motor_config.pole_pairs);
	}
	
	// 如果是云台电机类型，特殊处理
	if (motor_config.motor_type == MOTOR_TYPE_GIMBAL)
	{
		// 云台电机可能工作在电压模式或特殊控制模式
		// 将电流值重新解释为电压值（可能是直接开环电压控制）
		motor_Vdq_setpoint_.d = vd + id;  // 电压 = 前馈电压 + 电流值（特殊处理）
		motor_Vdq_setpoint_.q = vq + iq;  // 电压 = 前馈电压 + 电流值（特殊处理）
	}
	else
	{
		// 普通电机：直接使用计算的前馈电压
		// 注意：这里只计算了前馈部分，实际控制中还有PI控制器的输出
		motor_Vdq_setpoint_.d = vd;  // 直轴电压设定值
		motor_Vdq_setpoint_.q = vq;  // 交轴电压设定值
	}
}
/****************************************************************************/



