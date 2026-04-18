
#include "MyProject.h"


/****************************************************************************/
AUTOTURNING_t  autotuning_;
CONTROLLER_Config_t  ctrl_config;

uint32_t  control_error;
//float last_error_time_ = 0.0f;
// Inputs
float *controller_pos_estimate_linear_src_ = NULL;
float *controller_pos_estimate_circular_src_ = NULL;
float *controller_vel_estimate_src_ = NULL;
float *controller_pos_wrap_src_ = NULL; 

float pos_setpoint_ = 0.0f; // [turns]
float vel_setpoint_ = 0.0f; // [turn/s]
float vel_integrator_torque_ = 0.0f;    // [Nm]
float torque_setpoint_ = 0.0f;  // [Nm]

float input_pos_ = 0.0f;     // [turns]
float input_vel_ = 0.0f;     // [turn/s]
float input_torque_ = 0.0f;  // [Nm]
float input_filter_kp_ = 0.0f;
float input_filter_ki_ = 0.0f;

float autotuning_phase_ = 0.0f;

bool input_pos_updated_ = false;

bool trajectory_done_ = true;

float mechanical_power_ = 0.0f; // [W]
float electrical_power_ = 0.0f; // [W]

// Outputs
float torque_output_ = 0.0f;
float mit_target_velocity_ = 0.0f,mit_target_pos_ = 0.0f, mit_target_torque_ = 0.0f; // MIT模式下的目标速度
float mit_kp = 0.0f, mit_kd = 0.05f; // MIT模式下的控制参数
/****************************************************************************/
void Controller_update_filter_gains(void);  //函数声明
/****************************************************************************/
void controller_config_default(void)
{
	ctrl_config.control_mode = CONTROL_MODE_POSITION_CONTROL;  //see: ControlMode_t
	ctrl_config.input_mode = INPUT_MODE_PASSTHROUGH;  //see: InputMode_t
	ctrl_config.pos_gain = 20.0f;                  // [(turn/s) / turn]
	ctrl_config.vel_gain = 1.0f / 6.0f;            // [Nm/(turn/s)]
	ctrl_config.vel_integrator_gain = 2.0f / 6.0f; // [Nm/(turn/s * s)]
	ctrl_config.vel_limit = 2.0f;                  // [turn/s] Infinity to disable.
	ctrl_config.vel_limit_tolerance = 1.2f;        // ratio to vel_lim. Infinity to disable.
	ctrl_config.vel_integrator_limit = INFINITY;   // Vel. integrator clamping value. Infinity to disable.
	ctrl_config.vel_ramp_rate = 1.0f;              // [(turn/s) / s]
	ctrl_config.torque_ramp_rate = 0.01f;          // Nm / sec
	ctrl_config.circular_setpoints = 0;
	ctrl_config.circular_setpoint_range = 1.0f;    // Circular range when circular_setpoints is true. [turn]
	//ctrl_config.steps_per_circular_range = 1024;
	ctrl_config.inertia = 0.0f;                    // [Nm/(turn/s^2)]
	ctrl_config.input_filter_bandwidth = 2.0f;     // [1/s]
	ctrl_config.homing_speed = 0.25f;              // [turn/s]
	
	ctrl_config.gain_scheduling_width = 10.0f;
	ctrl_config.enable_gain_scheduling = false;
	ctrl_config.enable_vel_limit = true;
	ctrl_config.enable_overspeed_error = true;
	ctrl_config.enable_torque_mode_vel_limit = true;  // 力矩模式下，如果电机转速超过vel_limit ，电机输出的力矩将会减小
	//ctrl_config.axis_to_mirror = -1;
	//ctrl_config.mirror_ratio = 1.0f;
	//ctrl_config.torque_mirror_ratio = 0.0f;
	//ctrl_config.load_encoder_axis = -1;  // default depends on Axis number and is set in load_configuration(). Set to -1 to select sensorless estimator.
	ctrl_config.mechanical_power_bandwidth = 20.0f; // [rad/s] filter cutoff for mechanical power for spinout detction
	ctrl_config.electrical_power_bandwidth = 20.0f; // [rad/s] filter cutoff for electrical power for spinout detection
	ctrl_config.spinout_electrical_power_threshold = 10.0f;  // [W] electrical power threshold for spinout detection
	ctrl_config.spinout_mechanical_power_threshold = -10.0f; // [W] mechanical power threshold for spinout detection
	
	Controller_update_filter_gains();
}
/**************************************/
void controller_para_init(void)
{
	ctrl_config.control_mode = CONTROL_mode;         //控制模式
	ctrl_config.input_mode = INPUT_mode;             //输入模式
	ctrl_config.torque_ramp_rate = TORQUE_ramp_rate; //Nm / sec，力矩爬升率
	ctrl_config.vel_ramp_rate = VELOCITY_ramp_rate;  //速度的爬升率
	ctrl_config.vel_gain = VELOCITY_P;               //速度P参数
	ctrl_config.vel_integrator_gain = VELOCITY_I;    //速度I参数
	ctrl_config.vel_limit = VELOCITY_limit;          //最大转速限制，圈/秒
	ctrl_config.pos_gain = POSITION_P;               //位置P参数
}
/****************************************************************************/
//arm() 函数中调用一次
void controller_reset(void) 
{
	// pos_setpoint is initialized in start_closed_loop_control
	vel_setpoint_ = 0.0f;
	vel_integrator_torque_ = 0.0f;
	torque_setpoint_ = 0.0f; 
	mechanical_power_ = 0.0f;
	electrical_power_ = 0.0f;
}
/****************************************************************************/
void move_to_pos(float goal_point)
{
	planTrapezoidal(goal_point, pos_setpoint_, vel_setpoint_, trapTraj_config.vel_limit, trapTraj_config.accel_limit, trapTraj_config.decel_limit);
	trap_traj_.t_ = 0.0f;
	trajectory_done_ = false;
}

// S型轨迹移动函数（与梯形轨迹的move_to_pos对应）
void move_to_pos_s_curve(float goal_point)
{
    planSCurve_simple(goal_point, pos_setpoint_, vel_setpoint_);
    // planSCurve(goal_point, pos_setpoint_, vel_setpoint_,
    //            sCurveTraj_config.vel_limit,
    //            sCurveTraj_config.accel_limit,
    //            sCurveTraj_config.decel_limit,
    //            sCurveTraj_config.jerk_limit);
    s_curve_traj_.t_ = 0.0f;
    s_curve_trajectory_done_ = false;
}

/****************************************************************************/
bool control_mode_updated(void)
{
	float *estimate;
	
	if (ctrl_config.control_mode >= CONTROL_MODE_POSITION_CONTROL)
	{
		estimate = (ctrl_config.circular_setpoints ? controller_pos_estimate_circular_src_ : controller_pos_estimate_linear_src_);
		pos_setpoint_ = *estimate;
		input_pos_ = *estimate;    //把当前位置做为目标位置，保证闭环后电机不动
	}
	return true;
}
/****************************************************************************/
//生成的两个变量用于 INPUT_MODE_POS_FILTER 模式
void Controller_update_filter_gains(void)
{
	float bandwidth = min(ctrl_config.input_filter_bandwidth, 0.25f * current_meas_hz);
	input_filter_ki_ = 2.0f * bandwidth;  // basic conversion to discrete time
	input_filter_kp_ = 0.25f * (input_filter_ki_ * input_filter_ki_); // Critically damped
}
/****************************************************************************/
static float limitVel(float vel_limit, float vel_estimate, float vel_gain, float torque)
{
	float Tmax = (vel_limit - vel_estimate) * vel_gain;
	float Tmin = (-vel_limit - vel_estimate) * vel_gain;
	return clamp(torque, Tmin, Tmax);
}
/****************************************************************************/
bool controller_update(void)
{
	// 控制器更新函数，每个控制周期调用一次
    // 定义指针变量用于获取位置、速度估计值
    float *pos_estimate_linear;       // 线性位置估计值指针（用于非圆周运动）
    float *pos_estimate_circular;     // 圆周位置估计值指针（用于圆周运动）
    float *pos_wrap;                  // 圆周位置包装值指针（用于圆周范围限制）
    float *vel_estimate;              // 速度估计值指针
    float *anticogging_pos_estimate;  // 抗齿槽效应位置估计值指针
    float *anticogging_vel_estimate;  // 抗齿槽效应速度估计值指针
        
    // 从全局变量获取位置和速度估计值
    pos_estimate_linear = controller_pos_estimate_linear_src_;    // 获取线性位置估计源
    pos_estimate_circular = controller_pos_estimate_circular_src_;// 获取圆周位置估计源
    pos_wrap = controller_pos_wrap_src_;                         // 获取圆周包装值源
    vel_estimate = controller_vel_estimate_src_;                 // 获取速度估计源
    
    // 设置抗齿槽效应估计值指针（默认使用局部变量）
    anticogging_pos_estimate = &pos_estimate_;  // 指向局部位置估计变量
    anticogging_vel_estimate = &vel_estimate_;  // 指向局部速度估计变量
    
    // 检查速度估计值是否有效，无效则返回0
    if (!has_value(vel_estimate))return 0;  // 如果速度估计值无效，直接返回失败
    
    // 检查是否正在进行抗齿槽效应校准
    if (anticogging.calib_anticogging)  // 如果校准标志为真
    {
        // 非阻塞式抗齿槽效应校准
        anticogging_calibration(*anticogging_pos_estimate, *anticogging_vel_estimate);  // 调用校准函数
    }
        
    // 如果是圆周设定点模式，对输入位置进行包装
    if (ctrl_config.circular_setpoints)  // 检查是否启用圆周设定点
    {
        // 检查圆周包装值是否有效
        if (!has_value(pos_wrap))  // 如果包装值无效
        {
            set_error(ERROR_INVALID_CIRCULAR_RANGE);  // 设置圆周范围无效错误
            return 0;                                 // 返回失败
        }
        // 对输入位置进行圆周包装（取模运算）
        input_pos_ = fmodf_pos(input_pos_, *pos_wrap);  // 将输入位置限制在0~pos_wrap范围内
    }
    
    // 根据输入模式选择不同的控制策略
    switch (ctrl_config.input_mode)  // 检查输入模式
    {
        case INPUT_MODE_INACTIVE: {  // 不起作用模式，不赋值给设定值
            // 什么都不做，保持原有设定值
            } break;
        
        case INPUT_MODE_PASSTHROUGH: {  // 直通模式，输入指令直接赋值给设定值
            pos_setpoint_ = input_pos_;       // 位置设定值 = 输入位置
            vel_setpoint_ = input_vel_;       // 速度设定值 = 输入速度
            torque_setpoint_ = input_torque_; // 力矩设定值 = 输入力矩
        } break;
        
        case INPUT_MODE_VEL_RAMP: {  // 速度爬升模式
            // 计算每个周期的最大速度变化步长
            float max_step_size = fabsf(current_meas_period * ctrl_config.vel_ramp_rate);  // 最大步长 = 周期×爬升率
            // 计算完整的速度变化量
            float full_step = input_vel_ - vel_setpoint_;  // 需要变化的总速度
            // 限制步长在最大允许范围内
            float step = clamp(full_step, -max_step_size, max_step_size);  // 限幅
            // 更新速度设定值
            vel_setpoint_ += step;  // 速度设定值增加步长
            // 根据加速度计算力矩设定值
            torque_setpoint_ = (step / current_meas_period) * ctrl_config.inertia;  // 力矩 = 加速度×惯量
        } break;
        
        case INPUT_MODE_TORQUE_RAMP: {  // 力矩爬升模式
            // 计算每个周期的最大力矩变化步长
            float max_step_size = fabsf(current_meas_period * ctrl_config.torque_ramp_rate);  // 最大步长 = 周期×力矩爬升率
            // 计算完整的力矩变化量
            float full_step = input_torque_ - torque_setpoint_;  // 需要变化的总力矩
            // 限制步长在最大允许范围内
            float step = clamp(full_step, -max_step_size, max_step_size);  // 限幅
            // 更新力矩设定值
            torque_setpoint_ += step;  // 力矩设定值增加步长
        } break;
        
        case INPUT_MODE_POS_FILTER: {  // 位置平滑模式
            // 二阶位置跟踪滤波器
            float delta_pos = input_pos_ - pos_setpoint_;  // 位置误差 = 输入位置 - 当前位置设定值
            // 如果是圆周运动，需要对位置误差进行包装
            if (ctrl_config.circular_setpoints)  // 检查圆周设定点
            {
                delta_pos = wrap_pm(delta_pos, *pos_wrap);  // 将位置误差包装到±pos_wrap/2范围内
            }
            // 计算速度误差
            float delta_vel = input_vel_ - vel_setpoint_;   // 速度误差 = 输入速度 - 当前速度设定值
            // 计算需要的加速度（PD控制）
            float accel = input_filter_kp_*delta_pos + input_filter_ki_*delta_vel;  // 加速度 = Kp×位置误差 + Ki×速度误差
            // 根据加速度计算力矩设定值
            torque_setpoint_ = accel * ctrl_config.inertia;  // 力矩 = 加速度×惯量
            // 更新速度设定值
            vel_setpoint_ += current_meas_period * accel;    // 速度设定值 = 原速度 + 加速度×时间
            // 更新位置设定值
            pos_setpoint_ += current_meas_period * vel_setpoint_;  // 位置设定值 = 原位置 + 速度×时间
        } break;
        
        case INPUT_MODE_TRAP_TRAJ: {  // 梯形轨迹
            // 检查输入位置是否更新
            if(input_pos_updated_)  // 如果输入位置已更新
            {
                move_to_pos(input_pos_);  // 调用移动到目标位置函数
                input_pos_updated_ = false;  // 清除更新标志
            }
            // 避免更新未初始化的轨迹
            if (trajectory_done_)break;  // 如果轨迹已完成，跳出
            
            // 检查轨迹是否完成（时间超过总时间）
            if (trap_traj_.t_ > trap_traj_.Tf_)  // 当前时间 > 轨迹总时间
            {
                // 轨迹完成后切换到位置控制模式，避免循环计数器溢出问题
                ctrl_config.control_mode = CONTROL_MODE_POSITION_CONTROL;  // 切换到位置控制模式
                pos_setpoint_ = trap_traj_.Xf_;  // 位置设定值 = 轨迹终点位置
                vel_setpoint_ = 0.0f;            // 速度设定值归零
                torque_setpoint_ = 0.0f;         // 力矩设定值归零
                trajectory_done_ = true;         // 设置轨迹完成标志
            }
            else  // 轨迹未完成
            {
                // 计算当前轨迹步的状态
                Step_t  traj_step = trap_traj_eval(trap_traj_.t_);  // 评估轨迹在当前时间点的状态
                pos_setpoint_ = traj_step.Y;       // 位置设定值 = 轨迹位置
                vel_setpoint_ = traj_step.Yd;      // 速度设定值 = 轨迹速度
                torque_setpoint_ = traj_step.Ydd * ctrl_config.inertia;  // 力矩设定值 = 轨迹加速度×惯量
                trap_traj_.t_ += current_meas_period;  // 轨迹时间增加一个周期
            }
            // 抗齿槽效应位置估计使用位置设定值而不是位置估计值（前馈补偿）
            anticogging_pos_estimate = &pos_setpoint_;  // 更改位置估计指针指向位置设定值
        } break;
        
        case INPUT_MODE_TUNING: {  // 单圈循环模式，只在一圈内转动
            // 更新自动调谐相位（周期性变化）
            autotuning_phase_ = wrap_pm_pi(autotuning_phase_ + (2.0f * M_PI * autotuning_.frequency * current_meas_period));  // 相位累加并包装到±π范围内
            // 计算正弦和余弦值
            float c = our_arm_cos_f32(autotuning_phase_);  // 余弦值
            float s = our_arm_sin_f32(autotuning_phase_);  // 正弦值
            // 位置设定值 = 输入位置 + 位置振幅×正弦波
            pos_setpoint_ = input_pos_ + autotuning_.pos_amplitude * s; // + pos_amp_c * c
            // 速度设定值 = 输入速度 + 速度振幅×余弦波
            vel_setpoint_ = input_vel_ + autotuning_.vel_amplitude * c;
            // 力矩设定值 = 输入力矩 + 力矩振幅×负正弦波
            torque_setpoint_ = input_torque_ + autotuning_.torque_amplitude * -s;
        } break;
        
        case INPUT_MODE_MIT: {  // 麻省理工学院模式（PD+前馈控制）
            // MIT控制公式：力矩设定值 = Kp×位置误差 + Kd×速度误差 + 前馈力矩
            torque_setpoint_ = mit_kp * (mit_target_pos_ - *pos_estimate_linear)  // Kp×位置误差
                             + mit_kd * (mit_target_velocity_ - *vel_estimate)    // Kd×速度误差
                             + mit_target_torque_;                                // 前馈力矩
        } break;

		case INPUT_MODE_S_CURVE_TRAJ: {  // S曲线轨迹模式
			// 检查输入位置是否更新
        if(input_pos_updated_)
        {
            move_to_pos_s_curve(input_pos_);  // 使用S型轨迹移动函数
            input_pos_updated_ = false;
        }
        
        // 避免更新未初始化的轨迹
        if (s_curve_trajectory_done_) break;
        
        // 检查轨迹是否完成（时间超过总时间）
        if (s_curve_traj_.t_ > s_curve_traj_.T_)
        {
            // 轨迹完成后切换到位置控制模式
            ctrl_config.control_mode = CONTROL_MODE_POSITION_CONTROL;
            pos_setpoint_ = s_curve_traj_.Xf_;
            vel_setpoint_ = 0.0f;
            torque_setpoint_ = 0.0f;
            s_curve_trajectory_done_ = true;
        }
        else
        {
            // 计算当前轨迹步的状态
            Step_t traj_step = s_curve_traj_eval(s_curve_traj_.t_);
            pos_setpoint_ = traj_step.Y;
            vel_setpoint_ = traj_step.Yd;
            torque_setpoint_ = traj_step.Ydd * ctrl_config.inertia;  // 力矩 = 加速度 × 惯量
            
            // 更新时间
            s_curve_traj_.t_ += current_meas_period;
        }
        
        // 抗齿槽效应位置估计使用位置设定值而不是位置估计值（前馈补偿）
        anticogging_pos_estimate = &pos_setpoint_;
		} break;

        default: {  // 无效的输入模式
            set_error(ERROR_INVALID_INPUT_MODE);  // 设置输入模式无效错误
            return 0;  // 返回失败
        }
    }
    
    // 永远不要让设定值超过限制
    if(ctrl_config.enable_vel_limit)  // 检查是否启用速度限制
        vel_setpoint_ = clamp(vel_setpoint_, -ctrl_config.vel_limit, ctrl_config.vel_limit);  // 限制速度设定值在±vel_limit范围内
    
    // 获取电机最大可用力矩
    const float Tlim = motor_max_available_torque();  // max_torque = effective_current_lim_ * config_.torque_constant = 60*0.04f = 2.4f;
    // 限制力矩设定值在最大力矩范围内
    torque_setpoint_ = clamp(torque_setpoint_, -Tlim, Tlim);
    
    // 位置控制 - 位置环PID运算开始
    // TODO: 决定这里使用编码器位置还是PLL位置
    float gain_scheduling_multiplier = 1.0f;  // 增益调度乘数，默认为1
    float vel_des = vel_setpoint_;  // 期望速度初始化为速度设定值
    
    // 如果控制模式包含位置控制或更高级别
    if (ctrl_config.control_mode >= CONTROL_MODE_POSITION_CONTROL)  // 检查控制模式级别
    {
        float pos_err;  // 位置误差
        // 如果是圆周设定点模式
        if (ctrl_config.circular_setpoints)  // 检查圆周设定点
        {
            // 保持位置设定值不漂移（圆周包装）
            pos_setpoint_ = fmodf_pos(pos_setpoint_, *pos_wrap);  // 对位置设定值进行圆周包装
            // 计算圆周位置误差
            pos_err = pos_setpoint_ - *pos_estimate_circular;  // 位置误差 = 设定值 - 圆周位置估计
            pos_err = wrap_pm(pos_err, *pos_wrap);  // 包装位置误差到±pos_wrap/2范围内
        }
        else  // 线性设定点模式
        {
            // 计算线性位置误差
            pos_err = pos_setpoint_ - *pos_estimate_linear;  // 位置误差 = 设定值 - 线性位置估计
        }
        
        // 位置环P控制：期望速度增加位置误差×位置增益
        vel_des += ctrl_config.pos_gain * pos_err;  // 位置环PID运算，只有P参数
        
        // 基于位置误差的V形增益调度
        float abs_pos_err = fabsf(pos_err);  // 位置误差绝对值
        // 检查是否启用增益调度且误差在调度宽度内
        if (ctrl_config.enable_gain_scheduling && abs_pos_err <= ctrl_config.gain_scheduling_width) {
            // 计算增益调度乘数（误差越大，增益乘数越大）
            gain_scheduling_multiplier = abs_pos_err / ctrl_config.gain_scheduling_width;  // 线性增益调度
        }
    }
    
    // 速度限制
    float vel_lim = ctrl_config.vel_limit;  // 获取速度限制值
    if (ctrl_config.enable_vel_limit)  // 检查是否启用速度限制
    {
        vel_des = clamp(vel_des, -vel_lim, vel_lim);  // 限制期望速度在±vel_lim范围内
    }
    // 过速故障检查（在本模块中完成，与速度限制保持一致）
    if (ctrl_config.enable_overspeed_error)   // 检查是否启用过速错误检测（0.0f表示禁用）
    {
        // 如果实际速度超过速度限制×容忍度
        if (fabsf(*vel_estimate) > ctrl_config.vel_limit_tolerance * vel_lim)  // 检查过速
        {
            set_error(ERROR_OVERSPEED);  // 设置过速错误
            return 0;  // 返回失败
        }
    }
    // 位置环PID运算结束
    
    // 速度环PID运算开始
    float vel_gain = ctrl_config.vel_gain;  // 速度环P增益
    float vel_integrator_gain = ctrl_config.vel_integrator_gain;  // 速度环I增益
    // 速度控制
    float torque = torque_setpoint_;  // 初始力矩 = 力矩设定值
    
    // 抗齿槽效应校准后启用
    // 获取当前位置并应用电流前馈
    // 确保正确处理负编码器位置（-1 == motor->encoder.encoder_cpr - 1）
    if(anticogging_valid_ && anticogging.anticogging_enabled)  // 检查抗齿槽效应是否有效且启用
    {
        // 计算抗齿槽效应位置索引
        float anticogging_pos = *anticogging_pos_estimate * COG_num;  // 抗齿槽位置 = 位置估计×齿槽数量
        int cogindex = (int)anticogging_pos % COG_num;  // 计算齿槽索引（取模）
        // 索引边界检查
        if(cogindex<0)cogindex=0;  // 确保索引不小于0
        if(cogindex>COG_num)cogindex=COG_num;  // 确保索引不超过最大值
        // 添加抗齿槽补偿力矩
        torque += anticogging.cogging_map[cogindex];  // 力矩增加齿槽补偿值
        
        // （注释掉的LED指示代码）
        // static uint32_t lednumb;
        // if(++lednumb >= 2000)  //0.25s   LED灯放在这个地方，方便观察是否使用抗齿槽校准功能
        // {
        //     lednumb = 0;
        //     GPIOD->ODR^=(1<<2);  // 翻转LED状态
        // }
    }
    
    float v_err = 0.0f;  // 速度误差，初始化为0
    // 如果控制模式包含速度控制或更高级别
    if (ctrl_config.control_mode >= CONTROL_MODE_VELOCITY_CONTROL)  // 检查控制模式级别
    {
        // 计算速度误差
        v_err = vel_des - *vel_estimate;  // 速度误差 = 期望速度 - 速度估计值
        // 速度环P控制：力矩增加速度误差×速度增益×增益调度乘数
        torque += (vel_gain * gain_scheduling_multiplier) * v_err;  // 速度环P运算

        // 限幅前的速度积分作用
        torque += vel_integrator_torque_;  // 力矩增加速度积分项
    }
    
    // 电流模式下的速度限制（当不处于速度控制模式时）
    if (ctrl_config.control_mode < CONTROL_MODE_VELOCITY_CONTROL && ctrl_config.enable_torque_mode_vel_limit)
    {
        // 调用速度限制函数
        torque = limitVel(ctrl_config.vel_limit, *vel_estimate, vel_gain, torque);
    }
    
    // 力矩限制
    uint8_t limited = 0;  // 限制标志，0=未限制，1=已限制
    if (torque > Tlim)  // 力矩超过上限
    {
        limited = 1;      // 设置限制标志
        torque = Tlim;    // 力矩钳位到上限
    }
    if (torque < -Tlim)  // 力矩低于下限
    {
        limited = 1;      // 设置限制标志
        torque = -Tlim;   // 力矩钳位到下限
    }
    
    // 速度积分器（行为取决于是否被限制）
    if (ctrl_config.control_mode < CONTROL_MODE_VELOCITY_CONTROL)  // 不处于速度控制模式
    {
        // 如果不使用积分器，则重置积分项
        vel_integrator_torque_ = 0.0f;  // 积分项清零
    }
    else  // 处于速度控制模式
    {
        if (limited)  // 如果力矩被限制
        {
            // TODO: 使衰减因子可配置
            vel_integrator_torque_ *= 0.99f;  // 积分项衰减（防积分饱和）
        }
        else  // 力矩未被限制
        {
            // 积分项增加：速度误差×积分增益×增益调度乘数×周期时间
            vel_integrator_torque_ += ((vel_integrator_gain * gain_scheduling_multiplier) * current_meas_period) * v_err;  // 速度环I运算
        }
        // 积分器限幅防止积分饱和
        vel_integrator_torque_ = clamp(vel_integrator_torque_, -ctrl_config.vel_integrator_limit, ctrl_config.vel_integrator_limit);
    }
    // 速度环PID运算结束
    
    // （注释掉的功率计算和旋转检测代码）
    
    // float ideal_electrical_power = 0.0f;
    // if (motor_config.motor_type != MOTOR_TYPE_GIMBAL)
    // {
    //     // 计算理想电功率
    //     ideal_electrical_power = power_ - \
    //         // SQ(Iq_measured) * 1.5f * motor_config.phase_resistance - \
    //         // SQ(Iq_measured) * 1.5f * motor_config.phase_resistance;

    //         SQ(Iq_measured) * 1.5f * motor_config.phase_resistance - \
    //         SQ(Iq_measured) * 1.5f * motor_config.phase_resistance;
    // }
    
    // else
    // {
    //     ideal_electrical_power = power_;
    // }
    // // 更新机械功率（一阶低通滤波器）
    // mechanical_power_ += ctrl_config.mechanical_power_bandwidth * current_meas_period * (torque * *vel_estimate * M_PI * 2.0f - mechanical_power_);
    // // 更新电功率（一阶低通滤波器）
    // electrical_power_ += ctrl_config.electrical_power_bandwidth * current_meas_period * (ideal_electrical_power - electrical_power_);

    // 旋转检测
    // 如果机械功率为负（制动）但测量功率为正，说明有问题
    // 这表明控制器试图停止，但仍在产生力矩
    // 通常由编码器偏移不正确引起
    // if ((mechanical_power_ < ctrl_config.spinout_mechanical_power_threshold) && (electrical_power_ > ctrl_config.spinout_electrical_power_threshold))
    // {
    //     set_error(ERROR_SPINOUT_DETECTED);  // 设置旋转检测错误
    //     return 0;  // 返回失败
    // }
    
    
    // 输出最终力矩值
    torque_output_ = torque;  // 设置力矩输出值
    
    // TODO: 这与其他粘性错误不一致。
    // 然而，如果我们使ERROR_INVALID_ESTIMATE具有粘性，那么
    // 正常的电机校准+编码器校准序列会使控制器处于错误状态，这会造成混淆。
    // 清除无效估计错误标志
    motor_error &= ~ERROR_INVALID_ESTIMATE;  // 清除无效估计错误位
    
    return 1;  // 返回成功
}
/****************************************************************************/



