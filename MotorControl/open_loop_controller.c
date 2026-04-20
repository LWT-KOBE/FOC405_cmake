
#include "MyProject.h"


/****************************************************************************/
OPENLOOP_struct openloop_controller_;
/****************************************************************************/
void openloop_controller_update(void)
{
    // 获取开环控制器结构体指针
    // p 指向全局的开环控制器状态结构体
    OPENLOOP_struct *p;
    p = &openloop_controller_;
    
    // 保存上一控制周期的状态值（用于斜坡限制）
    // 只保存直轴电流Id（交轴电流Iq在开环中通常设为0）
    float prev_Id = p->Idq_setpoint_.d;
    //float prev_Iq = p->Idq_setpoint_.q;  // 注释掉，可能暂时不用
    
    // 保存上一周期的电压设定值
    float prev_Vd = p->Vdq_setpoint_.d;
    //float prev_Vq = p->Vdq_setpoint_.q;  // 注释掉，可能暂时不用
    
    // 获取当前电角度和电角速度
    float phase = p->phase_;       // 当前电角度，单位rad
    float phase_vel = p->phase_vel_; // 当前电角速度，单位rad/s
    
    // 设置固定的控制周期（125μs，对应8kHz控制频率）
    // 注释显示原代码根据时间戳计算dt，但这里使用了固定值
    // TIM_1_8_CLOCK_HZ 可能是时钟频率，如72MHz或类似
    float dt = 0.000125f; // 0.000125秒 = 125微秒
    
    // 更新直轴电流设定值，并应用斜坡限制
    // 确保电流变化率不超过 max_current_ramp_
    // clamp(目标值, 最小值, 最大值) 将值限制在最小最大值之间
    // 这里确保当前Id相对上一周期的变化量不超过 max_current_ramp_ * dt
    p->Idq_setpoint_.d = clamp(p->target_current_, 
                              prev_Id - p->max_current_ramp_ * dt, 
                              prev_Id + p->max_current_ramp_ * dt);
    
    // 开环控制中，交轴电流通常设为0（不产生转矩）
    // 因为开环主要是对齐或测试，不是驱动
    p->Idq_setpoint_.q = 0;
    
    // 更新直轴电压设定值，同样应用斜坡限制
    // 确保电压变化率不超过 max_voltage_ramp_
    p->Vdq_setpoint_.d = clamp(p->target_voltage_,
                              prev_Vd - p->max_voltage_ramp_ * dt,
                              prev_Vd + p->max_voltage_ramp_ * dt);
    
    // 开环控制中，交轴电压也设为0
    p->Vdq_setpoint_.q = 0;
    
    // 更新电角速度，应用斜坡限制
    // 确保角速度变化率不超过 max_phase_vel_ramp_
    phase_vel = clamp(p->target_vel_,
                     p->phase_vel_ - p->max_phase_vel_ramp_ * dt,
                     p->phase_vel_ + p->max_phase_vel_ramp_ * dt);
    
    // 将计算后的角速度保存回结构体
    p->phase_vel_ = phase_vel;
    
    // 更新电角度：新角度 = 旧角度 + 角速度 × 时间
    // wrap_pm_pi() 将角度限制在 [-π, π] 或 [-180°, 180°] 范围内
    // 防止角度无限增长导致数值溢出
    p->phase_ = wrap_pm_pi(phase + phase_vel * dt);
    
    // 累计总行程距离（角度积分）
    // total_distance_ 记录电机自启动以来转过的总电角度
    // 用于行程统计或某些控制算法
    p->total_distance_ = p->total_distance_ + phase_vel * dt;
    
    // 更新时间戳（注释掉了，原代码可能根据实际时间计算dt）
    //p->timestamp_ = timestamp;
}
/****************************************************************************/



