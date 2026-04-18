#ifndef _TRAP_TRAJ_H
#define _TRAP_TRAJ_H

#include "MyProject.h"


/****************************************************************************/
typedef struct 
{
	float vel_limit;   // [turn/s]
	float accel_limit; // [turn/s^2]
	float decel_limit; // [turn/s^2]
} TRAPTRAJ_Config_t;

typedef struct 
{
	float Y;
	float Yd;
	float Ydd;
} Step_t;

typedef struct 
{
	float Xi_;
	float Xf_;
	float Vi_;
	
	float Ar_;
	float Vr_;
	float Dr_;
	
	float Ta_;
	float Tv_;
	float Td_;
	float Tf_;
	
	float yAccel_;
	
	float t_;
} TRAPTRAJ_t;

extern  TRAPTRAJ_Config_t  trapTraj_config;
extern  TRAPTRAJ_t  trap_traj_;
/****************************************************************************/
void trapTraj_config_default(void);
uint8_t planTrapezoidal(float Xf, float Xi, float Vi,float Vmax, float Amax, float Dmax);
Step_t trap_traj_eval(float t);
/****************************************************************************/


// 1. 在原有结构体后添加S型轨迹结构体
typedef struct 
{
    float vel_limit;     // [turn/s]
    float accel_limit;   // [turn/s^2]
    float decel_limit;   // [turn/s^2]
    float jerk_limit;    // [turn/s^3] - S型轨迹特有参数
} SCURVETRAJ_Config_t;

typedef struct 
{
    float Xi_;          // 起始位置
    float Xf_;          // 目标位置
    float Vi_;          // 初始速度
    
    // 基本参数
    float Vmax_;        // 最大速度
    float Amax_;        // 最大加速度
    float Dmax_;        // 最大减速度
    float Jmax_;        // 最大加加速度
    
    // 阶段时间
    float Tj1_;         // 加加速度上升时间
    float Tj2_;         // 加加速度下降时间
    float Ta_;          // 加速段时间
    float Tv_;          // 匀速段时间
    float Td_;          // 减速段时间
    float T_;           // 总时间
    
    // 中间计算结果
    float Vr_;          // 实际达到的最大速度
    float Ar_;          // 实际达到的最大加速度
    float Dr_;          // 实际达到的最大减速度
    
    // 阶段边界位置
    float yAccel_;      // 加速段结束位置
    float yDecel_;      // 减速段开始位置
    
    float t_;           // 当前时间
    uint8_t has_coast_; // 是否有匀速段
} SCURVETRAJ_t;

/****************************************************************************/
// 全局变量声明（与梯形轨迹保持一致）
extern SCURVETRAJ_Config_t  sCurveTraj_config;
extern SCURVETRAJ_t  s_curve_traj_;
extern bool s_curve_trajectory_done_;  // 轨迹完成标志

void sCurveTraj_config_default(void);
uint8_t planSCurve(float Xf, float Xi, float Vi, float Vmax, float Amax, float Dmax, float Jmax);
uint8_t planSCurve_simple(float Xf, float Xi, float Vi);
Step_t s_curve_traj_eval(float t);

#endif
