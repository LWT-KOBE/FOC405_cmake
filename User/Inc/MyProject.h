#ifndef MYPROJECT_H
#define MYPROJECT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

// usb相关头文件
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"

// 系统外设头文件 
#include "delay.h"
#include "usart2.h"
#include "timer.h"
#include "adc.h"
#include "spi3.h"


// 电机控制相关头文件
#include "utils.h"
#include "arm_cos_f32.h"
#include "foc.h"
#include "open_loop_controller.h"
#include "board.h"
#include "motor.h"
#include "axis.h"
#include "encoder.h"
#include "trapTraj.h"
#include "controller.h"
#include "flash_writer.h"
#include "anticogging.h"
#include "ntc.h"
//
// // 系统调度与调试相关头文件
#include "vofa.h"
#include "SEGGER_RTT.h"
// #include "scheduler.h"
// #include "debug.h"
#include "IAP.h"
#include "dwt.h"
#include "can.h"
#include "communication_usb.h"
#include "communication_usart.h"
#include "communication_can.h"

/****************************************************************************/
#define TIM_1_8_CLOCK_HZ 168000000
#define TIM_1_8_PERIOD_CLOCKS 3500
#define TIM_1_8_DEADTIME_CLOCKS 20  //100=595ns
#define TIM_APB1_CLOCK_HZ 84000000
#define TIM_APB1_PERIOD_CLOCKS 4096
#define TIM_APB1_DEADTIME_CLOCKS 40  //50=595ns
#define TIM_1_8_RCR 2

#define GR 8

// Period in [s]
#define CURRENT_MEAS_PERIOD ( (float)2*TIM_1_8_PERIOD_CLOCKS*(TIM_1_8_RCR+1) / (float)TIM_1_8_CLOCK_HZ )
static const float current_meas_period = CURRENT_MEAS_PERIOD;

// Frequency in [Hz]
#define CURRENT_MEAS_HZ ( (float)(TIM_1_8_CLOCK_HZ) / (float)(2*TIM_1_8_PERIOD_CLOCKS*(TIM_1_8_RCR+1)) )
static const int current_meas_hz = CURRENT_MEAS_HZ;
/****************************************************************************/
//错误标志位
#define  ERROR_NONE                             0
#define  ERROR_MODULATION_MAGNITUDE            (1<<0)
#define  ERROR_CURRENT_SENSE_SATURATION        (1<<1)
#define  ERROR_CURRENT_LIMIT_VIOLATION         (1<<2)
#define  ERROR_PHASE_RESISTANCE_OUT_OF_RANGE   (1<<3)
#define  ERROR_UNKNOWN_VBUS_VOLTAGE            (1<<4)
#define  ERROR_NOT_HIGH_CURRENT_MOTOR          (1<<5)
#define  ERROR_UNBALANCED_PHASES               (1<<6)
#define  ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE   (1<<7)
#define  ERROR_UNSTABLE_GAIN                   (1<<8)
#define  ERROR_ABS_SPI_COM_FAIL                (1<<9)
#define  ERROR_UNSUPPORTED_ENCODER_MODE        (1<<10)
#define  ERROR_INDEX_NOT_FOUND_YET             (1<<11)
#define  ERROR_HALL_NOT_CALIBRATED_YET         (1<<12)
#define  ERROR_NO_RESPONSE                     (1<<13)
#define  ERROR_CPR_POLEPAIRS_MISMATCH          (1<<14)
#define  ERROR_INVALID_STATE                   (1<<15)
#define  ERROR_INVALID_CIRCULAR_RANGE          (1<<16)
#define  ERROR_INVALID_INPUT_MODE              (1<<17)
#define  ERROR_OVERSPEED                       (1<<18)
#define  ERROR_SPINOUT_DETECTED                (1<<19)
#define  ERROR_INVALID_ESTIMATE                (1<<20)
#define  ERROR_UNKNOWN_PHASE_VEL               (1<<21)
/****************************************************************************/
//根据驱动板硬件设置参数
//#define VBUS_S_DIVIDER_RATIO   18.73f    //电源分压电阻2.2k+39k
#define VBUS_S_DIVIDER_RATIO   19.0f    //电源分压电阻1k+18k
#define SHUNT_RESISTANCE       0.002f    //采样电阻，如果是0.5mΩ=0.0005f,1mΩ=0.001f
#define PHASE_CURRENT_GAIN     20.0f     //电流采样运放倍数，20倍
/****************************************************************************/
//更改参数，烧写代码前请先清除flash中保存的参数（如果之前保存过的话）
//电机配置参数，根据电机型号选择
#define  MOTOR_type                 MOTOR_TYPE_HIGH_CURRENT   //MOTOR_TYPE_GIMBAL //MOTOR_TYPE_HIGH_CURRENT
#define  MOTOR_pole_pairs                          14    //电机极对数
#define  MOTOR_calibration_current                 2    //校准电流
#define  MOTOR_resistance_calib_max_voltage        6    //校准限制电压
#define  MOTOR_current_lim                        30    //电机最大运行电流

//MT6701采用三线SPI:接CS、SCLK、MISO；不需要接MOSI引脚。
#define  ENCODER_mode        MODE_SPI_AS5047P    //编码器类型，共6种:MODE_INCREMENTAL,MODE_SPI_AS5047P,MODE_SPI_MT6701,MODE_SPI_MA730,MODE_SPI_TLE5012B,MODE_SPI_MT6835
#define  ENCODER_cpr                  16384      //AS5047P=MT6701=MA730=16384,TLE5012B=32768,MT6835=2097152,MODE_INCREMENTAL=4000,
#define  ENCODER_bandwidth             1000      //默认1000，hall电机的cpr比较小，可设置为100

//控制模式，以下参数请熟练官方ODrive操作后再设置，参数的意义和大小与odrivetool中的设置一致
#define  CONTROL_mode        CONTROL_MODE_POSITION_CONTROL  //控制模式:CONTROL_MODE_VOLTAGE_CONTROL,CONTROL_MODE_TORQUE_CONTROL,CONTROL_MODE_VELOCITY_CONTROL,CONTROL_MODE_POSITION_CONTROL
#define  INPUT_mode           INPUT_MODE_TRAP_TRAJ          //输入模式:INPUT_MODE_INACTIVE,INPUT_MODE_PASSTHROUGH,INPUT_MODE_VEL_RAMP,INPUT_MODE_TORQUE_RAMP,INPUT_MODE_POS_FILTER,INPUT_MODE_TRAP_TRAJ,INPUT_MODE_TUNING
#define  TORQUE_ramp_rate                0.1f               //力矩爬升率，力矩模式起作用
#define  VELOCITY_ramp_rate              200                 //加速度，速度模式起作用
#define  POSITION_P                      20                 //位置P参数
#define  VELOCITY_P                      0.04f              //速度P参数
#define  VELOCITY_I                      0.5f               //速度I参数
#define  VELOCITY_limit                  50                 //最大速度限制，力矩模式超过限速力矩会下降
#define  TRAPTraj_vel_limit              40     //梯形轨迹速度限制
#define  TRAPTraj_accel_limit            20     //梯形轨迹加速加速度
#define  TRAPTraj_decel_limit            20     //梯形轨迹减速加速度
#define  ANTIcogging_pos_threshold       72     //抗齿槽校准时位置允许误差  180/3600=0.05  如遇齿槽效应比较大的电机，可放宽误差
#define  ANTIcogging_vel_threshold       72     //抗齿槽校准时速度限制  72/3600=0.02

//以上为常用参数，更多参数设置请到上电初始化的motor_para_init()、MagneticSensor_Init()、controller_para_init() 这几个函数中设置
//其它参数，对于大部分电机来说使用默认即可，无需更改
/****************************************************************************/


#endif


