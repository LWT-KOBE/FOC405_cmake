#ifndef __NTC_H
#define __NTC_H
#include "MyProject.h"

#include <stdint.h>
#include <math.h>

/**
 * @brief NTC热敏电阻参数结构体
 * @note 需要根据实际使用的NTC型号配置这些参数
 */
typedef struct {
    float nominal_resistance;    // 标称电阻值（通常为10kΩ）
    float nominal_temperature;   // 标称温度（通常为25°C）
    float b_value;               // B值（例如：3950）
    float series_resistance;     // 分压电阻阻值（与NTC串联的电阻）
    float reference_voltage;     // ADC参考电压（单位：V）
    uint16_t adc_resolution;     // ADC分辨率（例如：12位为4095）
} NTC_Params_t;

/**
 * @brief 温度计算结果结构体
 */
typedef struct {
    float temperature_c;         // 摄氏度
    float temperature_f;         // 华氏度
    float resistance;            // 计算出的NTC电阻值
    uint8_t is_valid;            // 数据是否有效标志
} TemperatureResult_t;

/**
 * @brief 初始化NTC参数
 * @param params NTC参数结构体指针
 * @param r_nominal 标称电阻值（Ω）
 * @param t_nominal 标称温度（°C）
 * @param b_val B值
 * @param r_series 分压电阻阻值（Ω）
 * @param v_ref 参考电压（V）
 * @param adc_res ADC分辨率（最大值）
 */
void NTC_InitParams(NTC_Params_t *params, 
                    float r_nominal, 
                    float t_nominal, 
                    float b_val, 
                    float r_series, 
                    float v_ref, 
                    uint16_t adc_res);

/**
 * @brief 根据ADC值计算温度（主函数）
 * @param adc_value ADC采样值
 * @param params NTC参数结构体指针
 * @return 温度计算结果结构体
 */
TemperatureResult_t NTC_CalculateTemperature(uint16_t adc_value, const NTC_Params_t *params);

/**
 * @brief 简化的温度计算函数（只返回摄氏度）
 * @param adc_value ADC采样值
 * @param params NTC参数结构体指针
 * @return 温度值（°C），如果计算失败返回-1000.0
 */
float NTC_GetTemperatureC(uint16_t adc_value, const NTC_Params_t *params);

/**
 * @brief 将摄氏度转换为华氏度
 * @param celsius 摄氏度
 * @return 华氏度
 */
float NTC_CelsiusToFahrenheit(float celsius);

/**
 * @brief 将华氏度转换为摄氏度
 * @param fahrenheit 华氏度
 * @return 摄氏度
 */
float NTC_FahrenheitToCelsius(float fahrenheit);


float Get_Temperature(uint16_t value);



#endif
