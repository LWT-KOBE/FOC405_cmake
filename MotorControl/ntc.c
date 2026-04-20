#include "ntc.h"

/**
 * @brief 定义一些常数
 */
#define ABSOLUTE_ZERO_C      (-273.15f)   // 绝对零度（°C）
#define KELVIN_OFFSET        273.15f      // 开尔文偏移量
#define MIN_VALID_TEMP_C     (-50.0f)     // 最小有效温度
#define MAX_VALID_TEMP_C     (150.0f)     // 最大有效温度

/**
 * @brief 初始化NTC参数
 */
void NTC_InitParams(NTC_Params_t *params, 
                    float r_nominal, 
                    float t_nominal, 
                    float b_val, 
                    float r_series, 
                    float v_ref, 
                    uint16_t adc_res)
{
    if (params == NULL) return;
    
    params->nominal_resistance = r_nominal;
    params->nominal_temperature = t_nominal;
    params->b_value = b_val;
    params->series_resistance = r_series;
    params->reference_voltage = v_ref;
    params->adc_resolution = adc_res;
}

/**
 * @brief 计算温度的核心函数
 */
TemperatureResult_t NTC_CalculateTemperature(uint16_t adc_value, const NTC_Params_t *params)
{
    TemperatureResult_t result = {0};
    result.is_valid = 0;  // 默认为无效
    
    if (params == NULL) {
        return result;
    }
    
    // 检查ADC值是否有效
    if (adc_value >= params->adc_resolution) {
        return result;
    }
    
    // 防止除零错误和边界处理
    if (adc_value == 0) {
        // ADC值为0，表示温度非常低或NTC开路
        // 因为NTC在上，ADC测量的是下拉电阻的电压
        // ADC=0 意味着NTC电阻非常大（温度很低）
        result.temperature_c = MIN_VALID_TEMP_C;
        result.temperature_f = NTC_CelsiusToFahrenheit(result.temperature_c);
        result.resistance = INFINITY;  // 近似开路
        result.is_valid = 1;
        return result;
    }
    
    if (adc_value == params->adc_resolution - 1) {
        // ADC值接近最大值，表示温度非常高或NTC短路
        // ADC=最大值 意味着NTC电阻非常小（温度很高）
        result.temperature_c = MAX_VALID_TEMP_C;
        result.temperature_f = NTC_CelsiusToFahrenheit(result.temperature_c);
        result.resistance = 0.0f;  // 近似短路
        result.is_valid = 1;
        return result;
    }
    
    // 电路分析：
    // VCC (3.3V)
    //   |
    //   NTC (热敏电阻)
    //   |
    //   +--- ADC引脚（测量此点电压）
    //   |
    //  下拉电阻 (series_resistance)
    //   |
    //  GND
    
    // 1. 计算ADC引脚电压（NTC与下拉电阻连接点的电压）
    float v_adc = (float)adc_value / params->adc_resolution * params->reference_voltage;
    
    // 2. 计算流过电路的电流
    // 电流 = 下拉电阻两端的电压 / 下拉电阻
    float v_drop_resistor = v_adc;  // 下拉电阻两端的电压等于ADC电压
    float i_circuit = v_drop_resistor / params->series_resistance;
    
    // 3. 计算NTC两端的电压
    float v_ntc = params->reference_voltage - v_adc;
    
    // 4. 计算NTC的电阻值
    // R_ntc = V_ntc / I
    float r_ntc = v_ntc / i_circuit;
    result.resistance = r_ntc;
    
    // 5. 使用Steinhart-Hart方程计算温度
    float t0_kelvin = params->nominal_temperature + KELVIN_OFFSET;
    
    // 防止对数运算的参数为0或负数
    if (r_ntc <= 0.0f || params->nominal_resistance <= 0.0f) {
        return result;
    }
    
    // 计算ln(R/R0)
    float log_r_ratio = logf(r_ntc / params->nominal_resistance);
    
    // 计算倒数温度
    float inv_temperature = 1.0f / t0_kelvin + (1.0f / params->b_value) * log_r_ratio;
    
    // 6. 计算开尔文温度
    float temperature_k = 1.0f / inv_temperature;
    
    // 7. 转换为摄氏度
    result.temperature_c = temperature_k - KELVIN_OFFSET;
    
    // 8. 转换为华氏度
    result.temperature_f = NTC_CelsiusToFahrenheit(result.temperature_c);
    
    // 9. 检查温度是否在合理范围内
    if (result.temperature_c >= MIN_VALID_TEMP_C && result.temperature_c <= MAX_VALID_TEMP_C) {
        result.is_valid = 1;
    }
    
    return result;
}

/**
 * @brief 简化的温度计算函数
 */
float NTC_GetTemperatureC(uint16_t adc_value, const NTC_Params_t *params)
{
    TemperatureResult_t result = NTC_CalculateTemperature(adc_value, params);
    
    if (result.is_valid) {
        return result.temperature_c;
    } else {
        return -1000.0f;  // 返回一个明显错误的温度值
    }
}

/**
 * @brief 摄氏度转华氏度
 */
float NTC_CelsiusToFahrenheit(float celsius)
{
    return celsius * 9.0f / 5.0f + 32.0f;
}

/**
 * @brief 华氏度转摄氏度
 */
float NTC_FahrenheitToCelsius(float fahrenheit)
{
    return (fahrenheit - 32.0f) * 5.0f / 9.0f;
}


//获取温度值
float Get_Temperature(uint16_t value)
{
        float temperature = 0.0f;
        
        //读取通道16内部温度传感器的值
        
        //转化为电压值
        temperature = (float)value * (3.3f / 4096.f);
        
        //转化为温度值
        temperature = (temperature - 0.76f) / 0.0025f + 25.0f;

        return temperature;
}
