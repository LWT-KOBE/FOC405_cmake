#ifndef __KTH7111_H
#define __KTH7111_H

#include "stm32f4xx.h"

typedef struct
{
    uint32_t cpr;
    float sample_hz;
    float bandwidth;
    float pll_kp;
    float pll_ki;
    uint16_t raw_angle;
    uint16_t raw_angle_last;
    int32_t shadow_count;
    int32_t count_in_cpr;
    float pos_estimate_counts;
    float pos_cpr_counts;
    float vel_estimate_counts;
    float interpolation;
    float spi_error_rate;
    int32_t max_delta_per_sample;
    uint8_t first_sample;
    uint8_t data_valid;
} KTH7111_PLL_t;

void KTH7111_PLL_Init(KTH7111_PLL_t *pObj, uint32_t cpr, float sample_hz, float bandwidth);
void KTH7111_PLL_SetSampleHz(KTH7111_PLL_t *pObj, float sample_hz);
void KTH7111_PLL_SetBandwidth(KTH7111_PLL_t *pObj, float bandwidth);
void KTH7111_PLL_SetMaxDelta(KTH7111_PLL_t *pObj, int32_t max_delta_per_sample);
void KTH7111_PLL_Reset(KTH7111_PLL_t *pObj, uint16_t raw_angle);
uint8_t KTH7111_PLL_Update(KTH7111_PLL_t *pObj, uint16_t raw_angle);
uint8_t KTH7111_PLL_SampleAndUpdate(KTH7111_PLL_t *pObj);
uint16_t KTH7111_PLL_GetRawAngle(const KTH7111_PLL_t *pObj);
uint16_t KTH7111_PLL_GetFiltAngle(const KTH7111_PLL_t *pObj);
float KTH7111_PLL_GetAngleTurn(const KTH7111_PLL_t *pObj);
float KTH7111_PLL_GetAccAngleDeg(const KTH7111_PLL_t *pObj);
float KTH7111_PLL_GetAngleDeg(const KTH7111_PLL_t *pObj);
float KTH7111_PLL_GetVelTurn(const KTH7111_PLL_t *pObj);
float KTH7111_PLL_GetVelDeg(const KTH7111_PLL_t *pObj);
float KTH7111_PLL_GetErrorRate(const KTH7111_PLL_t *pObj);

#endif
