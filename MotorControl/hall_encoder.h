//
// Created by BrainCo on 2026/7/10.
//


#ifndef HALL_ENCODER_H
#define HALL_ENCODER_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include <stdint.h>

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} HallGpio_t;

typedef struct {
    HallGpio_t hall_a;
    HallGpio_t hall_b;
    HallGpio_t hall_c;
    uint16_t pole_pairs;
    uint32_t sample_hz;
    uint8_t polarity_xor;
    uint8_t ignore_illegal_state;
    uint8_t enable_phase_interpolation;
    float bandwidth;
} HallEncoderConfig_t;

typedef struct {
    HallEncoderConfig_t config;
    volatile uint8_t raw_hall_state;
    volatile uint8_t hall_state;
    volatile uint8_t hall_sector;
    volatile uint8_t initialized;
    volatile uint8_t ready;
    volatile uint8_t error;
    volatile int8_t direction;
    volatile int32_t count;
    volatile int32_t shadow_count;
    volatile int32_t count_in_cpr;
    volatile int32_t delta_enc;
    volatile float pll_kp;
    volatile float pll_ki;
    volatile float pos_estimate_counts;
    volatile float pos_cpr_counts;
    volatile float vel_estimate_counts;
    volatile float interpolation;
    volatile float edge_pos[6];
    volatile float position_turns;
    volatile float velocity_turns_per_sec;
    volatile uint32_t samples_since_edge;
    volatile uint32_t update_count;
    volatile uint32_t edge_count;
    volatile uint32_t illegal_state_count;
    volatile uint32_t illegal_delta_count;
    volatile uint32_t large_delta_count;
    volatile uint32_t max_abs_delta;
} HallEncoder_t;

extern HallEncoder_t g_hall_encoder;

void HallEncoder_Init(HallEncoder_t *enc, const HallEncoderConfig_t *config);
void HallEncoder_SampleNow(HallEncoder_t *enc);
uint8_t HallEncoder_Update(HallEncoder_t *enc);
void HallEncoder_Reset(HallEncoder_t *enc);

void HallEncoder_TIM3_Init(uint32_t sample_hz);
void HallEncoder_TIM3_Start(void);
void HallEncoder_TIM3_Stop(void);

#endif
