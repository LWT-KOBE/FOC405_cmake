#include "hall_encoder.h"
#include <string.h>

#define HALL_DEFAULT_SAMPLE_HZ 10000u

HallEncoder_t g_hall_encoder;

/* 001, 011, 010, 110, 100, 101 */
static const int8_t hall_table[8] = {
    -1, 0, 2, 1, 4, 5, 3, -1
};

static int32_t mod_positive(int32_t value, int32_t modulo)
{
    int32_t ret = value % modulo;
    return (ret < 0) ? (ret + modulo) : ret;
}

static uint16_t hall_cpr(const HallEncoder_t *enc)
{
    return enc->config.pole_pairs * 6u;
}

static uint8_t read_hall_state(const HallEncoder_t *enc)
{
    uint8_t state = 0;

    if (GPIO_ReadInputDataBit(enc->config.hall_a.port, enc->config.hall_a.pin)) {
        state |= 0x01;
    }
    if (GPIO_ReadInputDataBit(enc->config.hall_b.port, enc->config.hall_b.pin)) {
        state |= 0x02;
    }
    if (GPIO_ReadInputDataBit(enc->config.hall_c.port, enc->config.hall_c.pin)) {
        state |= 0x04;
    }

    return state ^ enc->config.polarity_xor;
}

static void enable_gpio_clock(GPIO_TypeDef *port)
{
    if (port == GPIOA) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    else if (port == GPIOB) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    else if (port == GPIOC) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    else if (port == GPIOD) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
}

static void hall_gpio_init(HallGpio_t gpio)
{
    GPIO_InitTypeDef init;

    enable_gpio_clock(gpio.port);

    GPIO_StructInit(&init);
    init.GPIO_Pin = gpio.pin;
    init.GPIO_Mode = GPIO_Mode_IN;
    init.GPIO_PuPd = GPIO_PuPd_NOPULL;
    init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(gpio.port, &init);
}

void HallEncoder_Init(HallEncoder_t *enc, const HallEncoderConfig_t *config)
{
    memset(enc, 0, sizeof(*enc));
    enc->config = *config;

    if (enc->config.pole_pairs == 0) {
        enc->config.pole_pairs = 1;
    }

    if (enc->config.sample_hz == 0) {
        enc->config.sample_hz = HALL_DEFAULT_SAMPLE_HZ;
    }

    hall_gpio_init(enc->config.hall_a);
    hall_gpio_init(enc->config.hall_b);
    hall_gpio_init(enc->config.hall_c);

    HallEncoder_Reset(enc);
}

void HallEncoder_Reset(HallEncoder_t *enc)
{
    enc->hall_state = read_hall_state(enc);
    enc->hall_sector = 0;
    enc->initialized = 0;
    enc->error = 0;
    enc->direction = 0;
    enc->count = 0;
    enc->count_in_cpr = 0;
    enc->position_turns = 0.0f;
    enc->velocity_turns_per_sec = 0.0f;
    enc->samples_since_edge = 0;
}

void HallEncoder_Update(HallEncoder_t *enc)
{
    int8_t sector;
    int32_t delta;
    uint16_t cpr = hall_cpr(enc);

    enc->hall_state = read_hall_state(enc);
    sector = hall_table[enc->hall_state & 0x07];

    if (sector < 0) {
        enc->error = 1;
        enc->velocity_turns_per_sec = 0.0f;
        return;
    }

    enc->error = 0;
    enc->hall_sector = (uint8_t)sector;

    if (!enc->initialized) {
        enc->initialized = 1;
        enc->count = sector;
        enc->count_in_cpr = sector;
        enc->position_turns = (float)enc->count / (float)cpr;
        return;
    }

    delta = sector - mod_positive(enc->count_in_cpr, 6);
    delta = mod_positive(delta, 6);

    if (delta > 3) {
        delta -= 6;
    }

    if (delta == 0) {
        if (enc->samples_since_edge < 0xffffffffu) {
            enc->samples_since_edge++;
        }
        enc->velocity_turns_per_sec *= 0.98f;
        return;
    }

    if (delta == 3 || delta == -3) {
        enc->error = 1;
        enc->velocity_turns_per_sec = 0.0f;
        return;
    }

    enc->count += delta;
    enc->count_in_cpr = mod_positive(enc->count_in_cpr + delta, cpr);
    enc->position_turns = (float)enc->count / (float)cpr;
    enc->direction = (delta > 0) ? 1 : -1;

    if (enc->samples_since_edge > 0) {
        float dt = (float)(enc->samples_since_edge + 1u) / enc->config.sample_hz;
        enc->velocity_turns_per_sec = ((float)delta / (float)cpr) / dt;
    }

    enc->samples_since_edge = 0;
}

static uint32_t tim3_clock_hz(void)
{
    RCC_ClocksTypeDef clocks;
    uint32_t tim_clk;

    RCC_GetClocksFreq(&clocks);
    tim_clk = clocks.PCLK1_Frequency;

    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_HCLK_Div1) {
        tim_clk *= 2u;
    }

    return tim_clk;
}

void HallEncoder_TIM3_Init(uint32_t sample_hz)
{
    TIM_TimeBaseInitTypeDef tim;
    NVIC_InitTypeDef nvic;
    uint32_t timer_clk;
    uint16_t prescaler;
    uint16_t period;

    if (sample_hz == 0) {
        sample_hz = HALL_DEFAULT_SAMPLE_HZ;
    }

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    timer_clk = tim3_clock_hz();
    prescaler = (uint16_t)((timer_clk / 1000000u) - 1u);
    period = (uint16_t)((1000000u / sample_hz) - 1u);

    TIM_TimeBaseStructInit(&tim);
    tim.TIM_Prescaler = prescaler;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = period;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &tim);

    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel = TIM3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

void HallEncoder_TIM3_Start(void)
{
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
}

void HallEncoder_TIM3_Stop(void)
{
    TIM_Cmd(TIM3, DISABLE);
}

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        HallEncoder_Update(&g_hall_encoder);
    }
}

