#include <hall_encoder.h>

#include <string.h>

#include <utils.h>

#define HALL_DEFAULT_SAMPLE_HZ  10000u
#define HALL_DEFAULT_BANDWIDTH  100.0f

HallEncoder_t g_hall_encoder;
static uint16_t hall_port_samples_[3];
static const GPIO_TypeDef *hall_ports_to_sample[3] = {GPIOA, GPIOB, GPIOC};

static const float hall_edge_defaults[6] = {
    0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f
};

/* 001, 011, 010, 110, 100, 101 */
static const int8_t hall_table[8] = {
    -1, 0, 2, 1, 4, 5, 3, -1
};

static uint16_t hall_cpr(const HallEncoder_t *enc)
{
    return enc->config.pole_pairs * 6u;
}

static int32_t mod_positive(int32_t value, int32_t modulo)
{
    int32_t ret = value % modulo;
    return (ret < 0) ? (ret + modulo) : ret;
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
    init.GPIO_PuPd = GPIO_PuPd_UP;
    init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(gpio.port, &init);
}

static uint8_t read_raw_hall_state(const HallEncoder_t *enc)
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

    return state;
}

static uint8_t read_sampled_gpio(GPIO_TypeDef *port, uint16_t pin)
{
    uint32_t i;

    for (i = 0u; i < 3u; ++i) {
        if (hall_ports_to_sample[i] == port) {
            return (hall_port_samples_[i] & pin) ? 1u : 0u;
        }
    }

    return 0u;
}

void HallEncoder_SampleNow(HallEncoder_t *enc)
{
    uint32_t i;
    uint8_t state = 0u;

    for (i = 0u; i < 3u; ++i) {
        hall_port_samples_[i] = (uint16_t)hall_ports_to_sample[i]->IDR;
    }

    if (read_sampled_gpio(enc->config.hall_a.port, enc->config.hall_a.pin)) {
        state |= 0x01u;
    }
    if (read_sampled_gpio(enc->config.hall_b.port, enc->config.hall_b.pin)) {
        state |= 0x02u;
    }
    if (read_sampled_gpio(enc->config.hall_c.port, enc->config.hall_c.pin)) {
        state |= 0x04u;
    }

    enc->raw_hall_state = state;
    enc->hall_state = state ^ enc->config.polarity_xor;
}

static void hall_update_pll_gains(HallEncoder_t *enc)
{
    enc->pll_kp = 2.0f * enc->config.bandwidth;
    enc->pll_ki = 0.25f * enc->pll_kp * enc->pll_kp;
}

static int32_t hall_model(const HallEncoder_t *enc, float internal_pos)
{
    int32_t base_cnt = (int32_t)floorf(internal_pos);
    float pos_in_range = fmodf_pos(internal_pos, 6.0f);
    int pos_idx = (int)pos_in_range;
    int next_idx;
    float below_edge;
    float above_edge;

    if (pos_idx >= 6) {
        pos_idx = 5;
    }

    next_idx = (pos_idx == 5) ? 0 : (pos_idx + 1);
    below_edge = enc->edge_pos[pos_idx];
    above_edge = enc->edge_pos[next_idx];

    if (wrap_pm(pos_in_range - below_edge, 6.0f) < 0.0f) {
        return base_cnt - 1;
    }

    if (wrap_pm(pos_in_range - above_edge, 6.0f) > 0.0f) {
        return base_cnt + 1;
    }

    return base_cnt;
}

void HallEncoder_Init(HallEncoder_t *enc, const HallEncoderConfig_t *config)
{
    uint32_t i;

    memset(enc, 0, sizeof(*enc));
    enc->config = *config;

    if (enc->config.pole_pairs == 0u) {
        enc->config.pole_pairs = 1u;
    }

    if (enc->config.sample_hz == 0u) {
        enc->config.sample_hz = HALL_DEFAULT_SAMPLE_HZ;
    }

    if (enc->config.bandwidth <= 0.0f) {
        enc->config.bandwidth = HALL_DEFAULT_BANDWIDTH;
    }

    hall_gpio_init(enc->config.hall_a);
    hall_gpio_init(enc->config.hall_b);
    hall_gpio_init(enc->config.hall_c);

    for (i = 0u; i < 6u; ++i) {
        enc->edge_pos[i] = hall_edge_defaults[i];
    }

    hall_update_pll_gains(enc);
    HallEncoder_Reset(enc);
}

void HallEncoder_Reset(HallEncoder_t *enc)
{
    uint32_t i;

    enc->raw_hall_state = read_raw_hall_state(enc);
    enc->hall_state = enc->raw_hall_state ^ enc->config.polarity_xor;
    enc->hall_sector = 0u;
    enc->initialized = 0u;
    enc->ready = 0u;
    enc->error = 0u;
    enc->direction = 0;
    enc->count = 0;
    enc->shadow_count = 0;
    enc->count_in_cpr = 0;
    enc->delta_enc = 0;
    enc->pos_estimate_counts = 0.0f;
    enc->pos_cpr_counts = 0.0f;
    enc->vel_estimate_counts = 0.0f;
    enc->interpolation = 0.5f;
    for (i = 0; i < 6u; ++i) {
        if (!isfinite(enc->edge_pos[i])) {
            enc->edge_pos[i] = hall_edge_defaults[i];
        }
    }
    enc->position_turns = 0.0f;
    enc->velocity_turns_per_sec = 0.0f;
    enc->samples_since_edge = 0u;
    enc->update_count = 0u;
    enc->edge_count = 0u;
    enc->illegal_state_count = 0u;
    enc->illegal_delta_count = 0u;
    enc->large_delta_count = 0u;
    enc->max_abs_delta = 0u;
}

uint8_t HallEncoder_Update(HallEncoder_t *enc)
{
    int8_t sector;
    int32_t delta;
    uint16_t cpr = hall_cpr(enc);
    float dt = 1.0f / (float)enc->config.sample_hz;
    float delta_pos_counts;
    float delta_pos_cpr_counts;
    uint8_t snap_to_zero_vel = 0u;
    uint32_t abs_delta;

    enc->update_count++;

    sector = hall_table[enc->hall_state & 0x07];

    if (sector < 0) {
        enc->illegal_state_count++;
        enc->error = 1u;
        enc->direction = 0;
        enc->delta_enc = 0;
        if (!enc->config.ignore_illegal_state) {
            enc->vel_estimate_counts = 0.0f;
            enc->velocity_turns_per_sec = 0.0f;
            return 0u;
        }
        return 1u;
    }

    enc->error = 0u;
    enc->hall_sector = (uint8_t)sector;

    if (!enc->initialized) {
        enc->initialized = 1u;
        enc->ready = 1u;
        enc->count = sector;
        enc->shadow_count = sector;
        enc->count_in_cpr = sector;
        enc->delta_enc = 0;
        enc->pos_estimate_counts = (float)sector;
        enc->pos_cpr_counts = (float)sector;
        enc->vel_estimate_counts = 0.0f;
        enc->interpolation = 0.5f;
        enc->position_turns = (float)sector / (float)cpr;
        enc->velocity_turns_per_sec = 0.0f;
        enc->samples_since_edge = 0u;
        return 1u;
    }

    delta = sector - mod_positive(enc->count_in_cpr, 6);
    delta = mod_positive(delta, 6);
    if (delta > 3) {
        delta -= 6;
    }

    enc->delta_enc = delta;
    abs_delta = (delta < 0) ? (uint32_t)(-delta) : (uint32_t)delta;
    if (abs_delta > enc->max_abs_delta) {
        enc->max_abs_delta = abs_delta;
    }
    if (abs_delta > 1u) {
        enc->large_delta_count++;
    }

    if ((delta == 3) || (delta == -3)) {
        enc->illegal_delta_count++;
        enc->error = 1u;
        enc->direction = 0;
        if (!enc->config.ignore_illegal_state) {
            enc->vel_estimate_counts = 0.0f;
            enc->velocity_turns_per_sec = 0.0f;
            return 0u;
        }
        return 1u;
    }

    if (delta == 0) {
        if (enc->samples_since_edge < 0xffffffffu) {
            enc->samples_since_edge++;
        }
    } else {
        enc->samples_since_edge = 0u;
        enc->edge_count++;
        enc->direction = (delta > 0) ? 1 : -1;
    }

    enc->count += delta;
    enc->shadow_count += delta;
    enc->count_in_cpr = mod_positive(enc->count_in_cpr + delta, cpr);

    enc->pos_estimate_counts += dt * enc->vel_estimate_counts;
    enc->pos_cpr_counts += dt * enc->vel_estimate_counts;

    delta_pos_counts = (float)(enc->shadow_count - hall_model(enc, enc->pos_estimate_counts));
    delta_pos_cpr_counts = (float)(enc->count_in_cpr - hall_model(enc, enc->pos_cpr_counts));
    delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, (float)cpr);

    enc->pos_estimate_counts += dt * enc->pll_kp * delta_pos_counts;
    enc->pos_cpr_counts += dt * enc->pll_kp * delta_pos_cpr_counts;
    enc->pos_cpr_counts = fmodf_pos(enc->pos_cpr_counts, (float)cpr);
    enc->vel_estimate_counts += dt * enc->pll_ki * delta_pos_cpr_counts;

    if (fabsf(enc->vel_estimate_counts) < (0.5f * dt * enc->pll_ki)) {
        enc->vel_estimate_counts = 0.0f;
        snap_to_zero_vel = 1u;
    }

    if (snap_to_zero_vel || !enc->config.enable_phase_interpolation) {
        enc->interpolation = 0.5f;
    } else if (delta > 0) {
        enc->interpolation = 0.0f;
    } else if (delta < 0) {
        enc->interpolation = 1.0f;
    } else {
        enc->interpolation += dt * enc->vel_estimate_counts;
        if (enc->interpolation > 1.0f) {
            enc->interpolation = 1.0f;
        }
        if (enc->interpolation < 0.0f) {
            enc->interpolation = 0.0f;
        }
    }

    enc->position_turns = enc->pos_estimate_counts / (float)cpr;
    enc->velocity_turns_per_sec = enc->vel_estimate_counts / (float)cpr;
    return 1u;
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

    if (sample_hz == 0u) {
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
