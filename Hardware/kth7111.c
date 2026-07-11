#include "MyProject.h"
#include "kth7111.h"

static void KTH7111_PLL_UpdateGains(KTH7111_PLL_t *pObj)
{
    pObj->pll_kp = 2.0f * pObj->bandwidth;
    pObj->pll_ki = 0.25f * pObj->pll_kp * pObj->pll_kp;
}

static float KTH7111_GetSamplePeriod(const KTH7111_PLL_t *pObj)
{
    if (pObj->sample_hz <= 0.0f)
    {
        return 0.0f;
    }

    return 1.0f / pObj->sample_hz;
}

static int32_t KTH7111_UnwrapDelta(uint16_t angle_now, uint16_t angle_last)
{
    int32_t delta = (int32_t)angle_now - (int32_t)angle_last;

    if (delta > 32768)
    {
        delta -= 65536;
    }
    else if (delta < -32768)
    {
        delta += 65536;
    }



    return delta;
}

void KTH7111_PLL_Init(KTH7111_PLL_t *pObj, uint32_t cpr, float sample_hz, float bandwidth)
{
    memset(pObj, 0, sizeof(KTH7111_PLL_t));

    pObj->cpr = cpr;
    pObj->sample_hz = sample_hz;
    pObj->bandwidth = bandwidth;
    pObj->interpolation = 0.5f;
    pObj->max_delta_per_sample = 2500;
    pObj->first_sample = 1;
    pObj->data_valid = 0;

    KTH7111_PLL_UpdateGains(pObj);
}

void KTH7111_PLL_SetSampleHz(KTH7111_PLL_t *pObj, float sample_hz)
{
    pObj->sample_hz = sample_hz;
}

void KTH7111_PLL_SetBandwidth(KTH7111_PLL_t *pObj, float bandwidth)
{
    pObj->bandwidth = bandwidth;
    KTH7111_PLL_UpdateGains(pObj);
}

void KTH7111_PLL_SetMaxDelta(KTH7111_PLL_t *pObj, int32_t max_delta_per_sample)
{
    pObj->max_delta_per_sample = max_delta_per_sample;
}

void KTH7111_PLL_Reset(KTH7111_PLL_t *pObj, uint16_t raw_angle)
{
    pObj->raw_angle = raw_angle;
    pObj->raw_angle_last = raw_angle;
    pObj->shadow_count = (int32_t)raw_angle;
    pObj->count_in_cpr = (int32_t)raw_angle;
    pObj->pos_estimate_counts = (float)raw_angle;
    pObj->pos_cpr_counts = (float)raw_angle;
    pObj->vel_estimate_counts = 0.0f;
    pObj->interpolation = 0.5f;
    pObj->spi_error_rate = 0.0f;
    pObj->first_sample = 0;
    pObj->data_valid = 1;
}

uint8_t KTH7111_PLL_Update(KTH7111_PLL_t *pObj, uint16_t raw_angle)
{
    float sample_period;
    int32_t delta_raw;
    int32_t delta_enc;
    float delta_pos_counts;
    float delta_pos_cpr_counts;
    uint8_t snap_to_zero_vel = 0;

    sample_period = KTH7111_GetSamplePeriod(pObj);
    if (sample_period <= 0.0f)
    {
        return 0;
    }

    pObj->raw_angle = raw_angle;

    if (pObj->first_sample)
    {
        KTH7111_PLL_Reset(pObj, raw_angle);
        return 1;
    }

    delta_raw = KTH7111_UnwrapDelta(raw_angle, pObj->raw_angle_last);
    if ((pObj->max_delta_per_sample > 0) &&
        ((delta_raw > pObj->max_delta_per_sample) || (delta_raw < -pObj->max_delta_per_sample)))
    {
        pObj->pos_estimate_counts += sample_period * pObj->vel_estimate_counts;
        pObj->pos_cpr_counts += sample_period * pObj->vel_estimate_counts;
        pObj->pos_cpr_counts = fmodf_pos(pObj->pos_cpr_counts, (float)pObj->cpr);
        pObj->spi_error_rate += sample_period * (1.0f - pObj->spi_error_rate);
        pObj->data_valid = 0;
        return 0;
    }

    pObj->raw_angle_last = raw_angle;
    pObj->spi_error_rate += sample_period * (0.0f - pObj->spi_error_rate);
    pObj->data_valid = 1;

    delta_enc = (int32_t)raw_angle - pObj->count_in_cpr;
    delta_enc = mod(delta_enc, pObj->cpr);
    if (delta_enc > (pObj->cpr / 2))
    {
        delta_enc -= pObj->cpr;
    }

    pObj->shadow_count += delta_enc;
    pObj->count_in_cpr += delta_enc;
    pObj->count_in_cpr = mod(pObj->count_in_cpr, pObj->cpr);
    pObj->count_in_cpr = (int32_t)raw_angle;

    pObj->pos_estimate_counts += sample_period * pObj->vel_estimate_counts;
    pObj->pos_cpr_counts += sample_period * pObj->vel_estimate_counts;

    delta_pos_counts = (float)(pObj->shadow_count - (int32_t)pObj->pos_estimate_counts);
    delta_pos_cpr_counts = (float)(pObj->count_in_cpr - (int32_t)pObj->pos_cpr_counts);
    delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, (float)pObj->cpr);

    pObj->pos_estimate_counts += sample_period * pObj->pll_kp * delta_pos_counts;
    pObj->pos_cpr_counts += sample_period * pObj->pll_kp * delta_pos_cpr_counts;
    pObj->pos_cpr_counts = fmodf_pos(pObj->pos_cpr_counts, (float)pObj->cpr);
    pObj->vel_estimate_counts += sample_period * pObj->pll_ki * delta_pos_cpr_counts;

    if (fabsf(pObj->vel_estimate_counts) < 0.5f * sample_period * pObj->pll_ki)
    {
        pObj->vel_estimate_counts = 0.0f;
        snap_to_zero_vel = 1;
    }

    if (snap_to_zero_vel)
    {
        pObj->interpolation = 0.5f;
    }
    else if (delta_enc > 0)
    {
        pObj->interpolation = 0.0f;
    }
    else if (delta_enc < 0)
    {
        pObj->interpolation = 1.0f;
    }
    else
    {
        pObj->interpolation += sample_period * pObj->vel_estimate_counts;
        if (pObj->interpolation > 1.0f)
        {
            pObj->interpolation = 1.0f;
        }
        if (pObj->interpolation < 0.0f)
        {
            pObj->interpolation = 0.0f;
        }
    }

    return 1;
}

uint8_t KTH7111_PLL_SampleAndUpdate(KTH7111_PLL_t *pObj)
{
    return KTH7111_PLL_Update(pObj, KTH7111_ReadSSIAngle());
}

uint16_t KTH7111_PLL_GetRawAngle(const KTH7111_PLL_t *pObj)
{
    return pObj->raw_angle;
}

uint16_t KTH7111_PLL_GetFiltAngle(const KTH7111_PLL_t *pObj)
{
    int32_t angle = (int32_t)(pObj->pos_cpr_counts);
    angle = mod(angle, pObj->cpr);
    return (uint16_t)angle;
}

float KTH7111_PLL_GetAngleTurn(const KTH7111_PLL_t *pObj)
{
    return pObj->pos_estimate_counts / (float)pObj->cpr;
}

float KTH7111_PLL_GetAccAngleDeg(const KTH7111_PLL_t *pObj)
{
    return KTH7111_PLL_GetAngleTurn(pObj) * 360.0f;
}

float KTH7111_PLL_GetAngleDeg(const KTH7111_PLL_t *pObj)
{
    return (pObj->pos_cpr_counts / (float)pObj->cpr) * 360.0f;
}

float KTH7111_PLL_GetVelTurn(const KTH7111_PLL_t *pObj)
{
    return pObj->vel_estimate_counts / (float)pObj->cpr;
}

float KTH7111_PLL_GetVelDeg(const KTH7111_PLL_t *pObj)
{
    return (pObj->vel_estimate_counts / (float)pObj->cpr) * 360.0f;
}

float KTH7111_PLL_GetErrorRate(const KTH7111_PLL_t *pObj)
{
    return pObj->spi_error_rate;
}
