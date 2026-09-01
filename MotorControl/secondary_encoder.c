#include "MyProject.h"
#include "mlx90520.h"

#if SECOND_ENCODER_ENABLE

SecondaryEncoder_t second_encoder = {
	.enabled = 1u,
	.mode = SECOND_ENCODER_mode,
	.ready = 0u,
	.cpr = SECOND_MLX90520_CPR,
	.raw = 0u,
	.count_in_cpr = 0u,
	.position_turns = 0.0f,
	.status = 0u,
	.update_count = 0u,
	.error_count = 0u,
};

static uint32_t second_encoder_expand_raw(uint32_t raw22)
{
	return raw22 % second_encoder.cpr;
}

void SecondEncoder_Reset(void)
{
	second_encoder.ready = 0u;
	second_encoder.raw = 0u;
	second_encoder.count_in_cpr = 0u;
	second_encoder.position_turns = 0.0f;
	second_encoder.status = 0u;
	second_encoder.update_count = 0u;
	second_encoder.error_count = 0u;
}

void SecondEncoder_Init(void)
{
	if (!second_encoder.enabled) {
		return;
	}

	if (second_encoder.mode != MODE_SPI_MLX90520) {
		second_encoder.enabled = 0u;
		return;
	}

	MLX90520_SPI3_Init();
	delay_us(3000);
	SecondEncoder_Reset();
	second_encoder.status = (uint32_t)MLX90520_StartFrameRead();
}

bool SecondEncoder_Update(void)
{
	uint32_t raw22 = 0u;
	MLX90520_Status_t st;

	if (!second_encoder.enabled) {
		return false;
	}

	st = MLX90520_ReadRaw22(&raw22);
	second_encoder.status = (uint32_t)st;
	second_encoder.update_count++;

	if (st != MLX90520_OK) {
		second_encoder.ready = 0u;
		second_encoder.error_count++;
		return false;
	}

	second_encoder.raw = raw22;
	second_encoder.count_in_cpr = second_encoder_expand_raw(raw22);
	second_encoder.position_turns = (float)second_encoder.count_in_cpr / (float)second_encoder.cpr;
	second_encoder.ready = 1u;
	return true;
}

#else

SecondaryEncoder_t second_encoder = {0};

void SecondEncoder_Reset(void) {}
void SecondEncoder_Init(void) {}
bool SecondEncoder_Update(void) { return false; }

#endif
