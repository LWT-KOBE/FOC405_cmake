#ifndef __SECONDARY_ENCODER_H
#define __SECONDARY_ENCODER_H

#include "stm32f4xx.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
	uint8_t enabled;
	uint8_t mode;
	uint8_t ready;
	uint32_t cpr;
	uint32_t raw;
	uint32_t count_in_cpr;
	float position_turns;
	uint32_t status;
	uint32_t update_count;
	uint32_t error_count;
} SecondaryEncoder_t;

extern SecondaryEncoder_t second_encoder;

void SecondEncoder_Init(void);
bool SecondEncoder_Update(void);
void SecondEncoder_Reset(void);

#endif
