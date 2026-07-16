#ifndef __MLX90520_H
#define __MLX90520_H

#include "MyProject.h"

#define MLX90520_CPR_FROM_VDP(vdp)   ((uint32_t)(vdp) * 65536u)

typedef enum {
    MLX90520_OK = 0,
    MLX90520_SPI_ERROR,
    MLX90520_DEV_ID_ERROR,
    MLX90520_FS_ERROR,
    MLX90520_CRC_ERROR,
    MLX90520_ARG_ERROR
} MLX90520_Status_t;

typedef struct {
    uint8_t fs;
    uint16_t fc1;
    uint16_t fc2;
} MLX90520_Frame_t;

void MLX90520_SPI3_Init(void);
MLX90520_Status_t MLX90520_CheckDevice(void);
MLX90520_Status_t MLX90520_StartFrameRead(void);
void MLX90520_StopFrameRead(void);

MLX90520_Status_t MLX90520_ReadFrame2(MLX90520_Frame_t *frame);
MLX90520_Status_t MLX90520_ReadRaw16(uint16_t *raw16);
MLX90520_Status_t MLX90520_ReadRaw22(uint32_t *raw22);

#endif /* __MLX90520_H */
