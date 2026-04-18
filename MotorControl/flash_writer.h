#ifndef __FLASH_WRITER_H
#define __FLASH_WRITER_H

#include "stm32f4xx.h"


/*****************************************************************************/
extern  uint32_t flash_reg[128];
/*****************************************************************************/
uint32_t float2uint(float var);
float uint2float(uint32_t var);
void flash_para_read(void);
void flash_para_write(void);
void flash_para_erase(void);
/*****************************************************************************/

#endif


