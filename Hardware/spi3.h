
#ifndef __SPI3_H
#define __SPI3_H

/******************************************************************************/
#include "stm32f4xx.h"

/******************************************************************************/
void SPI3_Init_(uint16_t spi_cpol);
uint16_t SPIx_ReadWriteByte(uint16_t byte);
void SPI3_Init_KTH7112(uint16_t spi_cpol);
extern uint8_t KTH7112_RxBuf[3];
/******************************************************************************/


#endif
