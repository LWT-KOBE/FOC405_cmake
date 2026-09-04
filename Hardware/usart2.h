
#ifndef __USART2_H
#define __USART2_H
/******************************************************************************/
#include "stm32f4xx.h"
#include "MyProject.h"

enum{
    baudrate_2400 = 0,
    baudrate_4800 = 1,
    baudrate_9600 = 2,
    baudrate_115200 = 3,
    baudrate_230400 = 4,
    baudrate_460800 = 5,
    baudrate_921600 = 6,
};
extern uint32_t usart2_baudrate;
/******************************************************************************/
#define USART2_BUFFER_SIZE 256
/******************************************************************************/
extern char snd2_buff[USART2_BUFFER_SIZE];
extern char rcv2_buff[USART2_BUFFER_SIZE];
extern unsigned long rcv2_cntr;
extern unsigned long rcv2_flag;
/******************************************************************************/
void USART2_Init(unsigned long bound);
void USART2_SendDMA(uint32_t len);
void usart2_send_array(uint8_t *data, uint32_t len);

void uart2_init(u32 bound);
void usart2_set_baud(u8 bound);
void u2_SendByte(uint8_t Byte);
void u2_SendArray(uint8_t *Array, uint16_t Length);
/******************************************************************************/


#endif


