#ifndef DWT_H
#define DWT_H
#include "MyProject.h"


#ifdef __cplusplus
extern "C" {
#endif


// public
u8 DWT_Timer_Init(void);
uint32_t DWT_Get_CNT(void);
void DWT_Delay_us(uint32_t us);
uint32_t DWT_Get_Microsecond(void);
uint32_t DWT_Get_System_Clock_Freq(void);
void DWT_Init(void);
uint32_t Measure_Function_Time(void (*func)(void));
    
// private


#ifdef __cplusplus
} 


#endif
#endif

