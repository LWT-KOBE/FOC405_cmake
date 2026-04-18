#ifndef __VOFA_H
#define __VOFA_H 			   
#include "stm32f4xx.h"

#define CH_COUNT 20

typedef union
{
    float fdata;
    unsigned long ldata;
} FloatLongType;

struct Frame
{
	float fdata[CH_COUNT];
	unsigned char tail[4];
};

struct Frame_Send
{
	float fdata[40];
	unsigned char tail[4];
};

extern volatile struct Frame_Send DeviceFrame;
extern volatile struct Frame vofaFrame;


void RawData_Test(void);
void FireWater_Test(void);
void Float_to_Byte(float f,unsigned char byte[]);
void Byte_to_Float(float *f,unsigned char byte[]);
void JustFloat_Test(void);

void Float_to_Byte(float f,unsigned char byte[]);
void Vofa_sendData(float Byte);
void vofa_printf_USB(void);
void vofa_printf(void);
void vofa_sendData(float a, float b, float c, float d, float e, float f, float g, float h, float j, float k, float l, float o, float p, float i);
#endif





























