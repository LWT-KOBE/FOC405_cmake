

#include "stm32f4xx.h"


/*****************************************************************************/
void SPI3_Init_(uint16_t spi_cpol)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,ENABLE);
	
	// GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;  //PA0/GPIO1--CS0
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	// GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	// GPIO_InitStructure.GPIO_Speed =GPIO_Speed_25MHz;
	// GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	// GPIO_Init(GPIOA,&GPIO_InitStructure);
	// GPIO_SetBits(GPIOA, GPIO_Pin_0);     //CS0_H

	// 修改为PB3
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;  //PB3/GPIO8--CS0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_3);     //CS0_H

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;//PC10~12
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_SPI3); //PC10 复用为 SPI3
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_SPI3); //PC11 复用为 SPI3
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_SPI3); //PC12 复用为 SPI3
	
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,ENABLE);    //复位 SPI3
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,DISABLE);   //停止复位 SPI3
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//SPI3--双线全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;   //16位数据
	SPI_InitStructure.SPI_CPOL = spi_cpol;               //CPOL=0   SPI_CPOL_High  SPI_CPOL_Low
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;         //CPHA=1
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;            //NSS 信号由硬件管理
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;  //32--1.31MHz   APB1域频率42MHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;   //CRC 值计算的多项式
	SPI_Init(SPI3, &SPI_InitStructure);
	SPI_Cmd(SPI3, ENABLE);
}

void SPI3_Init_MLX90520(void)
{
	GPIO_InitTypeDef gpio;
	SPI_InitTypeDef spi;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

	gpio.GPIO_Pin = GPIO_Pin_3; // PB3 CS
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &gpio);
	GPIO_SetBits(GPIOB, GPIO_Pin_3);

	gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12; // SCK MISO MOSI
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &gpio);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, DISABLE);

	SPI_I2S_DeInit(SPI3);
	spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	spi.SPI_Mode = SPI_Mode_Master;
	spi.SPI_DataSize = SPI_DataSize_16b;
	spi.SPI_CPOL = SPI_CPOL_Low;
	spi.SPI_CPHA = SPI_CPHA_1Edge;
	spi.SPI_NSS = SPI_NSS_Soft;
	spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	spi.SPI_FirstBit = SPI_FirstBit_MSB;
	spi.SPI_CRCPolynomial = 7;
	SPI_Init(SPI3, &spi);
	SPI_Cmd(SPI3, ENABLE);
}

// void SPI3_Init_KTH7112(uint16_t spi_cpol)
// {
// 	GPIO_InitTypeDef GPIO_InitStructure;
// 	SPI_InitTypeDef  SPI_InitStructure;
//
// 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC, ENABLE);
// 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,ENABLE);
//
// 	// GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;  //PA0/GPIO1--CS0
// 	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
// 	// GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
// 	// GPIO_InitStructure.GPIO_Speed =GPIO_Speed_25MHz;
// 	// GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
// 	// GPIO_Init(GPIOA,&GPIO_InitStructure);
// 	// GPIO_SetBits(GPIOA, GPIO_Pin_0);     //CS0_H
//
// 	// 修改为PB3
// 	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;  //PB3/GPIO8--CS0
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
// 	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
// 	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_25MHz;
// 	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
// 	GPIO_Init(GPIOB,&GPIO_InitStructure);
// 	GPIO_SetBits(GPIOB, GPIO_Pin_3);     //CS0_H
//
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_12;//PC10~12
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
// 	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
// 	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
// 	GPIO_Init(GPIOC, &GPIO_InitStructure);
//
// 	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_SPI3); //PC10 复用为 SPI3
// 	// GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_SPI3); //PC11 复用为 SPI3
// 	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_SPI3); //PC12 复用为 SPI3
//
// 	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,ENABLE);    //复位 SPI3
// 	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,DISABLE);   //停止复位 SPI3
//
// 	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;//SPI3--单线半双工
// 	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
// 	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;   //8位数据
// 	SPI_InitStructure.SPI_CPOL = spi_cpol;               //CPOL=0   SPI_CPOL_High  SPI_CPOL_Low
// 	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;         //CPHA=1
// 	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;            //NSS 信号由硬件管理
// 	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;  //32--1.31MHz   APB1域频率42MHz
// 	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
// 	SPI_InitStructure.SPI_CRCPolynomial = 7;   //CRC 值计算的多项式
// 	SPI_Init(SPI3, &SPI_InitStructure);
// 	SPI_Cmd(SPI3, ENABLE);
// }

// 全局变量（给DMA用）
uint8_t KTH7112_RxBuf[3] = {0}; // DMA接收缓存：固定3字节

void SPI3_Init_KTH7112(uint16_t spi_cpol)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;
    DMA_InitTypeDef  DMA_InitStructure;

    // 开时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE); // DMA1时钟

    // ===================== CS 引脚 PB3
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_3); // CS 默认为高

    // ===================== SPI3 GPIO：PC10(SCK) PC12(MOSI)
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);

    // ===================== 复位SPI
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, DISABLE);

    // ===================== SPI 配置：1线 主机 8位 CPOL/CPHA
    SPI_InitStructure.SPI_Direction         = SPI_Direction_1Line_Tx;  // 单线
    SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL              = spi_cpol;
    SPI_InitStructure.SPI_CPHA              = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;  // 你原来的配置
    SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial      = 7;
    SPI_Init(SPI3, &SPI_InitStructure);

    // ===================== 关键：开启 SPI RX DMA
    SPI_DMACmd(SPI3, SPI_DMAReq_Rx, ENABLE);

    // ===================== 使能SPI
    SPI_Cmd(SPI3, ENABLE);

    // ===================== DMA 配置：DMA1 通道0 / Stream2（SPI3_RX 固定）
    DMA_DeInit(DMA1_Stream2);
    while (DMA1_Stream2->CR & DMA_SxCR_EN); // 等待DMA关闭

    DMA_InitStructure.DMA_Channel            = DMA_Channel_0;          // SPI3_RX = DMA1_Stream2_Channel0
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI3->DR;   // 外设地址：SPI DR
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)KTH7112_RxBuf;// 内存地址
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize         = 3;                      // 固定收3字节
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;   // 内存自增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;        // 普通模式
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;  // 最高优先级
    DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold       = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst         = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst     = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream2, &DMA_InitStructure);
}

void SPI3_Init_KTH7112_SSI(uint16_t spi_cpol)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,ENABLE);

	// GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;  //PA0/GPIO1--CS0
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	// GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	// GPIO_InitStructure.GPIO_Speed =GPIO_Speed_25MHz;
	// GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	// GPIO_Init(GPIOA,&GPIO_InitStructure);
	// GPIO_SetBits(GPIOA, GPIO_Pin_0);     //CS0_H

	// 修改为PB3
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;  //PB3/GPIO8--CS0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_3);     //CS0_H

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_12;//PC10~12
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_SPI3); //PC10 复用为 SPI3
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_SPI3); //PC11 复用为 SPI3
	// GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_SPI3); //PC12 复用为 SPI3

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,ENABLE);    //复位 SPI3
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,DISABLE);   //停止复位 SPI3

	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Rx;//SPI3--单线半双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;   //8位数据
	SPI_InitStructure.SPI_CPOL = spi_cpol;               //CPOL=0   SPI_CPOL_High  SPI_CPOL_Low
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;         //CPHA=1
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;            //NSS 信号由硬件管理
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;  //32--1.31MHz   APB1域频率42MHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;   //CRC 值计算的多项式
	SPI_Init(SPI3, &SPI_InitStructure);
	SPI_Cmd(SPI3, ENABLE);
}


void SPI3_ClearOVR_Flag(void)
{
	volatile uint16_t tmp;
	tmp = SPI3->DR;
	tmp = SPI3->SR;
	(void)tmp;
}

void SPI1_ClearOVR_Flag(void)
{
	volatile uint16_t tmp;
	tmp = SPI1->DR;
	tmp = SPI1->SR;
	(void)tmp;
}


void SPI3_Init_KTH7111_SSI(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;      // PC10 = SCK
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;      // PC11 = MISO
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, DISABLE);

	SPI_I2S_DeInit(SPI3);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_RxOnly;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPI3, &SPI_InitStructure);
	SPI_Cmd(SPI3, DISABLE);
}

void SPI1_Init_KTH7111_SSI(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* PA5 -> SCK */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	// GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);

	/* PB4 -> MISO */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	// GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
	// GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
	// GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	// GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	// GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, DISABLE);

	SPI_I2S_DeInit(SPI1);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_RxOnly;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	// SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	// SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	SPI_Cmd(SPI1, DISABLE);
}



/*****************************************************************************/
uint16_t SPIx_ReadWriteByte(uint16_t byte)
{
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI3, byte);
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(SPI3);
}

/******************************************************************************/

