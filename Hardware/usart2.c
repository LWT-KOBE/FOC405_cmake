
#include <stdio.h>
#include "usart2.h"

u8 usart2_baudrate = 3; //usart2波特率
/********************************************************************/
char snd2_buff[USART2_BUFFER_SIZE];
char rcv2_buff[USART2_BUFFER_SIZE];
unsigned long rcv2_cntr;
unsigned long rcv2_flag;
/******************************************************************************/
//加入以下代码，支持printf函数，而不需要选择MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;
//定义_sys_exit避免使用半主机模式
void _sys_exit(int x) 
{
	x = x; 
} 
//重定义fputc函数
int fputc(int ch, FILE *f)
{
	while((USART2->SR&0x40)==0);
	USART2->DR = (u8) ch;
	return ch;
}
#endif  
/********************************************************************/
void USART2_Init(unsigned long bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	
	//RCC_Config
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_DMA1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
	//GPIO_AF_Map
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
	
	//GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;     //上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//NVIC
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
// 	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
//   //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//   //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//   NVIC_Init(&NVIC_InitStructure);
	
	//USART2
	USART_DeInit(USART2);
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2,&	USART_InitStructure);
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);   //DMA收
	USART_Cmd(USART2,ENABLE);
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);   //接收空闲中断
	
	// // DMA Tx
	// DMA_DeInit(DMA1_Stream6);
	// DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	// DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART2->DR;
	// DMA_InitStructure.DMA_Memory0BaseAddr = (u32)snd2_buff;
	// DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	// DMA_InitStructure.DMA_BufferSize = USART2_BUFFER_SIZE;
	// DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	// DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	// DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	// DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	// DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	// DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA_Priority_Medium; //
	// DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	// DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	// DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	// DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	// DMA_Init(DMA1_Stream6, &DMA_InitStructure);
	// //DMA_Cmd(DMA1_Stream6,ENABLE);   //直到发送的时候再使能
	// DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE); 
	
	// DMA Rec
	DMA_DeInit(DMA1_Stream5);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART2->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)rcv2_buff;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = USART2_BUFFER_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream5,ENABLE);
}

void usart2_set_baud(u8 bound){
	switch (bound)
	{
	case baudrate_2400:
		USART2_Init(2400);
		/* code */
		break;
	case baudrate_4800:
		USART2_Init(4800);
		break;
	case baudrate_9600:
		USART2_Init(9600);
		break;
	case baudrate_115200:
		USART2_Init(115200);
		break;
	case baudrate_230400:
		USART2_Init(230400);
		break;

	case baudrate_460800:
		USART2_Init(460800);
		break;

	case baudrate_921600:
		USART2_Init(921600);
		break;

	default:
		break;
	}
}
/**************************************************************************
Function: Serial port 2 initialization
Input   : none
Output  : none
函数功能：串口2初始化
入口参数：无
返回  值：无
**************************************************************************/
void uart2_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 	//Enable the Usart clock //使能USART时钟
    // 使能DMA时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);	//TX
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);	//RX 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;            //上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);  		          //初始化
	
	//UsartNVIC configuration //UsartNVIC配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQ通道使能	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
	//Initialize the VIC register with the specified parameters 
	//根据指定的参数初始化VIC寄存器		
	NVIC_Init(&NVIC_InitStructure);
	
	//USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //收发模式
	USART_Init(USART2, &USART_InitStructure);      //Initialize serial port 2 //初始化串口2
	
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
	
    USART_ITConfig(USART2,USART_IT_TC,DISABLE); 
    USART_ITConfig(USART2,USART_IT_RXNE,DISABLE); 
    USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);//开启串口空闲中断	
	
	USART_Cmd(USART2, ENABLE);                     //Enable serial port 2 //使能串口2 
}


/********************************************************************/
void USART2_SendDMA(uint32_t len)
{
	DMA_Cmd(DMA1_Stream6,DISABLE);                         //关闭DMA传输
	//while(DMA_GetCmdStatus(DMA1_Stream6) != DISABLE){}     //确保DMA可以被设置
	DMA_SetCurrDataCounter(DMA1_Stream6, len);             //设置传输字节数
	DMA_Cmd(DMA1_Stream6,ENABLE);                         
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);         //开启DMA传输
}

void u2_SendByte(uint8_t Byte)		//发送一个字节数据
{
	USART_SendData(USART2, Byte);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

void u2_SendArray(uint8_t *Array, uint16_t Length)	//发送一个8位的数组
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		u2_SendByte(Array[i]);
	}
}

// 发送数组数据
void usart2_send_array(uint8_t *data, uint32_t len)
{
	if (len > USART2_BUFFER_SIZE) {
		len = USART2_BUFFER_SIZE;  //防止溢出
	}
	
	for (uint32_t i = 0; i < len; i++) {
		snd2_buff[i] = data[i];
	}
	
	USART2_SendDMA(len);
}

/********************************************************************
 * USART2 接收空闲中断服务函数 
********************************************************************/
extern  void commander_run(void);

void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2,USART_IT_IDLE) == SET)
	{
		rcv2_cntr=USART2->SR;
		rcv2_cntr=USART2->DR;     //清除中断标志USART_ClearITPendingBit(USART2,USART_IT_RXNE)
		rcv2_cntr=USART2_BUFFER_SIZE-DMA_GetCurrDataCounter(DMA1_Stream5);  //读取接收字节个数
		DMA_Cmd(DMA1_Stream5,DISABLE);                           //关闭DMA传输
		while(DMA_GetCmdStatus(DMA1_Stream5) != DISABLE);        //确保DMA可以被设置
		DMA_SetCurrDataCounter(DMA1_Stream5,USART2_BUFFER_SIZE); //数据传输量
		DMA_Cmd(DMA1_Stream5,ENABLE);                            //开启DMA传输
		
		commander_run();    //USART2的通信处理
	}
}
/********************************************************************
 * USART2 DMA发送中断服务函数 
********************************************************************/
void DMA1_Stream6_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream6,DMA_IT_TCIF6) == SET)
	{
		DMA_ClearITPendingBit(DMA1_Stream6,DMA_IT_TCIF6);
		DMA_Cmd(DMA1_Stream6,DISABLE);   //本次发送完成，关闭DMA，下次使用再打开
	}
}
/********************************************************************/



