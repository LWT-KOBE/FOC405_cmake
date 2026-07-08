#include "MyProject.h"

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;
/****************************************************************************/
uint32_t timecntr_pre=0;
extern  bool dc_calib_valid;
void motor_controller_task(void);

void motor_controller_task(void)
{
	run_state_machine_loop();
	// 在这里添加电机控制相关的代码
	// 例如：读取传感器数据，计算控制输出，更新PWM等
}


/*****************************************************************************/
/******************************************************************************/
/* us计时，每71.5分钟溢出循环一次 */
uint32_t timecount(void)
{
	uint32_t  diff,now_us;

	now_us = micros();    //0xFFFFFFFF=4294967295 us=71.5分钟
	if(now_us>=timecntr_pre)diff = now_us - timecntr_pre;   //us
	else
		diff = 0xFFFFFFFF - timecntr_pre + now_us;
	timecntr_pre = now_us;

	return diff;
}


// uint16_t kth7112_angle = 0,kth7111_angle = 2;
// float angle_k = 0;
/*****************************************************************************/
//支持USB通信和USART2通信，
//USART2接线：GPIO3接USB转串口的RXD，GPIO4接USB转串口的TXD，GND共地
/*****************************************************************************/
int main(void)
{
	//IAP_APP_Init();              //初始化IAP
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	USART2_Init(115200);        //排针的GPIO3为TXD2，GPIO4为RXD2，GND共地
	usart2_set_baud(baudrate_115200);  //设置波特率
	SPI1_Init_KTH7111_SSI();
	tim14_InitTick();           //1ms中断初始化，为系统提供计时
	TIM1_PWM_Init();            //M0接口PWM配置，但没有使能
	ADC_Common_Init();          //初始化ADC的引脚和通用配置，设置中断
	ADC1_DMA_Init();            //配置注入组，检测vbus，等待TIM1触发
	ADC2_TRGO_Init();           //配置注入组，检测m0_phB，等待TIM1触发
	ADC3_TRGO_Init();           //配置注入组，检测m0_phC，等待TIM1触发
	TIM2_Init();                //触发ADC1的规则转换
	TIM7_Init();

	CAN1_Set_BaudRate(OD_CAN_BaudRate); //CAN波特率设置
	// CAN1_Set_BaudRate(2); //CAN波特率设置
	//参数设置在MyProject.h中，更多参数设置请进入以下几个函数中设置
	motor_para_init();
	motor_setup();
	MagneticSensor_Init();
	// SPI3_Init_KTH7111_SSI();
	// SPI1_Init_KTH7111_SSI();
	controller_config_default();//电机控制参数上电默认值
	trapTraj_config_default();  //梯形轨迹参数上电默认值
	sCurveTraj_config_default(); //S型轨迹参数上电默认值
	controller_para_init();     //配置控制参数
	anticogging_init();         //抗齿槽参数默认值
	flash_para_read();          //读取参数
	// SPI3_Init_KTH7112(SPI_CPOL_Low);       //初始化KTH7112
	// SPI3_Init_KTH7112(SPI_CPOL_High);
	 if(encoder_config.pre_calibrated && axis_config.startup_closed_loop_control)current_state_ = AXIS_STATE_CLOSED_LOOP_CONTROL;  //如果已经校准并且配置上电闭环，设置闭环模式

	 delay_us(500000);
	 // USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID,&USR_desc, &USBD_CDC_cb, &USR_cb);  //USB转串口,COM174
	 TIM_Cmd(TIM1, ENABLE);   //TIM1触发中断，AD开始转换。但是PWM还没有输出

	 for(uint32_t i=0; i<2000; i++)   //最多等待2秒。main.cpp文件第565行
	 {
	 	if(dc_calib_valid)break;       //等待电流校准完成后退出
	 	delay_us(1000);
	 }

	 USART2_SendDMA(sprintf(snd2_buff,"Motor Ready!\r\n"));
	// 在MyProject.h中设置电机参数和控制模式。参数设置与官方代码一致，请先熟悉官方odrivetool的操作

	 DWT_Timer_Init();               //初始化DWT计时器
	while(1)
	{
		run_state_machine_loop();
		// kth7112_angle = KTH7112_ReadAngle();
		// KTH7111_ReadSSIAngle_SPI3(&kth7111_angle);
		// kth7111_angle = KTH7111_ReadSSIAngle();
		// kth7111_angle = KTH7111_ReadSSIAngle();
		// angle_k = (float)kth7111_angle/65535.0f * 360.0f;
		delay_us(1000);     //1ms，延时增加通信的可靠性

	}
}





