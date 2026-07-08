
#include "MyProject.h"
volatile uint8_t JS_RTT_BufferUp1[2048] = {0,};
const uint8_t JS_RTT_Channel = 1;
typedef struct {
    volatile uint32_t timestamp;
    volatile float msg2;
	volatile float msg3;
	volatile float msg4;
}RTT_MSG_U1I1;
RTT_MSG_U1I1 rtt_JsMsg,rtt_JsMsg2;

/****************************************************************************/
#define  CURRENT_SENSE_MIN_VOLT  0.3f
// #define  CURRENT_SENSE_MIN_VOLT  0.0f
#define  CURRENT_SENSE_MAX_VOLT  3.0f
#define  CURRENT_ADC_LOWER_BOUND  (uint32_t)((float)(1 << 12) * CURRENT_SENSE_MIN_VOLT / 3.3f)
#define  CURRENT_ADC_UPPER_BOUND  (uint32_t)((float)(1 << 12) * CURRENT_SENSE_MAX_VOLT / 3.3f)
/****************************************************************************/
float  vbus_voltage=12.0f;
Iph_ABC_t  current0;
/****************************************************************************/
float phase_current_from_adcval(uint32_t ADCValue)
{
	if (ADCValue < CURRENT_ADC_LOWER_BOUND || ADCValue > CURRENT_ADC_UPPER_BOUND)
	{
		set_error(ERROR_CURRENT_SENSE_SATURATION);
		return 0;
	}
	
	int adcval_bal = (int)ADCValue - (1 << 11);
	float amp_out_volt = (3.3f / (float)(1 << 12)) * (float)adcval_bal;
	float shunt_volt = amp_out_volt * (1/PHASE_CURRENT_GAIN);   //phase_current_rev_gain_ = 1/20倍放大
	float current = shunt_volt * (1/SHUNT_RESISTANCE);          //shunt_conductance_ = 1/0.001采样电阻;
	return current;
}
/****************************************************************************/
void vbus_sense_adc_cb(uint32_t adc_value) 
{
	float voltage_scale = 3.3f * VBUS_S_DIVIDER_RATIO / 4096;
	vbus_voltage = adc_value * voltage_scale;
}
/****************************************************************************/
uint16_t ib_val,ic_val;
//在TIM1的更新中断函数中被调用
uint8_t fetch_and_reset_adcs(Iph_ABC_t *current)
{
	uint8_t all_adcs_done = (((ADC1->SR & ADC_SR_JEOC) == ADC_SR_JEOC) && ((ADC2->SR & ADC_SR_JEOC) == ADC_SR_JEOC) && ((ADC3->SR & ADC_SR_JEOC) == ADC_SR_JEOC));
	if(!all_adcs_done)return 0;

	vbus_sense_adc_cb(ADC1->JDR1);
	//
	// if (current_state_ == AXIS_STATE_MOTOR_CALIBRATION || current_state_ == AXIS_STATE_ENCODER_OFFSET_CALIBRATION)
	// {
	// 	current->phB = -phase_current_from_adcval(ADC2->JDR1);
	// 	current->phC = -phase_current_from_adcval(ADC3->JDR1);
	// 	current->phA = -current->phB - current->phC;
	// }else
	// {
	// 	current->phB = phase_current_from_adcval(ADC2->JDR1);
	// 	current->phC = phase_current_from_adcval(ADC3->JDR1);
	// 	current->phA = -current->phB - current->phC;
	// }
	// current->phB = phase_current_from_adcval(ADC2->JDR1);
	// current->phC = phase_current_from_adcval(ADC3->JDR1);

	// current->phB = -phase_current_from_adcval(ADC2->JDR1);
	// current->phC = -phase_current_from_adcval(ADC3->JDR1);
	// current->phA = current->phB + current->phC;

	// 修改电流方向相反
	current->phB = -phase_current_from_adcval(ADC2->JDR1);
	current->phC = -phase_current_from_adcval(ADC3->JDR1);
	current->phA = -current->phB - current->phC;

	ib_val = ADC2->JDR1;
	ic_val = ADC3->JDR1;

	ADC1->SR = ~(ADC_SR_JEOC);
	ADC2->SR = ~(ADC_SR_JEOC | ADC_SR_OVR);
	ADC3->SR = ~(ADC_SR_JEOC | ADC_SR_OVR);
	
	return 1;
}
/****************************************************************************/
#define LED_blink   GPIOD->ODR^=(1<<2)  //PD2
/****************************************************************************/
uint32_t time_cntr=0;
volatile uint32_t timestamp_ = 0;
// uint16_t kth7112_angle = 0,kth7111_angle = 2;
// float angle_k = 0;

//中断频率16KHz，进入中断的同时触发ADC
void TIM1_UP_TIM10_IRQHandler(void)
{
	uint32_t i;
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	timestamp_ += TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1);
	
//	if(++time_cntr >= 8000)  //0.5s   指示灯放在了抗齿槽演示，controller.c第325行
//	{
//		time_cntr = 0;
//		LED_blink;
//	}
	// Scheduler_Tick_Handler(); // 调度器滴答处理函数，每625us调用一次

	// OD_CANSendData_2(CAN1,OD_CANID,MSG_GET_ENCODER_ESTIMATES,8,pos_estimate_,vel_estimate_,&ODSendData);
	// OD_CANSendData_2(CAN1,OD_CANID,MSG_GET_IQ,8,Idq_setpoint_.q,Iq_measured,&ODSendData);

	uint8_t counting_down = TIM1->CR1 & TIM_CR1_DIR;
	
	if(!counting_down)   //=0为递增计数,上臂为低下臂为高,此时采样
	{
		sample_now();                        //读取角度
		// kth7111_angle = KTH7111_ReadSSIAngle();
		// angle_k = (float)kth7111_angle/65535.0f * 360.0f;
		fetch_and_reset_adcs(&current0);     //电流采样，获得的采样值在current0
		current_meas_cb(timestamp_, &current0);  //传入采样值并运算
		control_loop_cb();
	}
	else
	{
		for(i=0;i<20;i++);                   //延时，同时等待ADC完毕
		fetch_and_reset_adcs(&current0);     //电流采样
		dc_calib_cb(&current0);  //timestamp_ + TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1), 
		pwm_update_cb();         //timestamp_ + 3 * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1);
	}

	// if(time_cntr == 0){
	// 	    time_cntr = 1;
	// 		SEGGER_RTT_ConfigUpBuffer(1,                  // 通道号
    //                        // 通道名字（命名有意义的，一定要按照官方文档“RTT channel naming convention”的规范来）
    //                        "JScope_t4f4f4f4",              // 数据包含1个32位的时间戳与1个uint32_t变量、1个uint32_t变量
    //                        (uint8_t*)&JS_RTT_BufferUp1[0], // 缓存地址
    //                        sizeof(JS_RTT_BufferUp1),       // 缓存大小
    //                        SEGGER_RTT_MODE_NO_BLOCK_SKIP); // 非阻塞
	// 		//SEGGER_RTT_ConfigUpBuffer(JS_RTT_Channel, "JScope_t4i4i4", &JS_RTT_UpBuff[0], sizeof(JS_RTT_UpBuff), SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
	// 	}
		

	// 	rtt_JsMsg.timestamp = DWT_Get_Microsecond(); // 从dwt定时器获取时间戳;
	// 	rtt_JsMsg.msg2 = current_meas_.phA;
	// 	rtt_JsMsg.msg3 = current_meas_.phB;
	// 	rtt_JsMsg.msg4 = current_meas_.phC;

	// 	// rtt_JsMsg.msg2 = ADC1->JDR1; //board temperature adc
	// 	// rtt_JsMsg.msg3 = ADC2->JDR1;
	// 	// rtt_JsMsg.msg4 = ADC3->JDR1;
	// 	SEGGER_RTT_Write(JS_RTT_Channel, &rtt_JsMsg, sizeof(rtt_JsMsg));	/* 上传数据 */
	
	// if(current_state_ == AXIS_STATE_CLOSED_LOOP_CONTROL){
	// 	vofaFrame.fdata[0] = vel_estimate_;
	// 	vofaFrame.fdata[1] = input_vel_;
	//
	// 	vofaFrame.fdata[2] = pos_estimate_;
	// 	vofaFrame.fdata[3] = input_pos_;
	// 	vofaFrame.fdata[4] = encoder_config.pre_calibrated;
	// 	vofaFrame.fdata[5] = current_meas_.phA;
	// 	vofaFrame.fdata[6] = current_meas_.phB;
	// 	vofaFrame.fdata[7] = current_meas_.phC;
	// 	vofaFrame.fdata[8] = Ibus;
	// 	vofaFrame.fdata[9] = vbus_voltage;
	// 	vofaFrame.fdata[10] = Iq_measured;
	// 	vofaFrame.fdata[11] = Idq_setpoint_.q;
	// 	vofaFrame.fdata[12] = temperature_board; // 板载MOS管温度值
	// 	vofaFrame.fdata[13] = temperature_motor; // 电机NTC温度值
	// 	// vofaFrame.fdata[14] = Get_Temperature(adc1_value[4]); // ADC原始值
	// 	// vofaFrame.fdata[15] = adc1_value[4]; // ADC原始值
	// 		vofa_printf_USB();
	// 	}
	
}

/****************************************************************************/
uint32_t print_flag;

void control_loop_cb(void)
{
	encoder_update();
	controller_update();
	openloop_controller_update();
	motor_update();
	foc_update();
	
	if((motor_error!=0)&&(print_flag==0))
	{
		print_flag=1;  //只打印一次
		disarm();
		
		// USART2_SendDMA(sprintf(snd2_buff,"error:0x%X\r\n", motor_error));      //串口打印
		// uint32_t len=sprintf((char *)usb_sndbuff,"error:0x%X\r\n", motor_error); //虚拟串口打印
		// usb_send(usb_sndbuff, len);
	}
}
/*****************************************************************************/
//擦除指令，擦除扇区后复位
//看门狗使用32KHz时钟，32分频，计数500，大概0.5秒复位
void IWDG_Init(void)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //取消寄存器写保护
	IWDG_SetPrescaler(IWDG_Prescaler_32); //设置 IWDG 分频系数
	IWDG_SetReload(500);     //设置 IWDG 装载值
	IWDG_ReloadCounter();    //reload
	IWDG_Enable();           //使能看门狗
}

/****************************************************************************/



