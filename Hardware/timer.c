#include "MyProject.h"
#include "timer.h"
#include "kth7111.h"
/*****************************************************************************/
// volatile uint8_t JS_RTT_BufferUp1[2048] = {0,};
// const uint8_t JS_RTT_Channel = 1;
// typedef struct {
//     volatile float msg1;
//     volatile float msg2;
// 	volatile float msg3;
// 	volatile float msg4;
// }RTT_MSG_U1I1;
// RTT_MSG_U1I1 rtt_JsMsg,rtt_JsMsg2;
// uint16_t kth7112_angle = 0,kth7111_angle = 2;
// float angle_k = 0;
uint16_t kth7112_angle = 0,kth7111_angle = 2;
float angle_k = 0;
// #define TIM7_INTERRUPT_HZ           1000.0f
#define TIM7_INTERRUPT_HZ           4000.0f

// #define KTH7111_PLL_BANDWIDTH_HZ    150.0f
// #define KTH7111_PLL_MAX_DELTA       2500

#define KTH7111_PLL_BANDWIDTH_HZ    80.0f
#define KTH7111_PLL_MAX_DELTA       128

static KTH7111_PLL_t kth7111_pll_;
void TIM1_PWM_Init(void)
{
	NVIC_InitTypeDef          NVIC_InitStructure;
	GPIO_InitTypeDef          GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef         TIM_OCInitStructure;
	TIM_BDTRInitTypeDef       TIM_BDTRInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_TIM1);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	TIM_DeInit(TIM1);
	TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3;
	TIM_TimeBaseInitStructure.TIM_Period = 3500;     //168M/7000=24KHz
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 2;      //重复计数器，每(2+1)次
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;    //使能输出
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;  //使能互补输出
	TIM_OCInitStructure.TIM_Pulse = 1750;                            //值越小，占空比越大
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;        //输出极性为高
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;      //输出互补极性为高
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;     //空闲状态为低
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;   //空闲状态为低
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStructure.TIM_DeadTime = 100;    // 100/168M=595ns
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	
  	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);	  //更新作为触发输出
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);    //更新中断使能
	//TIM_CtrlPWMOutputs(TIM1, ENABLE);  //PWM不输出，发送指令A开始输出
	//TIM_Cmd(TIM1, ENABLE);             //先不使能，等ADC配置完毕后再使能
}
/*****************************************************************************/
//TIM2更新中断触发ADC1的规则转换,1KHz频率
void TIM2_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_TimeBaseInitStructure.TIM_Period = 1000-1;      //1ms
	TIM_TimeBaseInitStructure.TIM_Prescaler=84-1;       //84分频=1MHz
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);	  //更新作为触发输出
	TIM_Cmd(TIM2,ENABLE);
}
/******************************************************************************/
//TIM3用于AB编码器计数
void TIM3_Encoder_Init(void)
{
	GPIO_InitTypeDef         GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
	TIM_ICInitTypeDef        TIM_ICInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;  //M0_ENC_A、M0_ENC_B
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;       //有外部上拉3.3K   
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_TIM3);
  	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_TIM3);
	
  	TIM_TimeBaseInitStructure.TIM_Period = 0xffff;
	TIM_TimeBaseInitStructure.TIM_Prescaler=0;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; //时钟分割
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//计数模式3
	
	TIM_ICStructInit(&TIM_ICInitStructure); 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 4;  //滤波器值
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
  	TIM_SetCounter(TIM3,0); //TIM3->CNT=0
  	TIM_Cmd(TIM3, ENABLE); 
}
/*****************************************************************************/

void TIM3_InputCapture_Config(void){
	TIM_ICInitTypeDef TIM_ICInitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	// 1. 使能 GPIO 和 TIM3 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
    
    // 2. 配置 PB4 (TIM3_CH1), PB5 (TIM3_CH2), PC9 (TIM3_CH4) 为复用功能
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // PB4 (TIM3_CH1)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;        // 复用模式
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      // 推挽输出
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;        // 上拉
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);  // PB4 复用为 TIM3_CH1

    // PB5 (TIM3_CH2)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);  // PB5 复用为 TIM3_CH2

    // PC9 (TIM3_CH4)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);  // PC9 复用为 TIM3_CH4

	// 3. 配置 TIM3 时基（基本定时器设置）
    TIM_TimeBaseStruct.TIM_Prescaler = 0;       // 不分频
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_Period = 0xFFFF;          // 最大计数值
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);

    // 4. 配置输入捕获（以 TIM3_CH1 为例）
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;    // PB4 (TIM3_CH1)
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;  // 上升沿捕获
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;  // 直接输入
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;  // 无分频
    TIM_ICInitStruct.TIM_ICFilter = 0x0;             // 无滤波
    TIM_ICInit(TIM3, &TIM_ICInitStruct);

    TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;    // PB5 (TIM3_CH2)
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;  // 上升沿捕获
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;  // 直接输入
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;  // 无分频
    TIM_ICInitStruct.TIM_ICFilter = 0x0;             // 无滤波
    TIM_ICInit(TIM3, &TIM_ICInitStruct);

	TIM_ICInitStruct.TIM_Channel = TIM_Channel_4;    // PC9 (TIM3_CH4)
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;  // 上升沿捕获
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;  // 直接输入
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;  // 无分频
    TIM_ICInitStruct.TIM_ICFilter = 0x0;             // 无滤波
    TIM_ICInit(TIM3, &TIM_ICInitStruct);
    // 6. 启动 TIM3
    TIM_Cmd(TIM3, ENABLE);

}

// 定时器7
void TIM7_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	
	// TIM_TimeBaseInitStructure.TIM_Period = 1000-1;      //1ms

	TIM_TimeBaseInitStructure.TIM_Period = 250-1;      //1ms
	TIM_TimeBaseInitStructure.TIM_Prescaler=84-1;       //84分频=1MHz
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM7,ENABLE);
}
void KTH7111_TIM7_PLL_Init(void)
{
	KTH7111_PLL_Init(&kth7111_pll_, 65536, TIM7_INTERRUPT_HZ, KTH7111_PLL_BANDWIDTH_HZ);
	KTH7111_PLL_SetMaxDelta(&kth7111_pll_, KTH7111_PLL_MAX_DELTA);
}
int time_cnt = 0;
int enc_cnt = 0;
volatile uint64_t can_cnt = 0;
volatile uint8_t flash_flag = 0;
float temperature_motor, temperature_board;
static volatile uint8_t tim7_temperature_pending = 0u;
static volatile uint8_t tim7_can_pending = 0u;
static volatile uint8_t tim7_vofa_pending = 0u;



// 1. 定义NTC参数（根据实际硬件配置）
NTC_Params_t ntc_params_motor, ntc_params_board;

static void TIM7_InitNtcParamsOnce(void)
{
	static uint8_t ntc_params_ready = 0u;

	if (ntc_params_ready) {
		return;
	}

	NTC_InitParams(&ntc_params_motor, 10000.0f, 25.0f, 3950.0f, 3300.0f, 3.3f, 4095);
	NTC_InitParams(&ntc_params_board, 10000.0f, 25.0f, 3950.0f, 3300.0f, 3.3f, 4095);
	ntc_params_ready = 1u;
}

void TIM7_LowPriorityService(void)
{
	TIM7_InitNtcParamsOnce();

	if (tim7_temperature_pending) {
		tim7_temperature_pending = 0u;
		temperature_board = NTC_GetTemperatureC(adc1_value[0], &ntc_params_board);
		temperature_motor = NTC_GetTemperatureC(adc1_value[1], &ntc_params_motor);
	}

	if (flash_flag == 1u) {
		flash_para_write();
		flash_flag = 0u;
	}

	if (tim7_can_pending) {
		tim7_can_pending = 0u;
		OD_CANSendData_2(CAN1, OD_CANID, MSG_GET_ENCODER_ESTIMATES, 8, pos_estimate_, vel_estimate_, &ODSendData);
		OD_CANSendData_2(CAN1, OD_CANID, MSG_GET_IQ, 8, Idq_setpoint_.q, Iq_measured, &ODSendData);
		OD_CANSendData_2(CAN1, OD_CANID, MSG_GET_BUS_VOLTAGE_CURRENT, 8, vbus_voltage, Ibus, &ODSendData);
	}

	if (tim7_vofa_pending && (current_state_ == AXIS_STATE_CLOSED_LOOP_CONTROL)) {
		tim7_vofa_pending = 0u;
		vofaFrame.fdata[0] = vel_estimate_;
		vofaFrame.fdata[1] = input_vel_;
		vofaFrame.fdata[2] = pos_estimate_;
		vofaFrame.fdata[3] = input_pos_;
		vofaFrame.fdata[4] = encoder_config.pre_calibrated;
		vofaFrame.fdata[5] = current_meas_.phA;
		vofaFrame.fdata[6] = current_meas_.phB;
		vofaFrame.fdata[7] = current_meas_.phC;
		vofaFrame.fdata[8] = Ibus;
		vofaFrame.fdata[9] = vbus_voltage;
		vofaFrame.fdata[10] = Iq_measured;
		vofaFrame.fdata[11] = Idq_setpoint_.q;
		vofaFrame.fdata[12] = temperature_board;
		vofaFrame.fdata[13] = temperature_motor;
		vofaFrame.fdata[14] = Id_measured;
		vofaFrame.fdata[15] = Idq_setpoint_src_->d;
		vofaFrame.fdata[16] = pos_val;
		vofa_printf_USB();
	}
}

void TIM7_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{	
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		tim7_temperature_pending = 1u;

		#if 0
		// if(c == 0){
		//     c = 1;
		// 	// SEGGER_RTT_ConfigUpBuffer(1,                  // 通道号
  //           //                 // 通道名字（命名有意义的，一定要按照官方文档“RTT channel naming convention”的规范来）
  //           //                 "JScope_f4f4f4f4",              // 数据包含1个32位的时间戳与1个uint32_t变量、1个uint32_t变量
  //           //                 (uint8_t*)&JS_RTT_BufferUp1[0], // 缓存地址
  //           //                 sizeof(JS_RTT_BufferUp1),       // 缓存大小
  //           //                 SEGGER_RTT_MODE_NO_BLOCK_SKIP); // 非阻塞
		// 	//SEGGER_RTT_ConfigUpBuffer(JS_RTT_Channel, "JScope_t4i4i4", &JS_RTT_UpBuff[0], sizeof(JS_RTT_UpBuff), SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
  //
		//
		//
			// 典型10K NTC (B=3950) 配置
			// 串联电阻: 3.3kΩ
			// ADC参考电压: 3.3V
			// ADC分辨率: 12位 (0-4095)
			NTC_InitParams(&ntc_params_motor,
						10000.0f,   // 标称电阻 10kΩ @25°C
						25.0f,      // 标称温度 25°C
						3950.0f,    // B值 3950
						3300.0f,   // 串联电阻 3.3kΩ
						3.3f,       // 参考电压 3.3V
						4095);      // 12位ADC分辨率
			NTC_InitParams(&ntc_params_board,
						10000.0f,   // 标称电阻 10kΩ @25°C
						25.0f,      // 标称温度 25°C
						3950.0f,    // B值 3950
						3300.0f,   // 串联电阻 3.3kΩ
						3.3f,       // 参考电压 3.3V
						4095);      // 12位ADC分辨率
		// }
		temperature_board = NTC_GetTemperatureC(adc1_value[0], &ntc_params_board);
		temperature_motor = NTC_GetTemperatureC(adc1_value[1], &ntc_params_motor);
	    // if (flash_flag == 1)
		if (flash_flag == 1)
		{
			flash_para_write();
			flash_flag = 0;
		}
		#endif

		#if (ENCODER_mode != MODE_HALL)
		KTH7111_PLL_SampleAndUpdate(&kth7111_pll_);
		kth7111_angle = KTH7111_PLL_GetFiltAngle(&kth7111_pll_);
		angle_k = KTH7111_PLL_GetAccAngleDeg(&kth7111_pll_) / 2.0f;
		#else
		kth7111_angle = 0u;
		angle_k = 0.0f;
		#endif

		// kth7111_angle = KTH7111_ReadSSIAngle();
		// angle_k = (float)kth7111_angle/65535.0f * 360.0f;

		// rtt_JsMsg.msg1 = pos_estimate_;
		// rtt_JsMsg.msg2 = vbus_voltage;
		// rtt_JsMsg.msg3 = vel_estimate_;
		// rtt_JsMsg.msg4 = input_vel_;
		// SEGGER_RTT_Write(JS_RTT_Channel, &rtt_JsMsg, sizeof(rtt_JsMsg));	/* 上传数据 */
		
		time_cnt++;
		//enc_cnt = HALL_GETState();
		can_cnt++;
		if ((can_cnt % 10u) == 0u) {
			tim7_can_pending = 1u;
		}

		// kth7111_angle = KTH7111_ReadSSIAngle();
		// angle_k = (float)kth7111_angle/65535.0f * 360.0f;
		// IAP CAN升级
		//IAP_Send_Device_ino();
		
		
		if(time_cnt > 1){
			time_cnt = 0;
			tim7_vofa_pending = 1u;
			#if 0
//			// 进入闭环控制状态才发送
			if(current_state_ == AXIS_STATE_CLOSED_LOOP_CONTROL){
				vofaFrame.fdata[0] = vel_estimate_;
				vofaFrame.fdata[1] = input_vel_;

				vofaFrame.fdata[2] = pos_estimate_;
				vofaFrame.fdata[3] = input_pos_;
				vofaFrame.fdata[4] = encoder_config.pre_calibrated;
				vofaFrame.fdata[5] = current_meas_.phA;
				vofaFrame.fdata[6] = current_meas_.phB;
				vofaFrame.fdata[7] = current_meas_.phC;
				vofaFrame.fdata[8] = Ibus;
				vofaFrame.fdata[9] = vbus_voltage;
				vofaFrame.fdata[10] = Iq_measured;
				vofaFrame.fdata[11] = Idq_setpoint_.q;
				vofaFrame.fdata[12] = temperature_board; // 板载MOS管温度值
				vofaFrame.fdata[13] = temperature_motor; // 电机NTC温度值
				vofaFrame.fdata[14] = Id_measured; //
				vofaFrame.fdata[15] = Idq_setpoint_src_->d;
				vofaFrame.fdata[16] = pos_val;
				vofa_printf_USB();
			}
			#endif
		}
		// TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		// 在这里添加定时器7的中断处理代码
	}
}
