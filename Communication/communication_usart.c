#include "communication_usart.h"


/*****************************************************************************/
/*****************************************************************************/
//串口2通信，排针的GPIO3为TXD2，GPIO4为RXD2，GND共地。
//因为闭环控制时run_closed_loop_control_loop()代码会进入while循环，所以通信指令放在了接收中断里处理，usart2.c文件中
//当电机转速较高时，比如T100，串口通信会影响电机的转动，此时请使用USB通信
void commander_run(void)
{
	switch(rcv2_buff[0])
	{
		case 'H': {
			USART2_SendDMA(sprintf(snd2_buff,"Closeloop2 Encoder:%d\r\n", encoder_config.mode));
		} break;
		
		case 'C': {  //测量电阻电感，3秒后电机“嘀”一声。然后进入校准，正反转8个电角度
			current_state_ = AXIS_STATE_MOTOR_CALIBRATION;
		} break;
		case 'G': {  //闭环控制
			current_state_ = AXIS_STATE_CLOSED_LOOP_CONTROL;
		} break;
		case 'I': {  //空闲模式
			input_pos_ = 0;   //清零输入值
			input_vel_ = 0;
			input_torque_ = 0;
			controller_reset();
			disarm();
			current_state_ = AXIS_STATE_IDLE;
		} break;
		
		case 'D': {  //开始抗齿槽校准
			start_anticogging_calibration();
			USART2_SendDMA(sprintf(snd2_buff,"start anticogging cal\r\n"));
		} break;
		case 'J': {  //读取校准到哪个位置
			USART2_SendDMA(sprintf(snd2_buff,"index=%d\r\n", anticogging.index));
		} break;
		
		case 'S': {  //设置力矩 S0.1
			input_torque_ = atof((const char *)(rcv2_buff+1));
			USART2_SendDMA(sprintf(snd2_buff,"S=%.2f\r\n", input_torque_));
		} break;
		case 'T': {  //设置速度 T10，单位 turn/s
			input_vel_ = atof((const char *)(rcv2_buff+1));
			USART2_SendDMA(sprintf(snd2_buff,"T=%.2f\r\n", input_vel_));
		} break;
		case 'K': {  //设置位置 K2，单位 turn
			input_pos_ = atof((const char *)(rcv2_buff+1));
			USART2_SendDMA(sprintf(snd2_buff,"K=%.2f\r\n", input_pos_));
			input_pos_updated_ = true;  //针对梯形轨迹模式，更新目标位置
		} break;
		case 'V': {  //读取实际速度，单位 turn/s
			USART2_SendDMA(sprintf(snd2_buff,"vel=%.2f\r\n", vel_estimate_));
		} break;
		case 'P': {  //当前绝对位置，单位 turn
			USART2_SendDMA(sprintf(snd2_buff,"pos=%.2f\r\n", pos_estimate_));
		} break;
		case 'Q': {  //读取错误标志
			USART2_SendDMA(sprintf(snd2_buff,"error=%X\r\n", motor_error));
		} break;
		
		case 'A': {  //配置是否已经校准  A1="pre_calibrated = True", A0="pre_calibrated = False",
			encoder_config.pre_calibrated = atoi((const char *)(rcv2_buff+1));
			USART2_SendDMA(sprintf(snd2_buff,"pre_calibrated = %d\r\n", encoder_config.pre_calibrated));
		} break;
		case 'B': {  //配置上电进入闭环模式  B1="startup_closed_loop_control = True", B0="startup_closed_loop_control = False"
			axis_config.startup_closed_loop_control = atoi((const char *)(rcv2_buff+1));
			USART2_SendDMA(sprintf(snd2_buff,"startup_closed_loop_control = %d\r\n", axis_config.startup_closed_loop_control));
		} break;
		case 'F': {  //保存参数
			flash_para_write();  //下载程序是不会擦除这个扇区，一旦保存不用时请及时擦除
			USART2_SendDMA(sprintf(snd2_buff,"flash save!\r\n"));
		} break;
		case 'E': {  //擦除扇区
			disarm();
			flash_para_erase();
			IWDG_Init();//擦除后重启
			USART2_SendDMA(sprintf(snd2_buff,"flash erase!\r\n"));
		} break;
		case 'R': {  //复位
			IWDG_Init();//使能看门狗，等待复位
			USART2_SendDMA(sprintf(snd2_buff,"reset!\r\n"));
		} break;
	}


	memset(rcv2_buff,0,16);  //USART2_BUFFER_SIZE //清空接收数组,长度覆盖接收的字节数即可
}

