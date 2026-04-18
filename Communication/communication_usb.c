#include "communication_usb.h"
uint8_t usb_recbuff[256];
uint8_t usb_sndbuff[256];
uint32_t usb_rcv_count;

/*****************************************************************************/
//USB虚拟串口通信
//放在USB接收中断里处理 App/usbd_cdc_vcp.c文件中的VCP_DataRx()函数中

u8 send_mode(void){
	if(ctrl_config.control_mode == CONTROL_MODE_POSITION_CONTROL && ctrl_config.input_mode == INPUT_MODE_TRAP_TRAJ){
		return 0; //位置梯形模式
	}else if(ctrl_config.control_mode == CONTROL_MODE_POSITION_CONTROL && ctrl_config.input_mode == INPUT_MODE_POS_FILTER){
		return 1; //位置滤波模式
	}else if(ctrl_config.control_mode == CONTROL_MODE_POSITION_CONTROL && ctrl_config.input_mode == INPUT_MODE_PASSTHROUGH){
		return 2; //位置直通模式
	}else if(ctrl_config.control_mode == CONTROL_MODE_VELOCITY_CONTROL && ctrl_config.input_mode == INPUT_MODE_VEL_RAMP){
		return 3; //速度斜坡模式
	}else if(ctrl_config.control_mode == CONTROL_MODE_VELOCITY_CONTROL && ctrl_config.input_mode == INPUT_MODE_PASSTHROUGH){
		return 4; //速度直通模式
	}else if(ctrl_config.control_mode == CONTROL_MODE_TORQUE_CONTROL && ctrl_config.input_mode == INPUT_MODE_TORQUE_RAMP){
		return 5; //力矩斜坡模式
	}else if(ctrl_config.control_mode == CONTROL_MODE_TORQUE_CONTROL && ctrl_config.input_mode == INPUT_MODE_PASSTHROUGH){
		return 6; //力矩直通模式
	}
}

void Send_Configuration(void){
	DeviceFrame.fdata[0] = OD_CANID;  //CAN的ID
	DeviceFrame.fdata[1] = OD_CAN_BaudRate;  //CAN的波特率
	DeviceFrame.fdata[2] = usart2_baudrate;  //usart2波特率
	DeviceFrame.fdata[3] = ctrl_config.control_mode;         //控制模式
	DeviceFrame.fdata[4] = ctrl_config.input_mode;             //输入模式
	DeviceFrame.fdata[5] = ctrl_config.torque_ramp_rate; //Nm / sec，力矩爬升率
	DeviceFrame.fdata[6] = ctrl_config.vel_ramp_rate;  //速度的爬升率
	DeviceFrame.fdata[7] = ctrl_config.pos_gain;		//位置P参数
	DeviceFrame.fdata[8] = ctrl_config.vel_gain;               //速度P参数
	DeviceFrame.fdata[9] = ctrl_config.vel_integrator_gain;    //速度I参数
	DeviceFrame.fdata[10] = ctrl_config.vel_limit;          //最大转速限制，圈/秒
	DeviceFrame.fdata[11] = ctrl_config.input_filter_bandwidth;     // [1/s]
	DeviceFrame.fdata[12] = trapTraj_config.vel_limit;      //梯形轨迹最大速度; // [turn/s]
	DeviceFrame.fdata[13] = trapTraj_config.accel_limit;  //梯形轨迹加速度; // [turn/s^2]
	DeviceFrame.fdata[14] = trapTraj_config.decel_limit;  //梯形轨迹减速度; // [turn/s^2]
	DeviceFrame.fdata[15] = motor_config.pole_pairs;  //电机极对数
	DeviceFrame.fdata[16] = motor_config.calibration_current;    // 校准电流[A]
	DeviceFrame.fdata[17] = motor_config.resistance_calib_max_voltage; // 校准电压[V]
	DeviceFrame.fdata[18] = motor_config.phase_resistance;        // 相电阻
	DeviceFrame.fdata[19] = motor_config.phase_inductance;	//相电感   如果是云台电机设置为0
	DeviceFrame.fdata[20] = motor_config.torque_constant;        // 力矩常数[Nm/A]
	DeviceFrame.fdata[21] = motor_config.motor_type;  		 //电机类型
	DeviceFrame.fdata[22] = motor_config.current_lim; //[A] 电机最大运行电流
	DeviceFrame.fdata[23] = motor_config.current_lim_margin;      // Maximum violation of current_lim
	DeviceFrame.fdata[24] = motor_config.current_control_bandwidth;  //电流环控制带宽 [rad/s]
	DeviceFrame.fdata[25] = encoder_config.mode;  //选择编码器型号，参数
	DeviceFrame.fdata[26] = encoder_config.cpr;    //编码器cpr
	DeviceFrame.fdata[27] = encoder_config.bandwidth;   //编码器带宽
	DeviceFrame.fdata[28] = encoder_config.pre_calibrated;                //是否已经校准
	DeviceFrame.fdata[29] = encoder_config.direction; // 电机方向

	DeviceFrame.fdata[30] = encoder_config.phase_offset;                  //偏移值 整数部分
	DeviceFrame.fdata[31] = encoder_config.phase_offset_float;//偏移值 小数部分
	DeviceFrame.fdata[32] = axis_config.startup_closed_loop_control;      //是否上电后进入闭环
	DeviceFrame.fdata[33] = anticogging_valid_;                           //抗齿槽校准数据是否正常标志位
	DeviceFrame.fdata[34] = send_mode();         //控制模式
	DeviceFrame.fdata[35] = current_state_;


	// usb_send((uint8_t *) (&DeviceFrame), sizeof(DeviceFrame));

}
void USBcommander_run(void)
{
	uint32_t len;
	
	if(usb_rcv_count != 0)
	{
		//当接收到符合格式的字符串，或者接收达到数组上限，就进行字符串数据的处理
		if(usb_rcv_count > 256-1 || usb_recbuff[usb_rcv_count-1] == '\n'){

			// 设置电机ID
			if(strstr((const char *)usb_recbuff, "Motor_ID_Set=") != NULL){
				sscanf((const char *)usb_recbuff, "Motor_ID_Set=%d", &OD_CANID); //设置电机ID
			}

			// 设置波特率
			if(strstr((const char *)usb_recbuff, "Baudrate=") != NULL){
				sscanf((const char *)usb_recbuff, "Baudrate=%d", &usart2_baudrate); //设置波特率
			}
			
			// 设置CAN波特率
			if(strstr((const char *)usb_recbuff, "Can_Baudrate=") != NULL){
				sscanf((const char *)usb_recbuff, "Can_Baudrate=%d", &OD_CAN_BaudRate); //设置CAN波特率
			}

			// 设置编码器类型
			if(strstr((const char *)usb_recbuff, "Encoder_Type=") != NULL){
				sscanf((const char *)usb_recbuff, "Encoder_Type=%d", &encoder_config.mode); 
			}
			
			// 设置编码器CPR
			if(strstr((const char *)usb_recbuff, "Encoder_CPR=") != NULL){
				sscanf((const char *)usb_recbuff, "Encoder_CPR=%d", &encoder_config.cpr); 
			}

			// 设置编码器带宽
			if(strstr((const char *)usb_recbuff, "Encoder_Bandwidth=") != NULL){
				sscanf((const char *)usb_recbuff, "Encoder_Bandwidth=%f", &encoder_config.bandwidth); 
			}

			// 设置编码器方向
			if(strstr((const char *)usb_recbuff, "Encoder_Direction=") != NULL){
				sscanf((const char *)usb_recbuff, "Encoder_Direction=%d", &encoder_config.direction); 
			}

			// 设置电机类型
			if(strstr((const char *)usb_recbuff, "Motor_Type=") != NULL){
				sscanf((const char *)usb_recbuff, "Motor_Type=%d", &motor_config.motor_type); 
			}

			// 设置电机极对数
			if(strstr((const char *)usb_recbuff, "Motor_Pole_Pairs=") != NULL){
				sscanf((const char *)usb_recbuff, "Motor_Pole_Pairs=%d", &motor_config.pole_pairs); 
			}

			// 设置电机校准电流
			if(strstr((const char *)usb_recbuff, "Motor_Calibration_Current=") != NULL){
				sscanf((const char *)usb_recbuff, "Motor_Calibration_Current=%f", &motor_config.calibration_current); 
			}

			// 设置电机校准电压
			if(strstr((const char *)usb_recbuff, "Motor_Resistance_Calib_Max_Voltage=") != NULL){
				sscanf((const char *)usb_recbuff, "Motor_Resistance_Calib_Max_Voltage=%f", &motor_config.resistance_calib_max_voltage); 
			}

			// 设置电机力矩常数
			if(strstr((const char *)usb_recbuff, "Motor_Torque_Constant=") != NULL){
				sscanf((const char *)usb_recbuff, "Motor_Torque_Constant=%f", &motor_config.torque_constant); 
			}

			// 设置控制模式
			if(strstr((const char *)usb_recbuff, "Motor_Control_Mode=") != NULL){
				int mode = 0;
				sscanf((const char *)usb_recbuff, "Motor_Control_Mode=%d", &mode); 
				
				switch(mode){
					case 0:
						// 位置控制模式
						ctrl_config.control_mode = CONTROL_MODE_POSITION_CONTROL;
						// 梯形轨迹模式
						ctrl_config.input_mode = INPUT_MODE_TRAP_TRAJ;
						break;
					case 1:
						//位置滤波器模式
						ctrl_config.control_mode = CONTROL_MODE_POSITION_CONTROL;
						ctrl_config.input_mode = INPUT_MODE_POS_FILTER;
						break;
					case 2:
						//位置直通模式
						ctrl_config.control_mode = CONTROL_MODE_POSITION_CONTROL;
						ctrl_config.input_mode = INPUT_MODE_PASSTHROUGH;
						break;
					case 3:
						//速度梯形模式
						ctrl_config.control_mode = CONTROL_MODE_VELOCITY_CONTROL;
						ctrl_config.input_mode = INPUT_MODE_VEL_RAMP;
						break;
					case 4:
						//速度直通模式
						ctrl_config.control_mode = CONTROL_MODE_VELOCITY_CONTROL;
						ctrl_config.input_mode = INPUT_MODE_PASSTHROUGH;
						break;
					case 5:
						//力矩梯形模式
						ctrl_config.control_mode = CONTROL_MODE_TORQUE_CONTROL;
						ctrl_config.input_mode = INPUT_MODE_TORQUE_RAMP;
						break;
					case 6:
						//力矩直通模式
						ctrl_config.control_mode = CONTROL_MODE_TORQUE_CONTROL;
						ctrl_config.input_mode = INPUT_MODE_PASSTHROUGH;
						break;
					case 7:
						//MIT模式
						ctrl_config.control_mode = CONTROL_MODE_TORQUE_CONTROL;
						ctrl_config.input_mode = INPUT_MODE_MIT;
						break;
				}
			}

			// 设置速度上限
			if(strstr((const char *)usb_recbuff, "Motor_Vel_Limit=") != NULL){
				sscanf((const char *)usb_recbuff, "Motor_Vel_Limit=%f", &ctrl_config.vel_limit); 
			}

			//设置位置环Kp
			if(strstr((const char *)usb_recbuff, "Motor_Pos_P=") != NULL){
				sscanf((const char *)usb_recbuff, "Motor_Pos_P=%f", &ctrl_config.pos_gain); 
			}

			//设置速度环Kp
			if(strstr((const char *)usb_recbuff, "Motor_Vel_P=") != NULL){
				sscanf((const char *)usb_recbuff, "Motor_Vel_P=%f", &ctrl_config.vel_gain); 
			}

			//设置速度环Ki
			if(strstr((const char *)usb_recbuff, "Motor_Vel_I=") != NULL){
				sscanf((const char *)usb_recbuff, "Motor_Vel_I=%f", &ctrl_config.vel_integrator_gain); 
			}

			//设置滤波器带宽
			// if(strstr((const char *)usb_recbuff, "Motor_Filter_Bandwidth=") != NULL){
			// 	sscanf((const char *)usb_recbuff, "Motor_Filter_Bandwidth=%f", &motor_config.vel_filter_bandwidth); 
			// }
			// 设置力矩加速度
			if(strstr((const char *)usb_recbuff, "Motor_Torque_Ramp_Rate=") != NULL){
				sscanf((const char *)usb_recbuff, "Motor_Torque_Ramp_Rate=%f", &ctrl_config.torque_ramp_rate); 
			}

			// 设置速度加速度
			if(strstr((const char *)usb_recbuff, "Motor_Vel_Ramp_Rate=") != NULL){
				sscanf((const char *)usb_recbuff, "Motor_Vel_Ramp_Rate=%f", &ctrl_config.vel_ramp_rate); 
			}

			// 设置梯形轨迹最大速度
			if(strstr((const char *)usb_recbuff, "Traj_Vel_Limit=") != NULL){
				sscanf((const char *)usb_recbuff, "Traj_Vel_Limit=%f", &trapTraj_config.vel_limit); 
			}

			// 设置梯形轨迹加速度
			if(strstr((const char *)usb_recbuff, "Traj_Accel_Limit=") != NULL){
				sscanf((const char *)usb_recbuff, "Traj_Accel_Limit=%f", &trapTraj_config.accel_limit); 
			}

			// 设置梯形轨迹减速度
			if(strstr((const char *)usb_recbuff, "Traj_Decel_Limit=") != NULL){
				sscanf((const char *)usb_recbuff, "Traj_Decel_Limit=%f", &trapTraj_config.decel_limit); 
			}

			// 设置电机运行电流最大值
			if(strstr((const char *)usb_recbuff, "Motor_Current_Limit=") != NULL){
				sscanf((const char *)usb_recbuff, "Motor_Current_Limit=%f", &motor_config.current_lim); 
			}

			// 保存参数
			if(strstr((const char *)usb_recbuff, "SAVE") != NULL){
				flash_para_write();  //下载程序是不会擦除这个扇区，一旦保存不用时请及时擦除
//				len=sprintf((char *)usb_sndbuff, "flash save!\r\n");
//				usb_send(usb_sndbuff, len);
				//IWDG_Init();//使能看门狗，等待复位
			}

			// 擦除扇区
			if(strstr((const char *)usb_recbuff, "ERASE") != NULL){
				disarm();
				flash_para_erase();
				IWDG_Init();//擦除后重启
//				len=sprintf((char *)usb_sndbuff, "flash erase!\r\n");
//				usb_send(usb_sndbuff, len);
			}

			// 复位
			if(strstr((const char *)usb_recbuff, "RESET") != NULL){
				IWDG_Init();//使能看门狗，等待复位
//				len=sprintf((char *)usb_sndbuff, "reset!\r\n");
//				usb_send(usb_sndbuff, len);
			}

			// 电机校准
			if(strstr((const char *)usb_recbuff, "CALIBRATE") != NULL){
				current_state_ = AXIS_STATE_MOTOR_CALIBRATION;
			}

			// 进入闭环
			if(strstr((const char *)usb_recbuff, "CLOSELOOP") != NULL){
				current_state_ = AXIS_STATE_CLOSED_LOOP_CONTROL;
			}

			// 进入空闲
			if(strstr((const char *)usb_recbuff, "IDLE") != NULL){
				input_pos_ = 0;   //清零输入值
				input_vel_ = 0;
				input_torque_ = 0;
				controller_reset();
				disarm();
				current_state_ = AXIS_STATE_IDLE;
			}

			// 配置是否校准
			if(strstr((const char *)usb_recbuff, "CATE_Yes") != NULL){
				encoder_config.pre_calibrated = 1;  //设置为已经校准
			}

//			// 配置是否上电闭环
			if(strstr((const char *)usb_recbuff, "CLOP_Yes") != NULL){
				axis_config.startup_closed_loop_control = 1;  //设置为上电闭环
			}

			// 输入目标力矩
			if(strstr((const char *)usb_recbuff, "TORQUE=") != NULL){
				sscanf((const char *)usb_recbuff, "TORQUE=%f", &input_torque_); //设置力矩
			}

			// 输入目标速度
			if(strstr((const char *)usb_recbuff, "VELOCITY=") != NULL){
				sscanf((const char *)usb_recbuff, "VELOCITY=%f", &input_vel_); //设置速度
			}

			// 输入目标位置
			if(strstr((const char *)usb_recbuff, "POSITION=") != NULL){
				sscanf((const char *)usb_recbuff, "POSITION=%f", &input_pos_); //设置位置
				input_pos_updated_ = true;  //针对梯形轨迹模式，更新目标位置
			}

			// 向上位机发送当前配置
			if(strstr((const char *)usb_recbuff, "Reading_Device") != NULL){
				Send_Configuration();
			}

			memset(usb_recbuff,0,usb_rcv_count); //清空接收数组
			usb_rcv_count = 0;
		}
		
		
		memset(usb_recbuff,0,usb_rcv_count); //清空接收数组
		usb_rcv_count = 0;
	}
}

/*****************************************************************************/