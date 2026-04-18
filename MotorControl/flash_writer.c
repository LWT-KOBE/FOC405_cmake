#include "MyProject.h"
#include "stm32f4xx_flash.h"
#include "flash_writer.h"
/*
STM32F405RGT6, 1M flash, 中文数据手册P.59
扇区 0   0x0800 0000 - 0x0800 3FFF 16 KB
扇区 1   0x0800 4000 - 0x0800 7FFF 16 KB
扇区 2   0x0800 8000 - 0x0800 BFFF 16 KB
扇区 3   0x0800 C000 - 0x0800 FFFF 16 KB
扇区 4   0x0801 0000 - 0x0801 FFFF 64 KB
扇区 5   0x0802 0000 - 0x0803 FFFF 128 KB
扇区 6   0x0804 0000 - 0x0805 FFFF 128 KB
扇区 7   0x0806 0000 - 0x0807 FFFF 128 KB
扇区 8   0x0808 0000 - 0x0809 FFFF 128 KB
扇区 9   0x080A 0000 - 0x080B FFFF 128 KB
扇区 10  0x080C 0000 - 0x080D FFFF 128 KB
扇区 11  0x080E 0000 - 0x080F FFFF 128 KB

#define FLASH_Sector_0     ((uint16_t)0x0000)
#define FLASH_Sector_1     ((uint16_t)0x0008)
#define FLASH_Sector_2     ((uint16_t)0x0010)
#define FLASH_Sector_3     ((uint16_t)0x0018)
#define FLASH_Sector_4     ((uint16_t)0x0020)
#define FLASH_Sector_5     ((uint16_t)0x0028)
#define FLASH_Sector_6     ((uint16_t)0x0030)
#define FLASH_Sector_7     ((uint16_t)0x0038)
#define FLASH_Sector_8     ((uint16_t)0x0040)
#define FLASH_Sector_9     ((uint16_t)0x0048)
#define FLASH_Sector_10    ((uint16_t)0x0050)
#define FLASH_Sector_11    ((uint16_t)0x0058)
*/
//第一次烧写固件前，请先整片擦除单片机，因为之前代码可能也使用了这个扇区保存参数
//比如官方odrive代码，好像就是使用的这个扇区，如果当时保存了参数，
//再运行当前代码，会把之前保存的参数用于当前代码中导致错误。


#define FLASH_Sector_11    ((uint16_t)0x0058)
#define Flash_Addr         0x080E0000     //保存参数的flash起始地址, 地址必须4字节对齐
#define NUMBER_PARA_        34
#define Flash_AntiCogging_Addr    0x080F0000     //保存抗齿槽校准参数的flash起始地址
/*****************************************************************************/
uint32_t flash_reg[128];
/*****************************************************************************/
typedef union
{
	float f;
	uint32_t da;
} UN32;

uint32_t float2uint(float var)
{
	UN32 a;
	
	a.f = var;
	return a.da;
}
float uint2float(uint32_t var)
{
	UN32 a;
	
	a.da = var;
	return a.f;
}
/*****************************************************************************/
void flash_para_read(void)
{
	uint32_t i;
	
	for(i=0; i<NUMBER_PARA_ ;i++)
	{
		flash_reg[i] = *(uint32_t*)(Flash_Addr+ i*4);
	}
	#if 1
	if(flash_reg[28]==1)   //已经校准
	{
		
		// 基础设置
		OD_CANID = flash_reg[0];  //CAN的ID
		OD_CAN_BaudRate = flash_reg[1];  //CAN的波特率
		usart2_baudrate = flash_reg[2];  //usart2波特率
		// 控制器设置
		ctrl_config.control_mode = flash_reg[3];         //控制模式
		ctrl_config.input_mode = flash_reg[4];             //输入模式
		ctrl_config.torque_ramp_rate = uint2float(flash_reg[5]); //Nm / sec，力矩爬升率
		ctrl_config.vel_ramp_rate = uint2float(flash_reg[6]);  //速度的爬升率
		ctrl_config.pos_gain = uint2float(flash_reg[7]);		//位置P参数
		ctrl_config.vel_gain = uint2float(flash_reg[8]);               //速度P参数
		ctrl_config.vel_integrator_gain = uint2float(flash_reg[9]);    //速度I参数
		ctrl_config.vel_limit = uint2float(flash_reg[10]);          //最大转速限制，圈/秒
		ctrl_config.input_filter_bandwidth = uint2float(flash_reg[11]);     // [1/s]

		trapTraj_config.vel_limit = uint2float(flash_reg[12]);      //梯形轨迹最大速度; // [turn/s]
		trapTraj_config.accel_limit = uint2float(flash_reg[13]);  //梯形轨迹加速度; // [turn/s^2]
		trapTraj_config.decel_limit = uint2float(flash_reg[14]);  //梯形轨迹减速度; // [turn/s^2]

		// 电机设置
		motor_config.pole_pairs = flash_reg[15];  //电机极对数
		motor_config.calibration_current = uint2float(flash_reg[16]);    // 校准电流[A]
		motor_config.resistance_calib_max_voltage = uint2float(flash_reg[17]); // 校准电压[V]
		motor_config.phase_resistance = uint2float(flash_reg[18]);        // 相电阻
		motor_config.phase_inductance = uint2float(flash_reg[19]);    //相电感   如果是云台电机设置为0
		motor_config.torque_constant = uint2float(flash_reg[20]);        // 力矩常数[Nm/A]
		motor_config.motor_type = flash_reg[21];  		 //电机类型
		// Read out max_allowed_current to see max supported value for current_lim.
		motor_config.current_lim = uint2float(flash_reg[22]); //[A] 电机最大运行电流
		motor_config.current_lim_margin = uint2float(flash_reg[23]);      // Maximum violation of current_lim
		motor_config.current_control_bandwidth = uint2float(flash_reg[24]);  //电流环控制带宽 [rad/s]
		

		// 编码器设置
		encoder_config.mode = flash_reg[25];  //选择编码器型号，参数设置宏定义在MyProject.h文件中
		encoder_config.cpr = flash_reg[26];    //编码器cpr
		encoder_config.bandwidth = uint2float(flash_reg[27]);   //编码器带宽
		encoder_config.pre_calibrated = flash_reg[28];                //是否已经校准
		encoder_config.direction = flash_reg[29]; // 电机方向
		
		encoder_config.phase_offset = flash_reg[30];                  //偏移值 整数部分
		encoder_config.phase_offset_float = uint2float(flash_reg[31]);//偏移值 小数部分
		// encoder_config.pre_calibrated = flash_reg[32];                //是否已经校准
		axis_config.startup_closed_loop_control = flash_reg[32];      //是否上电后进入闭环
		anticogging_valid_ = flash_reg[33];                           //抗齿槽校准数据是否正常标志位

		update_current_controller_gains();
		is_calibrated_ = 1;
	}
		// 控制模式设置
	
	#else
	if(flash_reg[5]==1)   //已经校准
	{
		motor_config.phase_resistance = uint2float(flash_reg[0]);    //相电阻
		motor_config.phase_inductance = uint2float(flash_reg[1]);    //相电感   如果是云台电机设置为0
		encoder_config.direction = flash_reg[2];                     //方向
		encoder_config.phase_offset = flash_reg[3];                  //偏移值 整数部分
		encoder_config.phase_offset_float = uint2float(flash_reg[4]);//偏移值 小数部分
		encoder_config.pre_calibrated = flash_reg[5];                //是否已经校准
		axis_config.startup_closed_loop_control = flash_reg[6];      //是否上电后进入闭环
		anticogging_valid_ = flash_reg[7];                           //抗齿槽校准数据是否正常标志位

		motor_config.pole_pairs = flash_reg[8];  //极对数
		ctrl_config.control_mode = flash_reg[9];         //控制模式
		ctrl_config.input_mode = flash_reg[10];           //输入模式
		ctrl_config.pos_gain = uint2float(flash_reg[11]);                   //位置增益
		ctrl_config.vel_gain = uint2float(flash_reg[12]);                   //速度增益
		ctrl_config.vel_integrator_gain = uint2float(flash_reg[13]);        //速度积分增益
		ctrl_config.vel_limit = uint2float(flash_reg[14]);                  //速度限制
		ctrl_config.vel_limit_tolerance = uint2float(flash_reg[15]);        //速度限制容差
		ctrl_config.vel_integrator_limit = uint2float(flash_reg[16]);       //速度积分限制
		ctrl_config.vel_ramp_rate = uint2float(flash_reg[17]);              //速度斜坡率
		ctrl_config.torque_ramp_rate = uint2float(flash_reg[18]); 			//力矩斜坡率
		OD_CANID = flash_reg[19];
		update_current_controller_gains();
		is_calibrated_ = 1;
	}
		#endif
		// motor_config.calibration_current = uint2float(flash_reg[20]); //校准电流
		// motor_config.current_control_bandwidth = uint2float(flash_reg[21]); //电流环带宽

		// update_current_controller_gains();
		// is_calibrated_ = 1;
		

	
	if(anticogging_valid_)   //如果抗齿槽校准数据正常，读出全部数据
	{
		for(i=0; i<COG_num ;i++)
		{
			anticogging.cogging_map[i] = *(float*)(Flash_AntiCogging_Addr+i*4);
		}
	}
}
/*****************************************************************************/
void flash_para_write(void)
{
	uint32_t i;
	FLASH_Unlock();
	FLASH_DataCacheCmd(DISABLE);  //禁止数据缓存
	
	FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);  //擦除整个扇区128K字节
	#if 1
	flash_reg[0] = OD_CANID;  //CAN的ID
	flash_reg[1] = OD_CAN_BaudRate;  //CAN的波特率
	flash_reg[2] = usart2_baudrate;  //usart2波特率
	flash_reg[3] = ctrl_config.control_mode;         //控制模式
	flash_reg[4] = ctrl_config.input_mode;             //输入模式
	flash_reg[5] = float2uint(ctrl_config.torque_ramp_rate); //Nm / sec，力矩爬升率
	flash_reg[6] = float2uint(ctrl_config.vel_ramp_rate);  //速度的爬升率
	flash_reg[7] = float2uint(ctrl_config.pos_gain);		//位置P参数
	flash_reg[8] = float2uint(ctrl_config.vel_gain);               //速度P参数
	flash_reg[9] = float2uint(ctrl_config.vel_integrator_gain);    //速度I参数
	flash_reg[10] = float2uint(ctrl_config.vel_limit);          //最大转速限制，圈/秒
	flash_reg[11] = float2uint(ctrl_config.input_filter_bandwidth);     // [1/s]
	flash_reg[12] = float2uint(trapTraj_config.vel_limit);      //梯形轨迹最大速度; // [turn/s]
	flash_reg[13] = float2uint(trapTraj_config.accel_limit);  //梯形轨迹加速度; // [turn/s^2]
	flash_reg[14] = float2uint(trapTraj_config.decel_limit);  //梯形轨迹减速度; // [turn/s^2]
	flash_reg[15] = motor_config.pole_pairs;  //电机极对数
	flash_reg[16] = float2uint(motor_config.calibration_current);    // 校准电流[A]
	flash_reg[17] = float2uint(motor_config.resistance_calib_max_voltage); // 校准电压[V]
	flash_reg[18] = float2uint(motor_config.phase_resistance);        // 相电阻
	flash_reg[19] = float2uint(motor_config.phase_inductance);	//相电感   如果是云台电机设置为0
	flash_reg[20] = float2uint(motor_config.torque_constant);        // 力矩常数[Nm/A]
	flash_reg[21] = motor_config.motor_type;  		 //电机类型
	flash_reg[22] = float2uint(motor_config.current_lim); //[A] 电机最大运行电流
	flash_reg[23] = float2uint(motor_config.current_lim_margin);      // Maximum violation of current_lim
	flash_reg[24] = float2uint(motor_config.current_control_bandwidth);  //电流环控制带宽 [rad/s]
	flash_reg[25] = encoder_config.mode;  //选择编码器型号，参数
	flash_reg[26] = encoder_config.cpr;    //编码器cpr
	flash_reg[27] = float2uint(encoder_config.bandwidth);   //编码器带宽
	flash_reg[28] = encoder_config.pre_calibrated;                //是否已经校准
	flash_reg[29] = encoder_config.direction; // 电机方向

	flash_reg[30] = encoder_config.phase_offset;                  //偏移值 整数部分
	flash_reg[31] = float2uint(encoder_config.phase_offset_float);//偏移值 小数部分
	flash_reg[32] = axis_config.startup_closed_loop_control;      //是否上电后进入闭环
	flash_reg[33] = anticogging_valid_;                           //抗齿槽校准数据是否正常标志位
	

	#else
	flash_reg[0] = float2uint(motor_config.phase_resistance);    //相电阻
	flash_reg[1] = float2uint(motor_config.phase_inductance);    //相电感
	flash_reg[2] = encoder_config.direction;                     //方向
	flash_reg[3] = encoder_config.phase_offset;                  //偏移值 整数部分
	flash_reg[4] = float2uint(encoder_config.phase_offset_float);//偏移值 小数部分
	flash_reg[5] = encoder_config.pre_calibrated;                //是否已经校准
	flash_reg[6] = axis_config.startup_closed_loop_control;      //是否上电后进入闭环
	flash_reg[7] = anticogging_valid_;                           //抗齿槽校准数据是否正常
	
	flash_reg[8] = motor_config.pole_pairs;  							//极对数
	flash_reg[9] = ctrl_config.control_mode;         					//控制模式
	flash_reg[10] = ctrl_config.input_mode;           					//输入模式
	flash_reg[11] = float2uint(ctrl_config.pos_gain);                   //位置增益
	flash_reg[12] = float2uint(ctrl_config.vel_gain);                   //速度增益
	flash_reg[13] = float2uint(ctrl_config.vel_integrator_gain);        //速度积分增益
	flash_reg[14] = float2uint(ctrl_config.vel_limit);                  //速度限制
	flash_reg[15] = float2uint(ctrl_config.vel_limit_tolerance);        //速度限制容差
	flash_reg[16] = float2uint(ctrl_config.vel_integrator_limit);       //速度积分限制
	flash_reg[17] = float2uint(ctrl_config.vel_ramp_rate);              //速度斜坡率
	flash_reg[18] = float2uint(ctrl_config.torque_ramp_rate);		    //力矩斜坡率
	flash_reg[19] = OD_CANID;			
	#endif
	for(i=0; i<NUMBER_PARA_; i++)  //保存20个参数
	{
		FLASH_ProgramWord(Flash_Addr+4*i, flash_reg[i]);
	}
	
	if(anticogging_valid_)  //如果抗齿槽校准数据正常，保存
	{
		for(i=0;i<COG_num;i++)
		{
			FLASH_ProgramWord(Flash_AntiCogging_Addr+4*i, float2uint(anticogging.cogging_map[i]));
		}
	}
	
	FLASH_DataCacheCmd(ENABLE);   //开启数据缓存
	FLASH_Lock();
}
/*****************************************************************************/
void flash_para_erase(void)
{
	FLASH_Unlock();
	FLASH_DataCacheCmd(DISABLE);  //禁止数据缓存
	FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);  //擦除整个扇区128K字节
	FLASH_DataCacheCmd(ENABLE);   //开启数据缓存
	FLASH_Lock();
}
/*****************************************************************************/




