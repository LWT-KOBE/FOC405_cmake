#ifndef IAP_H
#define IAP_H
#include "MyProject.h"

#define IAP_Bootloat_SIZE 		0x2000	//8K字节
#define TrainVersion  7  //版本号

#define PC2MCU_Rend_device_info 0x7D
#define MCU2PC_Rend_device_info 0x7B
#define PC2MCU_IAP				0x7F
#define MCU2PC_IAP 				0x7E


void IAP_APP_CAN_ReStart(CanRxMsg temp_CAN_Msg);
void IAP_APP_Init(void);
void SaveData(CanRxMsg temp_CAN_Msg);
u8 IAP_Send_Device_ino(void);


// /*用户自定义设备CAN_ID和设备类型*/
// #define IAP_CAN_ID			15
// #define IAP_DeviceType		5
// /*用户自定义设备CAN_ID和设备类型
// IAP_DeviceType:
// 设备类型：00:未定义
// 		  01:主板
// 		  02:磁导航-8
// 		  03:磁导航-16
// 		  04:地标
// 		  05:超声波
// 		  06:遥控器
// 	          07:扩展板
// */




// #define IAP_Bootloat_SIZE 		0x2000	//8K

// #define PC2MCU_Rend_device_info 0x7D
// #define MCU2PC_Rend_device_info 0x7B
// #define PC2MCU_IAP				0x7F
// #define MCU2PC_IAP 				0x7E


// extern u32 g_CpuID[3];
// extern u32 g_Lock_Code;

// //该函数请放在main函数最开头
// void IAP_APP_Init(void);	
// //该函数请放在CAN接收中断内
// void IAP_APP_CAN_ReStart(CanRxMsg temp_CAN_Msg);//APP程序CAN中断内,刷固件跳转程序
// u8 IAP_Send_Device_ino(void);				//向上位机发送设备信息

#endif /****************************************************************************/
