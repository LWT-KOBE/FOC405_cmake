#ifndef __CAN_H
#define __CAN_H

#include "MyProject.h"
#define AXIS0_ID 0x000 //M0的CANID
#define AXIS1_ID 0x002 //M1的CANID
#define AXIS2_ID 0x004 //M2的CANID
#define AXIS3_ID 0x006 //M3的CANID

//电机的ID 
enum{	
	axis0=0,	
	axis1=1,		
	axis2=2,
	axis3=3
};

enum{
    CAN_1M=0,
    CAN_500K=1,
    CAN_250K=2,
    CAN_125K=3
};
extern uint32_t OD_CANID; //CAN的ID
extern uint32_t OD_CAN_BaudRate; //CAN的波特率

#define OD_CAN_CMD_MASK      0x1FU
#define OD_CAN_NODE_ID_MASK  0x3FU

static inline uint32_t OD_CAN_MakeStdId(uint32_t node_id, uint32_t cmd_id)
{
	return ((node_id & OD_CAN_NODE_ID_MASK) << 5) | (cmd_id & OD_CAN_CMD_MASK);
}

static inline uint8_t OD_CAN_GetNodeId(uint32_t std_id)
{
	return (uint8_t)((std_id >> 5) & OD_CAN_NODE_ID_MASK);
}

static inline uint32_t OD_CAN_GetCmdId(uint32_t std_id)
{
	return std_id & OD_CAN_CMD_MASK;
}

/*CAN发送结构体*/
typedef struct {
	uint8_t cmd;
	uint8_t data[8];
}CANSendStruct_t;

typedef struct {
	volatile uint32_t enqueued;
	volatile uint32_t mailbox_loaded;
	volatile uint32_t mailbox_completed;
	volatile uint32_t transmit_ok;
	volatile uint32_t arbitration_lost;
	volatile uint32_t transmit_error;
	volatile uint32_t no_mailbox;
	volatile uint32_t queue_full;
	volatile uint32_t invalid_argument;
	volatile uint32_t pending;
	volatile uint32_t peak_pending;
	volatile uint32_t active_mailboxes;
} CAN1TxQueueStats_t;

extern CAN1TxQueueStats_t CAN1_TxQueueStats;

//CAN的电机控制命令
typedef  enum {
        MSG_CO_NMT_CTRL = 0x000,             		 // CANOpen NMT Message REC
		MSG_ODRIVE_HEARTBEAT = 0x001,                // ODrive心跳消息
		MSG_ODRIVE_ESTOP = 0x002,                    // ODrive急停消息
		MSG_GET_MOTOR_ERROR = 0x003,                 // 获取电机错误信息
		MSG_GET_ENCODER_ERROR = 0x004,               // 获取编码器错误信息
		MSG_GET_SENSORLESS_ERROR = 0x005,            // 获取无传感器错误信息
		MSG_SET_AXIS_NODE_ID = 0x006,                // 设置轴节点ID
		MSG_SET_AXIS_REQUESTED_STATE = 0x007,        // 设置轴请求状态
		MSG_SET_AXIS_STARTUP_CONFIG = 0x008,         // 设置轴启动配置
		MSG_GET_ENCODER_ESTIMATES = 0x009,           // 获取编码器估计值
		MSG_GET_ENCODER_COUNT = 0x00A,               // 获取编码器计数
		MSG_SET_CONTROLLER_MODES = 0x00B,            // 设置控制器模式
		MSG_SET_INPUT_POS = 0x00C,                   // 设置位置控制输入
		MSG_SET_INPUT_VEL = 0x00D,                   // 设置速度控制输入
		MSG_SET_INPUT_TORQUE = 0x00E,                // 设置扭矩控制输入
		MSG_SET_LIMITS = 0x00F,                      // 设置限制
		MSG_START_ANTICOGGING = 0x010,               // 启动反扭矩
		MSG_SET_TRAJ_VEL_LIMIT = 0x011,              // 设置轨迹速度限制
		MSG_SET_TRAJ_ACCEL_LIMITS = 0x012,           // 设置轨迹加速度限制
		MSG_SET_TRAJ_INERTIA = 0x013,                // 设置轨迹惯性
		MSG_GET_IQ = 0x014,                          // 获取电流
		MSG_SET_MIT_CONTROL = 0x015,				 // MIT模式控制
		MSG_GET_BUS_VOLTAGE_CURRENT = 0x016,         // 获取母线电压和电流
		MSG_CLEAR_ERRORS = 0x017,                    // 清除错误
		
		
		MSG_SET_LINEAR_COUNT = 0x018,				 //
        MSG_SET_POS_GAIN = 0x019,					 // 设置位置环增益
        MSG_SET_VEL_GAINS = 0x01A,					 //	设置速度环增益
//        MSG_GET_ADC_VOLTAGE = 0x01B,				 //
       
		MSG_RESET_ODRIVE = 0x01B,                    // 重置ODrive
		MSG_SAVE_CONFIG = 0x01C,					 // 保存ODrive
		//MSG_GET_CONTROLLER_ERROR = 0x01C,			 //
		MSG_GET_MOTOR_TEMP = 0x01D,					 // 获取温度
		//新增

		MSG_GET_POS_GAIN = 0x01E,
		MSG_GET_VEL_GAINS = 0x01F,
		//MSG_SAVE_CONFIG = 0x01A,                     // 保存配置

		MSG_CO_HEARTBEAT_CMD = 0x700,        // CANOpen NMT Heartbeat SEND
    }ODCmdStruct_t;

typedef struct {
	
	//位置环增益
	formatTrans32Struct_t Pos_gain[4];
	
	//速度环增益
	formatTrans32Struct_t Vel_gain[4];
	
	//速度环积分增益
	formatTrans32Struct_t Vel_integrator_gain[4];
	
	//电机的控制状态
//	ODAxisStateStruct_t	AxisState[4];
//	
//	//电机的控制模式	
//	ODControlMode ControlMode[4];
	
	//设定电机的位置——float型
	formatTrans32Struct_t SetPos[4];
	
	//设定电机的速度——float型	
	formatTrans32Struct_t SetVel[4];
	
	//设定电机的电流——float型	
	formatTrans32Struct_t SetCur[4];	
		
	//配置电机状态FLAG	
	uint8_t RequestedStateFlag;
	uint8_t RequestedStateFlag1;

	//保存ODRIVE的配置参数FLAG
	uint8_t flashSaveFlag;
	uint8_t flashSaveFlag1;
	
	//重启odrive
	uint8_t RebotFlag;
	uint8_t RebotFlag1;
	
	//配置电机控制模式FLAG	
	uint8_t ControlModeFlag;	
	uint8_t ControlModeFlag1;
	//清楚电机错误FLAG
	uint8_t clearerrorFlag;
	uint8_t clearerrorFlag1;
	
	//查询电机的电感/电阻/极对数和电机控制状态
	uint8_t readpairsFlag;
	uint8_t readpairsFlag1;
	
	//设置电流电压限制FLAG	
	uint8_t SetLimitFlag;
	uint8_t SetLimitFlag1;

	
	//配置电机位置环增添FLAG	
	uint8_t Pos_gainFlag;
	uint8_t Pos_gainFlag1;
	
	//配置电机速度环增益FLAG	
	uint8_t Vel_gainFlag;
	uint8_t Vel_gainFlag1;
	
	//
	u8 traj_Flag; 
	
	//速度限制——发送
	formatTrans32Struct_t vel_limit[4]; 
	
	//电流限制——发送
	formatTrans32Struct_t current_limit[4];
	
	//位置梯形模式加速度值
	formatTrans32Struct_t traj_accel_limit[4];
	//位置梯形模式减速度值
	formatTrans32Struct_t traj_decel_limit[4];
	//位置梯形模式速度限制值
	formatTrans32Struct_t traj_vel_limit[4];

	u8 OD_flag;
	uint8_t dataInitFlag;
	uint32_t loops;
	
} OdriveStruct_t;

extern CANSendStruct_t ODSendData;
void CAN1_Init(void);
void CAN1_Mode_Init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode);
void CAN1_Set_BaudRate(uint8_t baudRate);
void CAN1_SendData(CAN_TypeDef *CANx, uint32_t ID_CAN,uint8_t len,CANSendStruct_t* CanSendData);
void OdriveSendData(CAN_TypeDef *CANx, uint32_t ID_CAN, uint32_t CMD_CAN, uint8_t len, CANSendStruct_t* CanSendData);
void can_SendData(CAN_TypeDef *CANx, uint32_t ID_CAN, uint8_t len,CANSendStruct_t* CanSendData);
u8 CAN1_Send_Msg(u8* msg,u8 len);
void can_SendFloatData(CAN_TypeDef *CANx, uint32_t ID_CAN, uint8_t len,float data,CANSendStruct_t* CanSendData);
void CANSendData_2(CAN_TypeDef *CANx, uint32_t ID_CAN,uint8_t len,float data1,float data2,CANSendStruct_t* CanSendData);

void OD_CANSendData(CAN_TypeDef *CANx, uint32_t ID_CAN, uint32_t CMD_CAN,uint8_t len,float data,CANSendStruct_t* CanSendData);
void OD_CANSendData_2(CAN_TypeDef *CANx, uint32_t ID_CAN, uint32_t CMD_CAN,uint8_t len,float data1,float data2,CANSendStruct_t* CanSendData);
#endif
