#include "can.h"
CANSendStruct_t ODSendData;

float En_d40_angle = 0;
int32_t En_d40_raw = 0;

uint8_t OD_CANID; //CAN的ID
uint8_t OD_CAN_BaudRate; //波特率
// CAN1初始化函数
void CAN1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;           // 定义GPIO初始化结构体
    CAN_InitTypeDef CAN_InitStructure;             // 定义CAN初始化结构体
    CAN_FilterInitTypeDef CAN_FilterInitStructure; // 定义CAN滤波器初始化结构体
	NVIC_InitTypeDef		NVIC_InitStructure;
    // 1. 使能GPIOB和CAN1的时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);    // 使能GPIOB时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);     // 使能CAN1时钟

    // 2. 配置PB8为CAN1_RX，PB9为CAN1_TX
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_CAN1);  // PB8复用为CAN1
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_CAN1);  // PB9复用为CAN1

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;   // 选择PB8和PB9
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;             // 复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;       // 高速
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;           // 推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;             // 上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);                   // 初始化GPIOB

    // 3. CAN配置
    CAN_DeInit(CAN1);                                        // 复位CAN1
    CAN_StructInit(&CAN_InitStructure);                      // 初始化CAN结构体为默认值
    CAN_InitStructure.CAN_TTCM = DISABLE;                    // 禁用时间触发通信模式
    CAN_InitStructure.CAN_ABOM = DISABLE;                    // 禁用自动离线管理
    CAN_InitStructure.CAN_AWUM = DISABLE;                    // 禁用自动唤醒
    CAN_InitStructure.CAN_NART = ENABLE;                     // 禁用自动重传
    CAN_InitStructure.CAN_RFLM = DISABLE;                    // 禁用接收FIFO锁定模式
    CAN_InitStructure.CAN_TXFP = DISABLE;                    // 禁用发送FIFO优先级
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;            // 设置为正常模式
    // 1Mbps: CAN_CLK = 42MHz, Prescaler = 3, BS1 = 11, BS2 = 4, SJW = 1
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;                 // 同步跳转宽度1
    CAN_InitStructure.CAN_BS1 = CAN_BS1_11tq;                // 时间段1为11
    CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;                 // 时间段2为4
    CAN_InitStructure.CAN_Prescaler = 3;                     // 预分频为3
    CAN_Init(CAN1, &CAN_InitStructure);                      // 初始化CAN1

    // 4. CAN滤波器配置（接收所有报文）
    CAN_FilterInitStructure.CAN_FilterNumber = 0;            // 滤波器编号0
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; // 标识符屏蔽模式
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; // 32位宽度
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;       // 标识符高16位
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;        // 标识符低16位
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;   // 屏蔽高16位
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;    // 屏蔽低16位
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0; // 分配到FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;   // 使能滤波器
    CAN_FilterInit(&CAN_FilterInitStructure);                // 初始化CAN滤波器


	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂起中断允许


	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;// 主优先级为4
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;// 次优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

}

//CAN1初始化
void CAN1_Mode_Init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode)
{
	GPIO_InitTypeDef		GPIO_InitStructure;
	CAN_InitTypeDef			CAN_InitStructure;
	CAN_FilterInitTypeDef	CAN_FilterInitStructure;

	NVIC_InitTypeDef		NVIC_InitStructure;

	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
	//使能相关时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟


	//引脚复用映射配置
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_CAN1); //GPIOB8复用为CAN1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_CAN1); //GPIOB9复用为CAN1

	//初始化GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PB8，PB9

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PB8，PB9


	//CAN外设初始化
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);

	//CAN单元设置
	CAN_InitStructure.CAN_TTCM=DISABLE;		//非时间触发通信模式
	CAN_InitStructure.CAN_ABOM=ENABLE;		//软件自动离线管理
	CAN_InitStructure.CAN_AWUM=ENABLE;		//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART=DISABLE;		//禁止报文自动传送
	CAN_InitStructure.CAN_RFLM=DISABLE;		//报文不锁定,新的覆盖旧的
	CAN_InitStructure.CAN_TXFP=DISABLE;		//优先级由报文标识符决定

	CAN_InitStructure.CAN_Mode= mode;//模式设置
	CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1

	CAN_Init(CAN1, &CAN_InitStructure);// 初始化CAN1

	//配置过滤器
	CAN_SlaveStartBank(0);
	CAN_FilterInitStructure.CAN_FilterNumber=0;//过滤器0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;//32位
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化



	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂起中断允许


	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;// 主优先级为4
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;// 次优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

}

void CAN1_Set_BaudRate(uint8_t baudRate){
    switch (baudRate)
	{
	case CAN_1M:
		/* code */
		CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,3,CAN_Mode_Normal);
		break;
	case CAN_500K:
		CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal);
		break;
	case CAN_250K:
		CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,12,CAN_Mode_Normal);
		break;
	case CAN_125K:
		CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,24,CAN_Mode_Normal);
		break;

	default:
		break;
	}
}

// CAN1数据发送函数
// 参数：CANx - CAN控制器实例（如CAN1/CAN2）
//       ID_CAN - 报文标识符（标准ID）
//       len - 数据长度（0-8字节）
//       CanSendData - 包含发送数据的结构体指针
void CAN1_SendData(CAN_TypeDef *CANx, uint32_t ID_CAN,uint8_t len, CANSendStruct_t* CanSendData)
{
    CanTxMsg *txMessage;	// 定义CAN发送报文结构体指针
    uint8_t mbox;           // 用于存储发送邮箱号（0-2）
    uint8_t count;          // 数据拷贝循环计数器
    uint16_t i = 0;         // 发送状态检查超时计数器

    // 动态分配CAN报文内存（8字节对齐）
    txMessage = (CanTxMsg*)aqCalloc(8,sizeof(CanTxMsg));

    // 设置CAN报文头信息
    txMessage->StdId = ID_CAN;      // 设置标准标识符
    txMessage->IDE = CAN_Id_Standard; // 使用标准帧格式（非扩展帧）
    txMessage->RTR = CAN_RTR_Data;  // 设置为数据帧（非远程帧）
    txMessage->DLC = len;           // 设置数据长度（0-8）

    // 拷贝用户数据到CAN报文
    for (count = 0; count < len; count++) {
        txMessage->Data[count] = (uint8_t)CanSendData->data[count]; // 逐字节拷贝数据
    }

    // 启动CAN发送并获取使用的邮箱号
    mbox = CAN_Transmit(CANx, txMessage);

    // 等待发送完成（带超时保护）
    while (CAN_TransmitStatus(CANx,mbox) == 0x00) { // 0x00表示发送未完成
        i++;
        if (i >= 0xFFF) break; // 超过4095次等待则超时退出
    }

    // 释放动态分配的内存
    aqFree(txMessage,8,sizeof(CanTxMsg));
}


// CAN1数据发送函数
// 参数：CANx - CAN控制器实例（如CAN1/CAN2）
//       ID_CAN - 报文标识符（标准ID）
//       len - 数据长度（0-8字节）
//       CanSendData - 包含发送数据的结构体指针
void OdriveSendData(CAN_TypeDef *CANx, uint32_t ID_CAN, uint32_t CMD_CAN, uint8_t len, CANSendStruct_t* CanSendData)
{
    CanTxMsg *txMessage;	// 定义CAN发送报文结构体指针
    uint8_t mbox;           // 用于存储发送邮箱号（0-2）
    uint8_t count;          // 数据拷贝循环计数器
    uint16_t i = 0;         // 发送状态检查超时计数器

    // 动态分配CAN报文内存（8字节对齐）
    txMessage = (CanTxMsg*)aqCalloc(8,sizeof(CanTxMsg));

    // 设置CAN报文头信息
    //CAN ID 的前六位是轴ID（在odrive端设置为0x001），后五位是控制命令（比如 MSG_GET_ENCODER_ERROR）
	txMessage->StdId = (ID_CAN<<5)+CMD_CAN;
    txMessage->IDE = CAN_Id_Standard; // 使用标准帧格式（非扩展帧）
    txMessage->RTR = CAN_RTR_Data;  // 设置为数据帧（非远程帧）
    txMessage->DLC = len;           // 设置数据长度（0-8）

    // 拷贝用户数据到CAN报文
    for (count = 0; count < len; count++) {
        txMessage->Data[count] = (uint8_t)CanSendData->data[count]; // 逐字节拷贝数据
    }

    // 启动CAN发送并获取使用的邮箱号
    mbox = CAN_Transmit(CANx, txMessage);

    // 等待发送完成（带超时保护）
    // while (CAN_TransmitStatus(CANx,mbox) == 0x00) { // 0x00表示发送未完成
    //     i++;
    //     if (i >= 0xFFF) break; // 超过4095次等待则超时退出
    // }

    // 释放动态分配的内存
    aqFree(txMessage,8,sizeof(CanTxMsg));
}





void can_SendFloatData(CAN_TypeDef *CANx, uint32_t ID_CAN, uint8_t len,float data,CANSendStruct_t* CanSendData) {
	// 用于将 float 转换为字节数组的联合体
	FloatLongType fl;
	// 将 float 数据赋值给联合体的 float 成员
	fl.fdata = data;
	// 将联合体的 long 成员转换为字节数组并存储到 CanSendData->data 中
    CanSendData->data[0] = (unsigned char)fl.ldata;
	CanSendData->data[1] = (unsigned char)(fl.ldata>>8);
	CanSendData->data[2] = (unsigned char)(fl.ldata>>16);
	CanSendData->data[3] = (unsigned char)(fl.ldata>>24);

	// 发送数据
	CAN1_SendData(CANx,ID_CAN,len,CanSendData);
	// OdriveSendData(CANx,ID_CAN,CMD_CAN,len,CanSendData);
}

void OD_CANSendData(CAN_TypeDef *CANx, uint32_t ID_CAN, uint32_t CMD_CAN,uint8_t len,float data,CANSendStruct_t* CanSendData) {
	// 用于将 float 转换为字节数组的联合体
	FloatLongType fl;
	// 将 float 数据赋值给联合体的 float 成员
	fl.fdata = data;
	// 将联合体的 long 成员转换为字节数组并存储到 CanSendData->data 中
    CanSendData->data[0] = (unsigned char)fl.ldata;
	CanSendData->data[1] = (unsigned char)(fl.ldata>>8);
	CanSendData->data[2] = (unsigned char)(fl.ldata>>16);
	CanSendData->data[3] = (unsigned char)(fl.ldata>>24);



	// 发送数据
	OdriveSendData(CANx,ID_CAN,CMD_CAN,len,CanSendData);
}

void OD_CANSendData_2(CAN_TypeDef *CANx, uint32_t ID_CAN, uint32_t CMD_CAN,uint8_t len,float data1,float data2,CANSendStruct_t* CanSendData) {
	// 用于将 float 转换为字节数组的联合体
	FloatLongType fl,f2;
	// 将 float 数据赋值给联合体的 float 成员
	fl.fdata = data1;
	f2.fdata = data2;
	// 将联合体的 long 成员转换为字节数组并存储到 CanSendData->data 中
    CanSendData->data[0] = (unsigned char)fl.ldata;
	CanSendData->data[1] = (unsigned char)(fl.ldata>>8);
	CanSendData->data[2] = (unsigned char)(fl.ldata>>16);
	CanSendData->data[3] = (unsigned char)(fl.ldata>>24);

	CanSendData->data[4] = (unsigned char)f2.ldata;
	CanSendData->data[5] = (unsigned char)(f2.ldata>>8);
	CanSendData->data[6] = (unsigned char)(f2.ldata>>16);
	CanSendData->data[7] = (unsigned char)(f2.ldata>>24);


	// 发送数据
	OdriveSendData(CANx,ID_CAN,CMD_CAN,len,CanSendData);
}

void CANSendData_2(CAN_TypeDef *CANx, uint32_t ID_CAN,uint8_t len,float data1,float data2,CANSendStruct_t* CanSendData) {
	// 用于将 float 转换为字节数组的联合体
	FloatLongType fl,f2;
	// 将 float 数据赋值给联合体的 float 成员
	fl.fdata = data1;
	f2.fdata = data2;
	// 将联合体的 long 成员转换为字节数组并存储到 CanSendData->data 中
	CanSendData->data[0] = (unsigned char)fl.ldata;
	CanSendData->data[1] = (unsigned char)(fl.ldata>>8);
	CanSendData->data[2] = (unsigned char)(fl.ldata>>16);
	CanSendData->data[3] = (unsigned char)(fl.ldata>>24);

	CanSendData->data[4] = (unsigned char)f2.ldata;
	CanSendData->data[5] = (unsigned char)(f2.ldata>>8);
	CanSendData->data[6] = (unsigned char)(f2.ldata>>16);
	CanSendData->data[7] = (unsigned char)(f2.ldata>>24);


	// 发送数据
	CAN1_SendData(CANx,ID_CAN,len,CanSendData);
}

void can_SendIntData(CAN_TypeDef *CANx, uint32_t ID_CAN, uint8_t len,int data,CANSendStruct_t* CanSendData) {
    // 用于将 int 转换为字节数组的联合体

}




void ODSetPos_gainData(CanRxMsg* CanRevData) {
	// 将CAN数据转换为float（假设数据是小端序）
    float new_gain;
    memcpy(&new_gain, CanRevData->Data, sizeof(float));

    // 可选：检查增益值是否在合理范围内
    if (new_gain < 0.0f || new_gain > 1000.0f) {
        //return false; // 非法值
    }

    // 更新位置环增益
	ctrl_config.pos_gain = new_gain;
}

void ODSetVel_gainsData(CanRxMsg* CanRevData) {

	formatTrans32Struct_t vel_gain; // 用于将 float 转换为字节数组的联合体
	formatTrans32Struct_t vel_integrator_gain; // 用于将 float 转换为字节数组的联合体

	vel_gain.u8_temp[0] = CanRevData->Data[0];
	vel_gain.u8_temp[1] = CanRevData->Data[1];
	vel_gain.u8_temp[2] = CanRevData->Data[2];
	vel_gain.u8_temp[3] = CanRevData->Data[3];

	vel_integrator_gain.u8_temp[0] = CanRevData->Data[4];
	vel_integrator_gain.u8_temp[1] = CanRevData->Data[5];
	vel_integrator_gain.u8_temp[2] = CanRevData->Data[6];
	vel_integrator_gain.u8_temp[3] = CanRevData->Data[7];

    // 更新位置环增益
	ctrl_config.vel_gain = vel_gain.float_temp;
	ctrl_config.vel_integrator_gain = vel_integrator_gain.float_temp;
}

void OD_MSG_SET_CONTROLLER_MODES(CanRxMsg* CanRevData)
{
	switch (CanRevData->Data[0])
	{
		case 1:
			// 位置控制模式
			ctrl_config.control_mode = CONTROL_MODE_POSITION_CONTROL;
			// 梯形轨迹模式
			ctrl_config.input_mode = INPUT_MODE_TRAP_TRAJ;
			break;
		case 2:
			//位置滤波器模式
			ctrl_config.control_mode = CONTROL_MODE_POSITION_CONTROL;
			ctrl_config.input_mode = INPUT_MODE_POS_FILTER;
			break;
		case 3:
			//位置直通模式
			ctrl_config.control_mode = CONTROL_MODE_POSITION_CONTROL;
			ctrl_config.input_mode = INPUT_MODE_PASSTHROUGH;
			break;
		case 4:
			//速度梯形模式
			ctrl_config.control_mode = CONTROL_MODE_VELOCITY_CONTROL;
			ctrl_config.input_mode = INPUT_MODE_VEL_RAMP;
			break;
		case 5:
			//速度直通模式
			ctrl_config.control_mode = CONTROL_MODE_VELOCITY_CONTROL;
			ctrl_config.input_mode = INPUT_MODE_PASSTHROUGH;
			break;
		case 6:
			//力矩梯形模式
			ctrl_config.control_mode = CONTROL_MODE_TORQUE_CONTROL;
			ctrl_config.input_mode = INPUT_MODE_TORQUE_RAMP;
			break;
		case 7:
			//力矩直通模式
			ctrl_config.control_mode = CONTROL_MODE_TORQUE_CONTROL;
			ctrl_config.input_mode = INPUT_MODE_PASSTHROUGH;
			break;
		case 8:
			//MIT模式
			ctrl_config.control_mode = CONTROL_MODE_TORQUE_CONTROL;
			ctrl_config.input_mode = INPUT_MODE_MIT;
			break;
		default:
			ctrl_config.input_mode = INPUT_MODE_INACTIVE;
		    break;
	}
}

void OD_SET_INPUT_POS(CanRxMsg* CanRevData) {
	// 将CAN数据转换为float（假设数据是小端序）
    float new_pos;
    memcpy(&new_pos, CanRevData->Data, sizeof(float));

	// 更新输入位置
	input_pos_ = new_pos;
	input_pos_updated_ = true;  //针对梯形轨迹模式，更新目标位置
}

// 机制科技中空编码器
void En_d40_read(CanRxMsg* CanRevData)
{

	uint32_t raw = ((uint32_t)CanRevData->Data[6] << 24) |  // Byte7
				   ((uint32_t)CanRevData->Data[7] << 16) |  // Byte8
				   ((uint32_t)CanRevData->Data[1] << 8)  |  // Byte2
				   ((uint32_t)CanRevData->Data[2]) ;         // Byte3

	// uint32_t raw = ((uint32_t)CanRevData->Data[6] << 24) |  // Byte7
	// 			   ((uint32_t)CanRevData->Data[7] << 16) |  // Byte8
	// 			   ((uint32_t)CanRevData->Data[1] << 8)  ;  // Byte2
	// 			   // ((uint32_t)CanRevData->Data[2]);         // Byte3

	// 编码器值 只读取单圈绝对值
	En_d40_raw = (int32_t)raw & 0xfffff;

	// 读取多圈值 最多支持读取18圈
	// En_d40_raw = (int32_t)raw;

	// 将编码器值转换为角度
	En_d40_angle = (En_d40_raw) * 360.0f / 1048576;
}

void OD_SET_INPUT_VEL(CanRxMsg* CanRevData) {
	// 将CAN数据转换为float（假设数据是小端序）
    float new_vel;
    memcpy(&new_vel, CanRevData->Data, sizeof(float));

	// 更新输入速度
	input_vel_ = new_vel;
}


void OD_SET_INPUT_CUR(CanRxMsg* CanRevData) {
	// 将CAN数据转换为float（假设数据是小端序）
    float new_cur;
    memcpy(&new_cur, CanRevData->Data, sizeof(float));

	// 更新输入电流
	input_torque_ = new_cur;
}

void OD_SET_INPUT_LIMITS(CanRxMsg* CanRevData) {
	// 将CAN数据转换为float（假设数据是小端序）
    float cur_limit,vel_limit;
    memcpy(&cur_limit, CanRevData->Data, sizeof(float));
	memcpy(&vel_limit, CanRevData->Data + sizeof(float), sizeof(float));
	// 更新电流最大值
	motor_config.current_lim = cur_limit;
	// 更新速度最大值
	ctrl_config.vel_limit = vel_limit;
}

void OD_SET_TRAPTRAJ_VEL_LIMIT(CanRxMsg* CanRevData)
{
	float traptraj_vel_limits;
	memcpy(&traptraj_vel_limits, CanRevData->Data, sizeof(float));

	trapTraj_config.vel_limit = traptraj_vel_limits;
}

void OD_SET_TRAPTRAJ_ACCELS(CanRxMsg* CanRevData)
{
	float traptraj_accel_limits, traptraj_decel_limits;
	memcpy(&traptraj_accel_limits, CanRevData->Data, sizeof(float));

	memcpy(&traptraj_decel_limits, CanRevData->Data + sizeof(float), sizeof(float));
	trapTraj_config.accel_limit = traptraj_accel_limits;
	trapTraj_config.decel_limit = traptraj_decel_limits;
}


void OD_MSG_SET_AXIS_REQUESTED_STATE(CanRxMsg* CanRevData)
{
	current_state_ = CanRevData->Data[0];
}

u8 CAN1_Send_Msg(u8* msg,u8 len)
{
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x12;	 // 标准标识符为0
	TxMessage.ExtId=0x12;	 // 设置扩展标示符（29位）
	TxMessage.IDE=0;		  // 使用扩展标识符
	TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
	TxMessage.DLC=len;							 // 发送两帧信息
	for(i=0;i<len;i++)
	TxMessage.Data[i]=msg[i];				 // 第一帧信息
	mbox= CAN_Transmit(CAN1, &TxMessage);
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
	if(i>=0XFFF)return 1;
	return 0;

}
//can口接收数据查询
//buf:数据缓存区;
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
u8 CAN1_Receive_Msg(u8 *buf)
{
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];
	return RxMessage.DLC;
}

static int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}


// 从CAN数据包解析控制命令的函数
// 参数：
//   phandle - CAN处理器句柄，用于存储解析后的控制命令
//   cmd_data - 指向CAN数据包的指针（8字节数组）
static void set_mit_control_cmd(CanRxMsg* CanRevData)
{
    // ==============================================
    // 1. 从8字节CAN数据中解析出各个整型参数
    // ==============================================

    // 解析位置命令p：使用前2个字节（16位）
    // cmd_data[0]是高8位，cmd_data[1]是低8位
    // 示例：cmd_data[0]=0x12, cmd_data[1]=0x34 → p_int = 0x1234
    int p_int = (CanRevData->Data[0] << 8) | CanRevData->Data[1];

    // 解析速度命令v：使用第2字节的低4位和第3字节的高4位（共12位）
    // cmd_data[2]的高4位是v的高4位，cmd_data[3]的高4位是v的低4位
    // 示例：cmd_data[2]=0xAB, cmd_data[3]=0xCD → v_int = (0xAB0) | (0xC) = 0xABC
    int v_int = (CanRevData->Data[2] << 4) | (CanRevData->Data[3] >> 4);

    // 解析比例系数kp：使用第3字节的低4位和第4字节（共12位）
    // cmd_data[3]的低4位是kp的高4位，cmd_data[4]是kp的低8位
    // 示例：cmd_data[3]=0xCD, cmd_data[4]=0xEF → kp_int = (0xD00) | 0xEF = 0xDEF
    int kp_int = ((CanRevData->Data[3] & 0xF) << 8) | CanRevData->Data[4];

    // 解析微分系数kd：使用第5字节和第6字节的高4位（共12位）
    // cmd_data[5]是kd的高8位，cmd_data[6]的高4位是kd的低4位
    // 示例：cmd_data[5]=0x12, cmd_data[6]=0x34 → kd_int = (0x120) | (0x3) = 0x123
    int kd_int = (CanRevData->Data[5] << 4) | (CanRevData->Data[6] >> 4);

    // 解析扭矩命令t：使用第6字节的低4位和第7字节（共12位）
    // cmd_data[6]的低4位是t的高4位，cmd_data[7]是t的低8位
    // 示例：cmd_data[6]=0x34, cmd_data[7]=0x56 → t_int = (0x400) | 0x56 = 0x456
    int t_int = ((CanRevData->Data[6] & 0xF) << 8) | CanRevData->Data[7];

    // ==============================================
    // 2. 将整数转换为实际的物理量浮点值
    // ==============================================

    // 转换位置命令：16位整数 → 浮点位置值（单位：弧度或度）
    // uint_to_float参数：整数，最小值，最大值，位数
    mit_target_pos_ = uint_to_float(p_int, -20000, 20000, 16);

    // 转换速度命令：12位整数 → 浮点速度值（单位：rad/s或RPM）
    mit_target_velocity_ = uint_to_float(v_int, -2000, 2000, 12);

    // 转换比例系数：12位整数 → 浮点比例增益
    mit_kp = uint_to_float(kp_int, -200, 200, 12);

    // 转换微分系数：12位整数 → 浮点微分增益
    mit_kd = uint_to_float(kd_int, -200, 200, 12);

    // 转换扭矩命令：12位整数 → 浮点扭矩值（单位：Nm）
    mit_target_torque_ = uint_to_float(t_int, -20, 20, 12);
}


CanRxMsg can1_rx_msg;
u32 rxbuf3;
u8 flag_iap;
void CAN1_RX0_IRQHandler(void){
	//CanRxMsg can1_rx_msg;
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET){
		// 清除中断标志和标志位
		CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
		CAN_ClearFlag(CAN1, CAN_FLAG_FF0);

		// 从接收 FIFO 中读取消息
		CAN_Receive(CAN1, CAN_FIFO0, &can1_rx_msg);

		// 机致科技中空编码器
		if (can1_rx_msg.StdId == 0x64)
		{
			En_d40_read(&can1_rx_msg);
		}
		// CAN_IAP升级
		// IAP_APP_CAN_ReStart(can1_rx_msg);
		//SaveData(can1_rx_msg);
		// 存储接收到的标准 ID
		rxbuf3=can1_rx_msg.StdId;

		/*********以下是自定义部分**********/
		switch(can1_rx_msg.StdId >> 5){
		    case AXIS0_ID:
				switch(can1_rx_msg.StdId & 0x1F){
				    case MSG_CO_NMT_CTRL:
						// 处理 NMT 控制消息
						break;

					case MSG_CO_HEARTBEAT_CMD:
						// 处理心跳消息
						break;

					case MSG_ODRIVE_HEARTBEAT:
						// 处理设置输入位置消息
						break;

					case MSG_ODRIVE_ESTOP:
						// 处理急停消息
						break;

					case MSG_GET_MOTOR_ERROR:

						// 处理获取电机错误消息
            			break;

					case MSG_GET_ENCODER_ERROR:

						break;

					case MSG_GET_SENSORLESS_ERROR:

						break;

					case MSG_SET_AXIS_NODE_ID:

						break;

					case MSG_SET_AXIS_REQUESTED_STATE:
						// 设置轴请求状态
						OD_MSG_SET_AXIS_REQUESTED_STATE(&can1_rx_msg);
						break;

					case MSG_SET_AXIS_STARTUP_CONFIG:

						break;

					case MSG_GET_ENCODER_ESTIMATES:

						break;

					case MSG_GET_ENCODER_COUNT:

						break;

					case MSG_SET_INPUT_POS:
						// 设置输入位置
						OD_SET_INPUT_POS(&can1_rx_msg);
						break;

					case MSG_SET_INPUT_VEL:
						// 设置输入速度
						OD_SET_INPUT_VEL(&can1_rx_msg);
						break;

					case MSG_SET_INPUT_TORQUE:
						// 设置输入电流
						OD_SET_INPUT_CUR(&can1_rx_msg);
						break;

					case MSG_SET_CONTROLLER_MODES:
						// 设置控制模式
						OD_MSG_SET_CONTROLLER_MODES(&can1_rx_msg);
						break;

					case MSG_SET_LIMITS:
					    // 设置限制
						OD_SET_INPUT_LIMITS(&can1_rx_msg);
						break;

					case MSG_START_ANTICOGGING:

						break;

					case MSG_SET_TRAJ_INERTIA:

						break;

					case MSG_SET_TRAJ_ACCEL_LIMITS:
						// 设置轨迹加速度限制
						OD_SET_TRAPTRAJ_ACCELS(&can1_rx_msg);
						break;

					case MSG_SET_TRAJ_VEL_LIMIT:
						// 设置轨迹速度限制
					    OD_SET_TRAPTRAJ_VEL_LIMIT(&can1_rx_msg);
						break;

					case MSG_GET_IQ:
						// 获取电机电流
						break;

					case MSG_SET_MIT_CONTROL:
						// 设置MIT控制命令
						set_mit_control_cmd(&can1_rx_msg);
						break;

					case MSG_RESET_ODRIVE:
						// 重置 oDrive
						NVIC_SystemReset();
						break;

					case MSG_CLEAR_ERRORS:
						// 清除错误状态
						motor_error = 0;
						break;

					case MSG_SET_LINEAR_COUNT:
						// 设置线性编码器计数
						break;

					case MSG_SET_POS_GAIN:
						// 设置位置环增益
						ODSetPos_gainData(&can1_rx_msg);
						break;

					case MSG_SET_VEL_GAINS:
						// 设置速度环增益
						ODSetVel_gainsData(&can1_rx_msg);
						break;


					//新增
					// case MSG_GET_TEMP:
					//     if (msg.rtr)
					//         get_Temp_callback(axis, txmsg);
					//     break;

					case MSG_SAVE_CONFIG:
						flash_para_write();
						break;

					case MSG_GET_POS_GAIN:

						break;

					case MSG_GET_VEL_GAINS:

						break;


					default:
						break;
					}
		}

	}
}