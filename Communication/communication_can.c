#include "communication_can.h"

void Motor_CAN_Send_Data(void){

    // 发送预测的位置值和速度值
    if(can_cnt % 10 == 0)
        OD_CANSendData_2(CAN1,OD_CANID,MSG_GET_ENCODER_ESTIMATES,8,pos_estimate_,vel_estimate_,&ODSendData);
    // 发送Iq目标值和测量值
    if(can_cnt % 10 == 0)
        OD_CANSendData_2(CAN1,OD_CANID,MSG_GET_IQ,8,Idq_setpoint_.q,Iq_measured,&ODSendData);
    // 母线电压和电流
    if(can_cnt % 100 == 0)
        OD_CANSendData_2(CAN1,OD_CANID,MSG_GET_BUS_VOLTAGE_CURRENT,8,vbus_voltage,Ibus,&ODSendData);


    if (can_cnt % 50 == 0)
        CANSendData_2(CAN1,0x01,8,0,0,&ODSendData);
    // 发送板载、电机温度值
    if(can_cnt % 100 == 0)
        OD_CANSendData_2(CAN1,OD_CANID,MSG_GET_MOTOR_TEMP,8,temperature_board,temperature_motor,&ODSendData);
}
