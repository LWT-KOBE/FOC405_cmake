#include "vofa.h"
#include "MyProject.h"
volatile struct Frame vofaFrame = {
	.tail = {0x00, 0x00, 0x80, 0x7f}
};

volatile struct Frame_Send DeviceFrame = {
    .tail = {0x00, 0x00, 0xAA, 0x55}
};

//RawData����Э�� ����
void RawData_Test(void)		//  ֱ�ӵ���������ʹ�� �����Ƿ����
{
   
}

//FireWater����Э�� ����
float a=5,b=10,c=20;
void FireWater_Test(void)
{
    a+=100;
    b+=50;
    c+=10;
    //u1_printf("%.2f,%.2f,%.2f\n",a,b,c);
}


//ʹ��justfloat ����Э�� ����

/*
Ҫ����ʾ:
1. float��unsigned long������ͬ�����ݽṹ����
2. union������������ݴ������ͬ������ռ�
*/



/*
��������fת��Ϊ4���ֽ����ݴ����byte[4]��
*/
void Float_to_Byte(float f,unsigned char byte[])
{
    FloatLongType fl;
    fl.fdata=f;
    byte[0]=(unsigned char)fl.ldata;
    byte[1]=(unsigned char)(fl.ldata>>8);
    byte[2]=(unsigned char)(fl.ldata>>16);
    byte[3]=(unsigned char)(fl.ldata>>24);
}

/*
��4���ֽ�����byte[4]ת��Ϊ�����������*f��
*/
void Byte_to_Float(float *f,unsigned char byte[])
{
    FloatLongType fl;
    fl.ldata=0;
    fl.ldata=byte[3];
    fl.ldata=(fl.ldata<<8)|byte[2];
    fl.ldata=(fl.ldata<<8)|byte[1];
    fl.ldata=(fl.ldata<<8)|byte[0];
    *f=fl.fdata;
}



/*  ����֡��ʽ
typedef struct
{	
	u8 byte[4];		//floatת��Ϊ4���ֽ�����
	u8 tail[4];	//֡β
}Frame_TypeDef;
	byte ������ת��������	tail ֡β
u8 byte[4]={0};		//floatת��Ϊ4���ֽ�����
u8 tail[4]={0x00, 0x00, 0x80, 0x7f};	//֡β
*/



// ��vofa��������  ��������  ����ͨ��  ���ӻ���ʾ  ֡β
// void vofa_sendData(float a, float b)
// {
//     u8 byte[4] = {0};                      // floatת��Ϊ4���ֽ�����
//     u8 tail[4] = {0x00, 0x00, 0x80, 0x7f}; // ֡β

//     // ����λ����������ͨ������
//     Float_to_Byte(a, byte);
//     // u1_printf("%f\r\n",a);
//     // u1_SendArray(byte, 4); // 

//     Float_to_Byte(b, byte);
//     // u1_SendArray(byte, 4); // 

    
//     // // ����֡β
//     // u1_SendArray(tail, 4); // ֡βΪ 0x00 0x00 0x80 0x7f
// }

// ��vofa��������  1������  1��ͨ��  ���ӻ���ʾ  ֡β
void Vofa_sendData(float Byte)
{
    u8 byte[4] = {0};                      // floatת��Ϊ4���ֽ�����
    u8 tail[4] = {0x00, 0x00, 0x80, 0x7f}; // ֡β

    // ����λ����������ͨ������
    Float_to_Byte(Byte, byte);
//	snd2_buff = byte;
	u2_SendArray(byte,4);
    // u1_SendArray(byte, 4); // 1ת��Ϊ4�ֽ����� ����  0x00 0x00 0x80 0x3F

    // // ����֡β
    // u1_SendArray(tail, 4); // ֡βΪ 0x00 0x00 0x80 0x7f
	u2_SendArray(tail,4);
}

/*
*********************************************************************************************************
*	�� �� ��: vofa_print
*	����˵��: ����vfoa����֡
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void vofa_print(void)
{
    u2_SendArray((uint8_t *) (&vofaFrame), sizeof(vofaFrame));
}

/**
 * @brief  ͨ��USB����vofa����֡
 * @param  ��
 */
void vofa_printf_USB(void)
{
   usb_send((uint8_t *) (&vofaFrame), sizeof(vofaFrame));
}


