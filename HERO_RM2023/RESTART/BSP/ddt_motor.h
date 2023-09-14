#ifndef __DDT_MOTOR_H
#define __DDT_MOTOR_H

#include "main.h"

typedef struct
{
	uint16_t ID;						//ID
	uint16_t Mode;					//ģʽ
	int16_t current;				//ת�ص���
	int16_t rate_rpm;				//ת��
	int16_t ecd_value;			//������λ��
	uint16_t error_gate;		//������
	uint16_t crc_check;			//CRCУ��

	int16_t ecd_bias;			//��ʼֵ
	int rount_count;		//Ȧ��
	int16_t last_value;	//��һ�εı�����λ��
	float angle;					//�Ƕ�(�ۼ�)
	
}ddtEncoder_t;

extern int16_t speed_ddt;
extern volatile ddtEncoder_t ddt_Encoder;

extern void ddtEncoderProcess(u8* msg);
extern void ddt_SetMotor(int16_t val);
extern void ddt_SetMode(u16 mode);
extern void ddt_SetID(u16 ID);
extern void ddt_Stop(u16 stop);


#endif
