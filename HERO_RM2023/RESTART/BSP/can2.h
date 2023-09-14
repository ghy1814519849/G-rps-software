#ifndef __CAN2_H__
#define __CAN2_H__
#include "stm32f4xx.h"
#include "main.h"

void CAN2_Configuration(void);
void CAN2_TX_IRQHandler(void);
void CAN2_RX0_IRQHandler(void);


#define RATE_BUF_SIZE 6
typedef struct{
	int32_t raw_value;   									//���������������ԭʼֵ
	int32_t last_raw_value;								//��һ�εı�����ԭʼֵ
	int32_t ecd_value;                       //��������������ı�����ֵ
	int32_t diff;													//���α�����֮��Ĳ�ֵ
	int32_t temp_count;                   //������
	uint8_t buf_count;										//�˲�����buf��
	int32_t ecd_bias;											//��ʼ������ֵ	
	int32_t ecd_raw_rate;									//ͨ������������õ����ٶ�ԭʼֵ
	int32_t rate_buf[RATE_BUF_SIZE];			//buf��for filter
	int32_t round_cnt;										//Ȧ��
	int32_t filter_rate;									//�ٶ�
	double ecd_angle;		 									//�Ƕ�
	u32 temperature;
	int32_t current;
	int16_t rate_rpm;
}Encoder;

typedef struct
{
    int16_t  rotor_angle;
    int16_t  rotor_speed;
	  float    rotor_speed_fdata;
    int16_t  torque_current;
	  uint16_t rotor_angle_last;
	  float    rotor_out;
	  float    rotor_out_fdata;
		float    rotor_basic;
		int  DIAL_N;
}M2006_info_t;

#endif 
