#ifndef __CAN2_H__
#define __CAN2_H__
#include "stm32f4xx.h"
#include "main.h"

void CAN2_Configuration(void);
void CAN2_TX_IRQHandler(void);
void CAN2_RX0_IRQHandler(void);


#define RATE_BUF_SIZE 6
typedef struct{
	int32_t raw_value;   									//编码器不经处理的原始值
	int32_t last_raw_value;								//上一次的编码器原始值
	int32_t ecd_value;                       //经过处理后连续的编码器值
	int32_t diff;													//两次编码器之间的差值
	int32_t temp_count;                   //计数用
	uint8_t buf_count;										//滤波更新buf用
	int32_t ecd_bias;											//初始编码器值	
	int32_t ecd_raw_rate;									//通过编码器计算得到的速度原始值
	int32_t rate_buf[RATE_BUF_SIZE];			//buf，for filter
	int32_t round_cnt;										//圈数
	int32_t filter_rate;									//速度
	double ecd_angle;		 									//角度
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
