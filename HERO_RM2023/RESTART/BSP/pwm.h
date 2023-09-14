#ifndef __PWM_H__
#define __PWM_H__
#include "main.h"

#define PWM_STOP 0//����
#define PWM_UP 1
#define PWM_DOWN 2

#define PWM_MAX 140//�����е��λ0-270���ڻ�е��Χ�м��ƶ�
#define PWM_MID 133
#define PWM_MIN 85

//#define PWM1  TIM3->CCR3
#define PWM2  TIM3->CCR4//ͼ���������

extern int pwm_flag;
extern u32 pwm_mode;

void PWM_Configuration(void);
void PWM(void);

#endif /* __GUN_H__*/

