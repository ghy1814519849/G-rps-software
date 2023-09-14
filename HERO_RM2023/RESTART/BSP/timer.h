#ifndef __TIMER_H__
#define __TIMER_H__
#include "main.h"

void TIM2_Configuration(void);
void TIM4_Configuration(void);
void TIM6_Configuration(void);
void TIM8_Configuration(void);

void TIM6_DAC_IRQHandler(void);
void TIM4_IRQHandler(void);
void TIM2_IRQHandler(void);
uint32_t Get_Time_Micros(void);
#endif


