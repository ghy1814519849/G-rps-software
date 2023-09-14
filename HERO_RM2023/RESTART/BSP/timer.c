#include "main.h"

void TIM8_Configuration(void)
{
		TIM_TimeBaseInitTypeDef tim;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);    
		tim.TIM_Period = 0xFFFFFFFF;     
		tim.TIM_Prescaler = 168-1;	 //1M ��ʱ��  
		tim.TIM_ClockDivision = TIM_CKD_DIV1;	
		tim.TIM_CounterMode = TIM_CounterMode_Up;  
		TIM_ARRPreloadConfig(TIM8, ENABLE);	
		TIM_TimeBaseInit(TIM8, &tim);
		TIM_ARRPreloadConfig(TIM8, ENABLE);	
		TIM_PrescalerConfig(TIM8, 0, TIM_PSCReloadMode_Update);
		TIM_UpdateDisableConfig(TIM8, ENABLE);
		TIM_Cmd(TIM8,ENABLE);	   
}

void TIM6_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  tim;
    NVIC_InitTypeDef         nvic;
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
    nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    tim.TIM_Prescaler = 84-1;        //84M internal clock
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = 0;//TIM_CKD_DIV1;
    tim.TIM_Period = 1000-1;  //1ms,1000Hz
    TIM_TimeBaseInit(TIM6,&tim);
	
    TIM_Cmd(TIM6, ENABLE);	 
    TIM_ITConfig(TIM6, TIM_IT_Update,ENABLE);
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);	
}

void TIM6_DAC_IRQHandler(void)  
{	
    if (TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET) 
	  {
				TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
				TIM_ClearFlag(TIM6, TIM_FLAG_Update);
				Control_Task();
		}
}

void TIM4_Configuration(void)
{
    TIM_TimeBaseInitTypeDef tim;
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE); 

	  NVIC_InitTypeDef         nvic;	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
    nvic.NVIC_IRQChannel = TIM4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
	
    tim.TIM_Period = 0xFFFFFFFF; //ARR��ֵ �Ĵ�������   
    tim.TIM_Prescaler = 168-1;	 //1M ��ʱ��  
    tim.TIM_ClockDivision = TIM_CKD_DIV1;	
    tim.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_ARRPreloadConfig(TIM4, ENABLE);	//�ʡ�����ֹ��ʱ������ʱ��ARR��������д����ֵ���ڸ��·���ʱ���븲����ǰ��ֵ
    TIM_TimeBaseInit(TIM4, &tim);
//    TIM_ARRPreloadConfig(TIM4, ENABLE);	
//    TIM_PrescalerConfig(TIM4, 0, TIM_PSCReloadMode_Update);  //����Ԥ��Ƶ
//    TIM_UpdateDisableConfig(TIM4, ENABLE);  //���������������������������жϲ���
    TIM_Cmd(TIM4,ENABLE);	
		
		TIM_ITConfig(TIM4, TIM_IT_Update,ENABLE);
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);
}

//TIME4
void TIM4_IRQHandler(void)
{
	  if (TIM_GetITStatus(TIM4,TIM_IT_Update)!= RESET) 
		{
				TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
				TIM_ClearFlag(TIM4, TIM_FLAG_Update);

		}
}

void TIM2_Configuration(void)
{
    TIM_TimeBaseInitTypeDef tim;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    tim.TIM_Period = 0xFFFFFFFF;
    tim.TIM_Prescaler = 84 - 1;	 //1M ��ʱ��  
    tim.TIM_ClockDivision = TIM_CKD_DIV1;	
    tim.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_ARRPreloadConfig(TIM2, ENABLE);	
    TIM_TimeBaseInit(TIM2, &tim);
    TIM_Cmd(TIM2,ENABLE);	
}

//TIME2
void TIM2_IRQHandler(void)
{
	  if (TIM_GetITStatus(TIM2,TIM_IT_Update)!= RESET) 
		{
				TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
				TIM_ClearFlag(TIM2, TIM_FLAG_Update);
				BOTH_LED_TOGGLE();
		}
}

int32_t ms_count = 0;
//û�ù��������
uint32_t Get_Time_Micros(void)///////////////////////////////////////////////
{
		return TIM2->CNT;
}
