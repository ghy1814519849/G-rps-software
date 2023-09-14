#include "main.h"

void BSP_Init(void)
{
		ControtLoopTaskInit();//���������ʼ��

		delay_ms(500);
	
		Led_Configuration();
		TIM8_Configuration();
		TIM6_Configuration();
		TIM4_Configuration();
		TIM2_Configuration();
		Laser_Configuration();
		USART2_Configuration();//����2
		UART4_Configuration();//���� DMA
		USART3_Configuration();//����3

		USART6_Configuration_For_Hi220();//������
//		ch100_USART_Config();//������
	
		GMPitchEncoder.ecd_bias = GMPitchEncoder_Offset;//��̨Pitch���ʼ�Ƕ�
		GMYawEncoder.ecd_bias 	= GMYawEncoder_Offset;//��̨Yaw���ʼ�Ƕ�
		BUS1_CM9Encoder.ecd_bias		=	CameraEncoder_Offset;
		ddt_SetMode(0x01);//���õ��ģʽ
		BSP_UART5_InitConfig();//����ϵͳ����5DMAͨ��ʹ��
		communicate_param_init();//����ϵͳ�������ݴ��������ʼ��
	
		CAN1_Configuration();//can1ת�����ʼ��
		CAN2_Configuration();//ң�������Ƶ��̵��
		USART1_Configuration(100000);//ң����

		
}


