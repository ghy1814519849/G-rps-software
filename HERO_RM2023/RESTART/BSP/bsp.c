#include "main.h"

void BSP_Init(void)
{
		ControtLoopTaskInit();//控制任务初始化

		delay_ms(500);
	
		Led_Configuration();
		TIM8_Configuration();
		TIM6_Configuration();
		TIM4_Configuration();
		TIM2_Configuration();
		Laser_Configuration();
		USART2_Configuration();//串口2
		UART4_Configuration();//自瞄 DMA
		USART3_Configuration();//串口3

		USART6_Configuration_For_Hi220();//陀螺仪
//		ch100_USART_Config();//陀螺仪
	
		GMPitchEncoder.ecd_bias = GMPitchEncoder_Offset;//云台Pitch轴初始角度
		GMYawEncoder.ecd_bias 	= GMYawEncoder_Offset;//云台Yaw轴初始角度
		BUS1_CM9Encoder.ecd_bias		=	CameraEncoder_Offset;
		ddt_SetMode(0x01);//设置电机模式
		BSP_UART5_InitConfig();//裁判系统串口5DMA通信使能
		communicate_param_init();//裁判系统接收数据处理参数初始化
	
		CAN1_Configuration();//can1转电机初始化
		CAN2_Configuration();//遥控器控制底盘电机
		USART1_Configuration(100000);//遥控器

		
}


