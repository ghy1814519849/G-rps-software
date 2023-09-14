#include "main.h"

u8 USART2_DMA_RX_BUF[USART2_RX_BUF_LENGTH] = {0};
u8 USART2_DMA_TX_BUF[USART2_TX_BUF_LENGTH] = {0};

void USART2_Configuration()
{
		USART_InitTypeDef usart;
		GPIO_InitTypeDef  gpio;
		NVIC_InitTypeDef  nvic;
		DMA_InitTypeDef dma;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

		//串口2对应引脚复用映射
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2复用为USART2
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART2

		//USART2端口配置
		gpio.GPIO_Pin 	= GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2与GPIOA3
		gpio.GPIO_Mode 	= GPIO_Mode_AF;//复用功能
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;	//速度50MHz
		gpio.GPIO_PuPd 	= GPIO_PuPd_UP; //上拉
		GPIO_Init(GPIOA,&gpio); //初始化PA2，PA3

		USART_DeInit(USART2);
		usart.USART_BaudRate = 115200;
		usart.USART_WordLength = USART_WordLength_8b;
		usart.USART_StopBits = USART_StopBits_1;
		usart.USART_Parity = USART_Parity_No;
		usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
		usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART2, &usart);
//接收DMA
		USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
		DMA_DeInit(DMA1_Stream5);
		while(DMA_GetCmdStatus(DMA1_Stream5) != DISABLE) {}

		dma.DMA_Channel 						= DMA_Channel_4;
		dma.DMA_PeripheralBaseAddr	= (uint32_t)(&USART2->DR);
		dma.DMA_Memory0BaseAddr   	= (uint32_t)&USART2_DMA_RX_BUF[0];
		dma.DMA_DIR 			    			= DMA_DIR_PeripheralToMemory;
		dma.DMA_BufferSize					= USART2_RX_BUF_LENGTH;
		dma.DMA_PeripheralInc 			= DMA_PeripheralInc_Disable;
		dma.DMA_MemoryInc 					= DMA_MemoryInc_Enable;
		dma.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
		dma.DMA_MemoryDataSize 			= DMA_MemoryDataSize_Byte;
		dma.DMA_Mode 								= DMA_Mode_Normal;
		dma.DMA_Priority 						= DMA_Priority_Medium;
		dma.DMA_FIFOMode 						= DMA_FIFOMode_Disable;
		dma.DMA_FIFOThreshold 			= DMA_FIFOThreshold_Full;
		dma.DMA_MemoryBurst 				= DMA_MemoryBurst_Single;
		dma.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;

		DMA_Init(DMA1_Stream5,&dma);
		DMA_Cmd(DMA1_Stream5, ENABLE);

		nvic.NVIC_IRQChannel = USART2_IRQn;   // 发送DMA通道的中断配置
		nvic.NVIC_IRQChannelPreemptionPriority = 4;     // 优先级设置
		nvic.NVIC_IRQChannelSubPriority = 0;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
//发送DMA
		USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);

		DMA_Cmd(DMA1_Stream6, DISABLE);                           // 关DMA通道
		DMA_DeInit(DMA1_Stream6);
		while(DMA_GetCmdStatus(DMA1_Stream6) != DISABLE) {}
		dma.DMA_Channel 						= DMA_Channel_4;
		dma.DMA_PeripheralBaseAddr	= (uint32_t)(&USART2->DR);
		dma.DMA_Memory0BaseAddr   	= (uint32_t)&USART2_DMA_TX_BUF[0];
		dma.DMA_DIR 			   				= DMA_DIR_MemoryToPeripheral;
		dma.DMA_BufferSize					= 0;//USART2_TX_BUF_LENGTH;
		dma.DMA_PeripheralInc 			= DMA_PeripheralInc_Disable;
		dma.DMA_MemoryInc 					= DMA_MemoryInc_Enable;
		dma.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
		dma.DMA_MemoryDataSize 			= DMA_MemoryDataSize_Byte;
		dma.DMA_Mode 								= DMA_Mode_Normal;
		dma.DMA_Priority 						= DMA_Priority_Medium;
		dma.DMA_FIFOMode 						= DMA_FIFOMode_Disable;
		dma.DMA_FIFOThreshold 			= DMA_FIFOThreshold_Full;
		dma.DMA_MemoryBurst 				= DMA_MemoryBurst_Single;
		dma.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
		DMA_Init(DMA1_Stream6,&dma);
			
		nvic.NVIC_IRQChannel = DMA1_Stream6_IRQn;   // 发送DMA通道的中断配置
		nvic.NVIC_IRQChannelPreemptionPriority = 4;     // 优先级设置
		nvic.NVIC_IRQChannelSubPriority = 0;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
		DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);
			
		USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled
		USART_Cmd(USART2,ENABLE);
}

void USART2_IRQHandler(void)
{
		uint8_t length=0;
		if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)    //接收中断
    {
				(void)USART2->SR;
				(void)USART2->DR;//进中断后读SR，再读DR寄存器后清除本次中断USART_IT_IDLE标志位，否则读到的值落后了一帧
				DMA_Cmd(DMA1_Stream5, DISABLE);//关闭接收DMA
				DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);//清除标志位
				length = USART2_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA1_Stream5);//获得接收帧长度 
				//函数DMA_GetCurrDataCounter(DMA2_Stream5); 获取当前指针计数值,用内存缓冲区大小 - 此计数值 = 接收到的数据长度

				ddtEncoderProcess(USART2_DMA_RX_BUF);
				DMA_SetCurrDataCounter(DMA1_Stream5,USART2_RX_BUF_LENGTH);//设置传输数据长度
				DMA_Cmd(DMA1_Stream5, ENABLE);//打开DMA
    }
}


void Usart2DmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)

{
//    DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输
//    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}  //确保DMA可以被设置
		DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量
		DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输
}


void DMA1_Stream6_IRQHandler(void)
{
		//清除标志
		if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=RESET)//等待DMA1_Steam3传输完成
    {
				DMA_Cmd(DMA1_Stream6, DISABLE);                      //关闭DMA传输
				DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);//清除DMA1_Steam3传输完成标志
    }

}


//发送单字节
void Usart2SendByteInfoProc(u8 nSendInfo)
{
		u8 *pBuf = NULL;
		//指向发送缓冲区
		pBuf = USART2_DMA_TX_BUF;
		*pBuf++ = nSendInfo;
		Usart2DmaSendDataProc(DMA1_Stream6,1); //开始一次DMA传输！
}

//发送多字节

void Usart2SendBytesInfoProc(u8* pSendInfo, u16 nSendCount)
{
		u16 i = 0;
		u8 *pBuf = NULL;
		//指向发送缓冲区
		pBuf = USART2_DMA_TX_BUF;
		for (i=0; i<nSendCount; i++)
    {
				*(pBuf+i) = pSendInfo[i];
    }
		//DMA发送方式
		Usart2DmaSendDataProc(DMA1_Stream6,nSendCount); //开始一次DMA传输！
}

