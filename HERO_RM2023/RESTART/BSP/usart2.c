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

		//����2��Ӧ���Ÿ���ӳ��
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2����ΪUSART2
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3����ΪUSART2

		//USART2�˿�����
		gpio.GPIO_Pin 	= GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2��GPIOA3
		gpio.GPIO_Mode 	= GPIO_Mode_AF;//���ù���
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�50MHz
		gpio.GPIO_PuPd 	= GPIO_PuPd_UP; //����
		GPIO_Init(GPIOA,&gpio); //��ʼ��PA2��PA3

		USART_DeInit(USART2);
		usart.USART_BaudRate = 115200;
		usart.USART_WordLength = USART_WordLength_8b;
		usart.USART_StopBits = USART_StopBits_1;
		usart.USART_Parity = USART_Parity_No;
		usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
		usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART2, &usart);
//����DMA
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

		nvic.NVIC_IRQChannel = USART2_IRQn;   // ����DMAͨ�����ж�����
		nvic.NVIC_IRQChannelPreemptionPriority = 4;     // ���ȼ�����
		nvic.NVIC_IRQChannelSubPriority = 0;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
//����DMA
		USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);

		DMA_Cmd(DMA1_Stream6, DISABLE);                           // ��DMAͨ��
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
			
		nvic.NVIC_IRQChannel = DMA1_Stream6_IRQn;   // ����DMAͨ�����ж�����
		nvic.NVIC_IRQChannelPreemptionPriority = 4;     // ���ȼ�����
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
		if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)    //�����ж�
    {
				(void)USART2->SR;
				(void)USART2->DR;//���жϺ��SR���ٶ�DR�Ĵ�������������ж�USART_IT_IDLE��־λ�����������ֵ�����һ֡
				DMA_Cmd(DMA1_Stream5, DISABLE);//�رս���DMA
				DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);//�����־λ
				length = USART2_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA1_Stream5);//��ý���֡���� 
				//����DMA_GetCurrDataCounter(DMA2_Stream5); ��ȡ��ǰָ�����ֵ,���ڴ滺������С - �˼���ֵ = ���յ������ݳ���

				ddtEncoderProcess(USART2_DMA_RX_BUF);
				DMA_SetCurrDataCounter(DMA1_Stream5,USART2_RX_BUF_LENGTH);//���ô������ݳ���
				DMA_Cmd(DMA1_Stream5, ENABLE);//��DMA
    }
}


void Usart2DmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)

{
//    DMA_Cmd(DMA_Streamx, DISABLE);                      //�ر�DMA����
//    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}  //ȷ��DMA���Ա�����
		DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //���ݴ�����
		DMA_Cmd(DMA_Streamx, ENABLE);                      //����DMA����
}


void DMA1_Stream6_IRQHandler(void)
{
		//�����־
		if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=RESET)//�ȴ�DMA1_Steam3�������
    {
				DMA_Cmd(DMA1_Stream6, DISABLE);                      //�ر�DMA����
				DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);//���DMA1_Steam3������ɱ�־
    }

}


//���͵��ֽ�
void Usart2SendByteInfoProc(u8 nSendInfo)
{
		u8 *pBuf = NULL;
		//ָ���ͻ�����
		pBuf = USART2_DMA_TX_BUF;
		*pBuf++ = nSendInfo;
		Usart2DmaSendDataProc(DMA1_Stream6,1); //��ʼһ��DMA���䣡
}

//���Ͷ��ֽ�

void Usart2SendBytesInfoProc(u8* pSendInfo, u16 nSendCount)
{
		u16 i = 0;
		u8 *pBuf = NULL;
		//ָ���ͻ�����
		pBuf = USART2_DMA_TX_BUF;
		for (i=0; i<nSendCount; i++)
    {
				*(pBuf+i) = pSendInfo[i];
    }
		//DMA���ͷ�ʽ
		Usart2DmaSendDataProc(DMA1_Stream6,nSendCount); //��ʼһ��DMA���䣡
}

