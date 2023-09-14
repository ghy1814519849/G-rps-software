#ifndef __USART3_H
#define __USART3_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "main.h"

#define USART2_RX_BUF_LENGTH 10
#define USART2_TX_BUF_LENGTH 10


extern u8 USART2_DMA_RX_BUF[USART2_RX_BUF_LENGTH];
extern u8 USART2_DMA_TX_BUF[USART2_TX_BUF_LENGTH];

void USART2_Configuration(void);
void Usart2DmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
void Usart2SendByteInfoProc(u8 nSendInfo);
void Usart2SendBytesInfoProc(u8* pSendInfo, u16 nSendCount);


#endif
