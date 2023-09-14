#include "main.h"
static uart_dma_rxdata_t judge_rx_obj;
static unpack_data_t judge_unpack_obj;
static FIFO_S_t judge_rxdata_fifo;
static uint8_t judge_rxdata_buf[JUDGE_FRAME_BUFLEN];
static uint8_t judge_txdata_buf[JUDGE_FRAME_BUFLEN];
static uint8_t USART5_DMA_TX_BUF[UART5_TX_BUF_LENGTH];
uint8_t judge_dma_rxbuff[2][UART_RX_DMA_SIZE];
receive_judge_t judge_rece_mesg;

uint8_t  tx_buf[BSP_UART5_DMA_TX_BUF_LEN];
uint8_t  pdata[19];
uint8_t  ddata[62];
uint8_t CRC_SEND_COLOR_BUF[5] = {0xB5,0};

uint8_t _UART5_DMA_RX_BUF[2][BSP_UART5_DMA_RX_BUF_LEN];

static uint8_t _UART5_RX_BUF[BSP_UART5_RX_BUF_SIZE_IN_FRAMES * BSP_UART5_DMA_RX_BUF_LEN];

FIFO_S_t  _UART5_RX_FIFO;

/***********************************    ↓    DJI提供的CRC校检函数   ↓  ***********************************/
//crc8 generator polynomial:G(x)=x8+x5+x4+1
const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] =
{
  0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
  0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
  0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
  0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
  0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
  0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
  0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
  0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
  0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
  0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
  0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
  0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
  0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
  0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
  0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
  0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8)
{
  unsigned char ucIndex;
  while (dwLength--)
    {
      ucIndex = ucCRC8^(*pchMessage++);
      ucCRC8 = CRC8_TAB[ucIndex];
    }
  return(ucCRC8);
}

/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/

unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
  unsigned char ucExpected = 0;
  if ((pchMessage == 0) || (dwLength <= 2))
    return 0;
  ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT);
  return ( ucExpected == pchMessage[dwLength-1] );
}

unsigned char get_crc8(unsigned char* data, unsigned int length)
{
	unsigned char ucExpected = 0;
  if ((data == 0) || (length <= 2))
    return 0xFF;
  return Get_CRC8_Check_Sum (data, length, CRC8_INIT);
}

/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
  unsigned char ucCRC = 0;
  if ((pchMessage == 0) || (dwLength <= 2))
    return;
  ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT);
  pchMessage[dwLength-1] = ucCRC;
}

uint16_t CRC_INIT = 0xffff;

const uint16_t wCRC_Table[256] =
{
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
  0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
  0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
  0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
  0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
  0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
  0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
  0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
  0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
  0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
  0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
  0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
  0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
  0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
  0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
  0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
  0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
  0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
  0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
  0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
  0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
  0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/

uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
  uint8_t chData;
  if (pchMessage ==NULL)
    {
      return 0xFFFF;
    }
  while(dwLength--)
    {
      chData = *pchMessage++;
      (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^
                                  (uint16_t)(chData)) & 0x00ff];
    }
  return wCRC;
}

/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
  uint16_t wExpected = 0;
  if ((pchMessage == NULL) || (dwLength <= 2))
    {
      return 0;
//		return __FALSE;
    }
  wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);
  return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}
/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength)
{
  uint16_t wCRC = 0;
  if ((pchMessage == 0) || (dwLength <= 2))
    {
      return;
    }
  wCRC = Get_CRC16_Check_Sum ( (u8 *)pchMessage, dwLength-2, CRC_INIT );
  pchMessage[dwLength-2] = (u8)(wCRC & 0x00ff);
  pchMessage[dwLength-1] = (u8)((wCRC >> 8)& 0x00ff);
}
/***********************************    ↑    DJI提供的CRC校检函数   ↑  ***********************************/

/**
  * @brief  UART初始化
  * @param  void
  * @retval void
  */
FIFO_S_t* UART5_TranFifo;
void BSP_UART5_InitConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  //    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  //    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  //    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  //UART3(Judge)

  GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_12;
  GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_2;
  GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOC,GPIO_PinSource12, GPIO_AF_UART5);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource2, GPIO_AF_UART5);

  USART_InitStructure.USART_BaudRate              =   115200;
  USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode                  =    USART_Mode_Tx|USART_Mode_Rx;
  USART_InitStructure.USART_Parity                =   USART_Parity_No;
  USART_InitStructure.USART_StopBits              =   USART_StopBits_1;
  USART_InitStructure.USART_WordLength            =   USART_WordLength_8b;
  USART_Init(UART5, &USART_InitStructure);

  USART_Cmd(UART5, ENABLE);
  //UART5_FIFO_Init();
  USART_DMACmd(UART5, USART_DMAReq_Rx, ENABLE);
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_Channel           =   DMA_Channel_4;
  DMA_InitStructure.DMA_PeripheralBaseAddr=   (uint32_t)(&UART5->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr   =   (uint32_t)&_UART5_DMA_RX_BUF[0][0];
  DMA_InitStructure.DMA_DIR               =   DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize        =   sizeof(_UART5_DMA_RX_BUF)/2;
  DMA_InitStructure.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc         =   DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_MemoryDataSize    =   DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralDataSize=   DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_Mode              =   DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority          =   DMA_Priority_Low  ;
  DMA_InitStructure.DMA_FIFOMode          =   DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold     =   DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst       =   DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream0, &DMA_InitStructure);

  USART_DMACmd(UART5, USART_DMAReq_Tx, ENABLE);
  DMA_Cmd(DMA1_Stream7, DISABLE);                           // 关DMA通道
  DMA_DeInit(DMA1_Stream7);
  while(DMA_GetCmdStatus(DMA1_Stream7) != DISABLE) {}
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_PeripheralBaseAddr	= (uint32_t)(&UART5->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr   	= (uint32_t)&tx_buf[0];
  DMA_InitStructure.DMA_DIR 			    = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize			= sizeof(tx_buf);
  DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode 				= DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority 			= DMA_Priority_Medium;
  DMA_InitStructure.DMA_FIFOMode 			= DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst 		= DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream7,&DMA_InitStructure);

//	DMA_Cmd(DMA1_Stream3, DISABLE);                           // 关DMA通道
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream7_IRQn;   // 发送DMA通道的中断配置
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     // 优先级设置
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
  //配置Memory1,Memory0是第一个使用的Memory
  DMA_DoubleBufferModeConfig(DMA1_Stream0,  (uint32_t)&_UART5_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
  DMA_DoubleBufferModeCmd(DMA1_Stream0, ENABLE);
  DMA_Cmd(DMA1_Stream0, ENABLE);
  NVIC_InitStructure.NVIC_IRQChannel						=	UART5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	0;
  NVIC_Init(&NVIC_InitStructure);
  USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);
  USART_Cmd(UART5, ENABLE);
  UART5_TranFifo = FIFO_S_Create(100);
  if(!UART5_TranFifo)
    {
      // while(1);  avoid while in program
    }
}
//void ESP8266_SendPack(uint16_t Num)
//{
//    if(DMA1_Stream3->NDTR)
//    {
//        return;
//    }
//    DMA_Cmd(DMA1_Stream3, DISABLE);                                     //关闭 DMA 传输
//    while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE){}                 //确保 DMA 可以被设置
//    DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3);       //清空标志位
//    DMA1_Stream3->M0AR = (uint32_t)ESP8266TXBuffer;              //设置数据
//    DMA_SetCurrDataCounter(DMA1_Stream3, Num);                          //数据传输量
//    DMA_Cmd(DMA1_Stream3, ENABLE);                                      //开启 DMA 传输
//}
void UART5_PrintBlock(uint8_t* pdata, uint8_t len)
{
  uint8_t i = 0;
  for(i = 0; i < len; i++)
    {
      FIFO_S_Put(UART5_TranFifo,pdata[i]);
    }
  USART_ITConfig(UART5,USART_IT_TXE,ENABLE);  //发送寄存器空中断
}

static void UART5_FIFO_Init(void)
{
  // FIFO_S_Init(&_UART5_RX_FIFO, (void *)_UART5_RX_BUF, RC_FRAME_LENGTH, BSP_UART5_RX_BUF_SIZE_IN_FRAMES);
}

void *UART5_GetRxBuf(void)
{
  return (void *)&_UART5_RX_FIFO;
}

/**
  * @brief  裁判系统初始化
  * @param  void
  * @retval void
  */

/**
  * @brief  裁判系统空闲中断(UART4)
  * @param  void
  * @retval void
  * @note   DJI很毒,以50HZ发送Info包，当有实时数据包产生时，在最近的一帧Info前无间隔加上实时数据包
  */

//开启一次DMA传输
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
//ndtr:数据传输量
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{

  DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输

  while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE) {}	//确保DMA可以被设置

  DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量

  DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输
}

void UART5_IRQHandler(void)
{
  static uint32_t this_time_rx_len = 0;
  if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)
    {
      (void)UART5->SR;
      (void)UART5->DR;

      if(DMA_GetCurrentMemoryTarget(DMA1_Stream0) == 0)
        {
          DMA_Cmd(DMA1_Stream0, DISABLE);
          this_time_rx_len = BSP_UART5_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream0);
          DMA1_Stream0->NDTR =(uint16_t)BSP_UART5_DMA_RX_BUF_LEN;     //relocate the dma memory pointer to the beginning position
          DMA1_Stream0->CR |= (uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 1
          DMA_Cmd(DMA1_Stream0, ENABLE);
					
          FIFO_S_Puts(&_UART5_RX_FIFO,(void *)&_UART5_DMA_RX_BUF[0][0],this_time_rx_len);
          unpack_fifo_data(&judge_unpack_obj,DN_REG_ID);
        }
      else
        {
          DMA_Cmd(DMA1_Stream0, DISABLE);
          this_time_rx_len =BSP_UART5_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream0);
          DMA1_Stream0->NDTR = (uint16_t)BSP_UART5_DMA_RX_BUF_LEN;      //relocate the dma memory pointer to the beginning position
          DMA1_Stream0->CR &= ~(uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 0
          DMA_Cmd(DMA1_Stream0, ENABLE);
					
          FIFO_S_Puts(&_UART5_RX_FIFO,(void *)&_UART5_DMA_RX_BUF[1][0],this_time_rx_len);
          unpack_fifo_data(&judge_unpack_obj,DN_REG_ID);
          //judgement_data_handle(_UART5_DMA_RX_BUF[1]);
        }
    }
}
/* data send (forward) */
/* data receive */
receive_judge_t judge_rece_mesg;
int COUNTER=0;
/**
  * @brief    get judgement system message
  */

int hero_42mm_shot_bullet_num = 0;
int hero_remain_hp = 0;
int hero_remain_last_hp = 0;
uint32_t event_data;
uint32_t event_X2_data = 0;
uint32_t event_X3_data = 0;
uint32_t event_X4_data = 0;
u8 robo_recive[4];
float pitch_data = 0;
float yaw_data = 0;
void judgement_data_handle(uint8_t *p_frame)
{
  frame_header_t *p_header = (frame_header_t*)p_frame;
  memcpy(p_header, p_frame, HEADER_LEN);
  uint16_t data_length = p_header->data_length;//数据帧中DATA段的长度
  uint16_t cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN);
  uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;//数据帧中DATA段的指针
  switch (cmd_id)
    {
    case GAME_STATE_ID://比赛状态数据：0x0001。发送频率：1Hz
      memcpy(&judge_rece_mesg.game_state, data_addr, data_length);
      break;

    case GAME_RESULT_ID://比赛结果数据：0x0002。发送频率：比赛结束后发送
      memcpy(&judge_rece_mesg.game_result, data_addr, data_length);
      break;

    case GAME_ROBOT_HP_ID	 ://机器人存活数据：0x0003。发送频率：1Hz
		{
			memcpy(&judge_rece_mesg.game_robot_HP, data_addr, data_length);
			hero_remain_last_hp = hero_remain_hp;
			hero_remain_hp = judge_rece_mesg.game_robot_HP.blue_1_robot_HP;//剩余血量
			if(hero_remain_last_hp != hero_remain_hp)
				Vehicle_Send_Data();
		}
			break;

    case EVENT_DADA_ID://场地事件数据：0x0101。发送频率：事件改变后发送
    {
			memcpy(&judge_rece_mesg.event_data, data_addr, data_length);
			event_data = judge_rece_mesg.event_data.event_type;
			event_X2_data = 1;//(event_data >> 6) & 1UL;//R2/B2状态		//取32位的状态数据中第6位的大小  //占领为1
			event_X3_data = 1;//(event_data >> 7) & 1UL;//R3/B3状态
			event_X4_data = 1;//(event_data >> 8) & 1UL;//R3/B3状态
		}
      break;

    case SUPPLY_PROJECTILE_ACTION_ID://补给站动作标识：0x0102。发送频率：动作改变后发送
      memcpy(&judge_rece_mesg.supply_projectile_action, data_addr, data_length);
      break;

    case SUPPLY_PROJECTILE_BOOKING_ID://请求补给站补弹子弹：cmd_id (0x0103)。发送频率：上限 10Hz
      memcpy(&judge_rece_mesg.supply_projectile_booking, data_addr, data_length);
      break;
		case REFEREE_WARNING_ID://裁判警告信息：cmd_id (0x0104)。发送频率：警告发生后发送
      memcpy(&judge_rece_mesg.referee_warning, data_addr, data_length);
			break;
    case GAME_ROBOT_STATE_ID:// 比赛机器人状态：0x0201。发送频率：10Hz
    { 
			memcpy(&judge_rece_mesg.game_robot_state, data_addr, data_length);
		}
    break;
    case POWER_HEAT_DATA_ID://实时功率热量数据：0x0202。发送频率：50Hz
    {
      memcpy(&judge_rece_mesg.power_heat_data, data_addr, data_length);
      Power_Control.Cnt_Power_Judge_Recieved++;
    }
    break;
    case GAME_ROBOT_POS_ID://机器人位置：0x0203。发送频率：10Hz
      memcpy(&judge_rece_mesg.game_robot_pos, data_addr, data_length);
      break;
    case BUFF_MUSK_ID:// 机器人增益：0x0204。发送频率：状态改变后发送
      memcpy(&judge_rece_mesg.buff_musk, data_addr, data_length);
      break;
    case AERIAL_ROBOT_ENERGY_ID:// 空中机器人能量状态：0x0205。发送频率：10Hz
      memcpy(&judge_rece_mesg.aerial_robot_energy, data_addr, data_length);
      break;
    case ROBOT_HURT_ID://伤害状态：0x0206。发送频率：伤害发生后发送
      memcpy(&judge_rece_mesg.robot_hurt, data_addr, data_length);
      break;
    case SHOOT_DATA_ID://实时射击信息：0x0207。发送频率：射击后发送
    {
      memcpy(&judge_rece_mesg.shoot_data, data_addr, data_length);
			hero_42mm_shot_bullet_num++;
			pitch_data = pid_pit.get;
			yaw_data = pid_yaw.get;
			shot_speed_num++;
			if(shot_speed_num>=5)
				shot_speed_num=5;
			avg_shot_speed = avg(judge_rece_mesg.shoot_data.bullet_speed);
    }
    break;
		case RFID_STATES_ID://机器人 RFID 状态:0x0209。发射速率:1Hz,发射范围:单一机器人
      memcpy(&judge_rece_mesg.rfid_status, data_addr, data_length);
      break;
    case STUDENT_INTERACTIVE_HEADER_DATA_ID://交互数据接收信息：0x0301。发送频率：上限 10Hz
    {  
			memcpy(&judge_rece_mesg.student_interactive_header_data, data_addr, 6);
			if ( judge_rece_mesg.student_interactive_header_data.data_cmd_id == 0x203 ) // 接收到的信息ID
			{
					memcpy ( &judge_rece_mesg.robot_interactive_data, data_addr + 6, data_length - 6 );
					if ( ( judge_rece_mesg.game_robot_state.robot_id == judge_rece_mesg.student_interactive_header_data.receiver_ID )
									&& ( judge_rece_mesg.student_interactive_header_data.send_ID == 3
									||  judge_rece_mesg.student_interactive_header_data.send_ID == 103 ) )    //发送端机器人ID
					{
							robo_recive[0] = judge_rece_mesg.robot_interactive_data.data[0];
							robo_recive[1] = judge_rece_mesg.robot_interactive_data.data[1];
							robo_recive[2] = judge_rece_mesg.robot_interactive_data.data[2];
							robo_recive[3] = judge_rece_mesg.robot_interactive_data.data[3];
					}
			}
		}
      break;
    }
}

void unpack_fifo_data(unpack_data_t *p_obj, uint8_t sof)
{
  uint8_t byte = 0;
  while ( FIFO_S_CountUsed(p_obj->data_fifo) )
    {
      byte = FIFO_S_Get(p_obj->data_fifo);
      switch(p_obj->unpack_step)
        {
        case STEP_HEADER_SOF:
        {
          if(byte == sof)
            {
              p_obj->unpack_step = STEP_LENGTH_LOW;
              p_obj->protocol_packet[p_obj->index++] = byte;
            }
          else
            {
              p_obj->index = 0;
            }
        }
        break;
        case STEP_LENGTH_LOW:
        {
          p_obj->data_len = byte;
          p_obj->protocol_packet[p_obj->index++] = byte;
          p_obj->unpack_step = STEP_LENGTH_HIGH;
        }
        break;
        case STEP_LENGTH_HIGH:
        {
          p_obj->data_len |= (byte << 8);
          p_obj->protocol_packet[p_obj->index++] = byte;

          if(p_obj->data_len < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CRC_LEN))
            {
              p_obj->unpack_step = STEP_FRAME_SEQ;
            }
          else
            {
              p_obj->unpack_step = STEP_HEADER_SOF;
              p_obj->index = 0;
            }
        }
        break;
        case STEP_FRAME_SEQ:
        {
          p_obj->protocol_packet[p_obj->index++] = byte;
          p_obj->unpack_step = STEP_HEADER_CRC8;
        }
        break;
        case STEP_HEADER_CRC8:
        {
          p_obj->protocol_packet[p_obj->index++] = byte;

          if (p_obj->index == HEADER_LEN)
            {
              if ( Verify_CRC8_Check_Sum(p_obj->protocol_packet, HEADER_LEN) )
                {
                  p_obj->unpack_step = STEP_DATA_CRC16;
                }
              else
                {
                  p_obj->unpack_step = STEP_HEADER_SOF;
                  p_obj->index = 0;
                }
            }
        }
        break;
        case STEP_DATA_CRC16:
        {
          if (p_obj->index < (HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN))
            {
              p_obj->protocol_packet[p_obj->index++] = byte;
            }
          if (p_obj->index >= (HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN))
            {
              p_obj->unpack_step = STEP_HEADER_SOF;
              p_obj->index = 0;

              if ( Verify_CRC16_Check_Sum(p_obj->protocol_packet, HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN) )
                {
                  if (sof ==DN_REG_ID)
                    {
                      //pc_data_handle(p_obj->protocol_packet);
                      judgement_data_handle(p_obj->protocol_packet);
                    }
                  else  //DN_REG_ID
                    {
                    }
                }
            }
        } break;
        default:
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
        break;
        }
    }
}
void dma_buffer_to_unpack_buffer(uart_dma_rxdata_t *dma_obj, uart_it_type_e it_type)
{
  uint16_t tmp_len;
  uint8_t  current_memory_id;
  uint16_t remain_data_counter;

  uint8_t  *pdata        = dma_obj->buff[0];
  uint16_t write_counter = FIFO_S_CountFree(dma_obj->data_fifo);

  get_dma_memory_msg(DMA1_Stream0, &current_memory_id, &remain_data_counter);

  if (UART_IDLE_IT == it_type)
    {
      if (current_memory_id)
        {
          dma_obj->write_index = dma_obj->buff_size*2 - remain_data_counter;
        }
      else
        {
          dma_obj->write_index = dma_obj->buff_size - remain_data_counter;
        }
    }
  else if (UART_DMA_FULL_IT == it_type)
    {
      if (current_memory_id)
        {
          dma_obj->write_index = dma_obj->buff_size;
        }
      else
        {
          dma_obj->write_index = dma_obj->buff_size*2;
        }
    }

  tmp_len = dma_obj->write_index - dma_obj->read_index;
  write_counter = FIFO_S_Puts(dma_obj->data_fifo, &pdata[dma_obj->read_index], tmp_len);
  dma_obj->read_index = (dma_obj->read_index + write_counter) % (dma_obj->buff_size*2);
}

uint8_t* protocol_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf)
{
  uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;
  frame_header_t *p_header = (frame_header_t*)tx_buf;
  p_header->sof          = sof;
  p_header->data_length  = len;
  p_header->seq          = 0;

  memcpy(&tx_buf[HEADER_LEN], (uint8_t*)&cmd_id, CMD_LEN);
  Append_CRC8_Check_Sum(tx_buf,HEADER_LEN);
  memcpy(&tx_buf[HEADER_LEN + CMD_LEN], p_data, len);
  Append_CRC16_Check_Sum(tx_buf, frame_length);
  return tx_buf;
}

void data_upload_handle(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf)
{
  uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;

  protocol_packet_pack(cmd_id, p_data, len, sof, tx_buf);   //crc校验
  if (sof == UP_REG_ID)
    // write_uart_blocking(&COMPUTER_HUART, tx_buf, frame_length);
    {}
  else if (sof == DN_REG_ID)
    {
      DMA_ClearFlag(DMA1_Stream7,DMA_FLAG_TCIF7);//清除DMA1_Steam7传输完成标志
      USART_DMACmd(UART5,USART_DMAReq_Tx,ENABLE);  //使能串口5的DMA发送
      MYDMA_Enable(DMA1_Stream7,frame_length);
    }
}

void get_dma_memory_msg(DMA_Stream_TypeDef *dma_stream, uint8_t *mem_id, uint16_t *remain_cnt)
{
  *mem_id    = DMA_GetCurrentMemoryTarget(dma_stream);
  *remain_cnt = DMA_GetCurrDataCounter(dma_stream);
}
void communicate_param_init(void)
{
  /* create the judge_rxdata_mutex mutex  */
  /* judge data fifo init */
  FIFO_S_Init(&judge_rxdata_fifo,judge_rxdata_buf,JUDGE_FRAME_BUFLEN);
  FIFO_S_Init(&_UART5_RX_FIFO, (void *)_UART5_RX_BUF,  JUDGE_FRAME_BUFLEN);

  /* initial judge data dma receiver object */
  judge_rx_obj.data_fifo = &judge_rxdata_fifo;
  judge_rx_obj.buff_size = UART_RX_DMA_SIZE;
  judge_rx_obj.buff[0] = judge_dma_rxbuff[0];
  judge_rx_obj.buff[1] = judge_dma_rxbuff[1];

  /* initial judge data unpack object */
  judge_unpack_obj.data_fifo = &_UART5_RX_FIFO;
  judge_unpack_obj.p_header = (frame_header_t *)judge_unpack_obj.protocol_packet;
  judge_unpack_obj.index = 0;
  judge_unpack_obj.data_len = 0;
  judge_unpack_obj.unpack_step = STEP_HEADER_SOF;
}

void Usart5DmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)

{
//    DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输
//    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}  //确保DMA可以被设置
  DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量
  DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输
}

void Usart5SendBytesInfoProc(u8* pSendInfo, u16 nSendCount)
{
  u16 i = 0;
  u8 *pBuf = NULL;
  //指向发送缓冲区
  pBuf = USART5_DMA_TX_BUF;
  for (i=0; i<nSendCount; i++)
    {
      *(pBuf+i) = pSendInfo[i];
    }
  //DMA发送方式
  Usart5DmaSendDataProc(DMA1_Stream7,nSendCount); //开始一次DMA传输！
}


