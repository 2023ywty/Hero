/**************************************************************************
 * @file     	usart3_dma.h
 * @brief    	串口三用于外部串口陀螺仪发送
 **************************************************************************/
#include "usart3_dma.h"
#include "stm32f4xx.h"
#include "gimbal_task.h"
#include "oscillography.h"
#include "Detect_Task.h"
/*****************************中间函数、变量声明****************************/
unsigned char visionData[25];	//DMA传送内存地址
u16 visionDataLength = 25;
s16 usart3IRQIn_sum = 0;
#define GET_AMOR_DATA_END1	0x7E		//自瞄数据帧倒数第二帧尾
#define GET_AMOR_DATA_END2	0x7D		//自瞄数据帧倒数第一帧尾

/*****************************88888888888888888****************************/

/**
  * @name 	USART3_Init()
  * @brief 	串口3初始化
  * @param	None
  * @return None
  */
void USART3_Init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8, GPIO_AF_USART3);

	GPIO_InitTypeDef gpio;
	USART_InitTypeDef usart3;
	gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &gpio);	
	
	USART_DeInit(USART3);
	usart3.USART_BaudRate = 115200;
	usart3.USART_WordLength = USART_WordLength_8b;
	usart3.USART_StopBits = USART_StopBits_1;
	usart3.USART_Parity = USART_Parity_No;
	usart3.USART_Mode = USART_Mode_Rx| USART_Mode_Tx;
	usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART3,&usart3);
	USART_Cmd(USART3,ENABLE);
	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
	
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = USART3_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 11;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
	
	DMA_InitTypeDef dma;
	DMA_DeInit(DMA1_Stream1);
	dma.DMA_Channel = DMA_Channel_4;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
	dma.DMA_Memory0BaseAddr = (uint32_t)visionData;
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dma.DMA_BufferSize = visionDataLength;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_Mode = DMA_Mode_Circular;
	dma.DMA_Priority = DMA_Priority_VeryHigh;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	dma.DMA_MemoryBurst = DMA_Mode_Normal;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream1,&dma);
	DMA_Cmd(DMA1_Stream1,ENABLE);
}


void USART3_IRQHandler()
{
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
	{
		u8 UARTtemp;
		UARTtemp = USART3->SR;
		UARTtemp = USART3->DR;
		DMA_Cmd(DMA1_Stream1, DISABLE);
		
		if(visionData[8] == GET_AMOR_DATA_END2 && visionData[7] == GET_AMOR_DATA_END1)
		{
			static s16 last_yaw = 0;
			static s16 last_pit = 0;
			u8 sum = 0;
			for(int i = 0; i < 6; i++)
				sum += visionData[i];
			if(sum == visionData[6])
			{
				DetectHook(DETECT_VISION);
				u8 pitch_err[3];
				u8 yaw_err[3];
				for(int i = 0; i < 3; i++)
				{
					yaw_err[i] = visionData[i];
					pitch_err[i] = visionData[i + 3];
				}
				
				s16 temp_yaw_err = yaw_err[1];
				s16 temp_pitch_err = pitch_err[1];
				s16 s_yawErr;
				s16 s_pitErr;
				s_yawErr = (temp_yaw_err*100) + yaw_err[2];
				s_pitErr = (temp_pitch_err*100) + pitch_err[2];
				if(!yaw_err[0])
				{
					s_yawErr *= -1;
				}

				if(!pitch_err[0])
				{
					s_pitErr *= -1;
				}
				oscillographyData[6] = s_yawErr;
				oscillographyData[7] = s_pitErr;
			}
		}
		USART_ClearITPendingBit(USART3, USART_IT_IDLE); 
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
		while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);
		DMA_SetCurrDataCounter(DMA1_Stream1, visionDataLength);
		DMA_Cmd(DMA1_Stream1, ENABLE);
	}
}
/**
  * @name 	USART3_Send_Data()

  * @brief 	串口3发送一个字节
  * @param	发送的字节内容
  * @return None
  */
void USART3_Send_Data(u8 data)
{
	while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET){}; 
		USART3->DR=(data & (uint16_t)0x01FF);
}

/**
  * @name 	USART3_Send_Buf()
  * @brief 	串口3发送一组数据
  * @param	DataToSend:发送的数据首指针 
  * @param	data_num:发送的数据数量
  * @return None
  */
void USART3_Send_Buf(u8 *DataToSend , u8 data_num)
{
	int data_i=0;
	for(data_i=0;data_i<data_num;data_i++)
		USART3_Send_Data(*(DataToSend+data_i));
}
