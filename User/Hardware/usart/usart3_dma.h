/**************************************************************************
 * @file     	usart3_dma.h
 * @brief    	串口三用于外部串口陀螺仪读取
 * @writer		宋立栋
 * @Q Q			2296054658
 **************************************************************************/
#ifndef _USART3_DMA_H
#define _USART3_DMA_H
#include "stm32f4xx.h"

void USART3_Init();	//串口3初始化
void USART3_Send_Data(u8 data);
void USART3_Send_Buf(u8 *DataToSend , u8 data_num);

	
#endif /*_USART3_DMA_H*/