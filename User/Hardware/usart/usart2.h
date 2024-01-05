/**************************************************************************
 * @file     	USART2.h
 * @brief    	串口二，用于调试，给上位机发送数据，接收上位机数据
 * @writer		宋立栋
 * @Q Q			2296054658
 **************************************************************************/
#ifndef _USART2_H
#define _USART2_H
#include "stm32f4xx.h"

void USART2_Init();	//串口二初始化函数
void USART2_Send_Buf(u8 *DataToSend , u8 data_num);	//通过串口2发送一组数据，用于上位机显示
void Send_data8(u8 *dataddr,u8 len,u8 func_word);
void USART2_Send_Data(u8 data);
#endif
