/**************************************************************************
 * @file     	USART2.h
 * @brief    	���ڶ������ڵ��ԣ�����λ���������ݣ�������λ������
 * @writer		������
 * @Q Q			2296054658
 **************************************************************************/
#ifndef _USART2_H
#define _USART2_H
#include "stm32f4xx.h"

void USART2_Init();	//���ڶ���ʼ������
void USART2_Send_Buf(u8 *DataToSend , u8 data_num);	//ͨ������2����һ�����ݣ�������λ����ʾ
void Send_data8(u8 *dataddr,u8 len,u8 func_word);
void USART2_Send_Data(u8 data);
#endif
