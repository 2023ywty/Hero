/**************************************************************************
 * @file     	USART2.c
 * @brief    	串口二，用于调试，给上位机发送数据，接收上位机数据
 **************************************************************************/
 
#include "usart2.h"
#include "stm32f4xx.h"
#include "oscillography.h"
#include <stdio.h>
/**********************************中间变量、函数声明************************************************/
uint16_t  roll_p,roll_i,roll_d,pitch_p,pitch_i,pitch_d,yaw_p,yaw_i,yaw_d; 	//用于接收上位机发送来的数据，名字与上位机中的名字一一对应
u8 Rx_Ok = 0;			//接收上位机数据中间变量
static u8 Rx_Act=0;		//接收上位机数据中间变量，正在使用的buf号
static u8 Rx_Adr=0;		//接收上位机数据中间变量，正在接收第几字节
u8 Rx_Buf[2][64];		//接收上位机数据中间变量，两个32字节的串口接收缓存
u8  Send_Data_Buf[30];
//uint8_t  Send_Data_Buf[34];
void USART2_Send_Data(u8 data);	//通过串口2发送一个八位字符
/*********************************************************************************************/

/**
  * @name 	USART2_Init()
  * @brief 	串口2初始化
  * @param	None
  * @return None
  */
void USART2_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	//使能USART2_GPIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);	//使能USART2时钟
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2); //GPIOD_10复用为USART2
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2); //GPIOD_11复用为USART2
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//上拉
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; 	//TX RX
	GPIO_Init(GPIOD,&GPIO_InitStructure); 
	
	USART_InitStructure.USART_BaudRate = 115200;										//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;							//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;								//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;									//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;		//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;						//收发模式
	USART_Init(USART2, &USART_InitStructure);											//初始化串口2

	USART_Cmd(USART2, ENABLE);  //使能串口2
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	//开启相关中断
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;			//串口3中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;		//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;			//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);								//根据指定的参数初始化NVIC寄存器
}

/**
  * @name 	USART2_Init()
  * @brief 	串口2接收中断，用于接收上位机发送来的数据
  * @param	None
  * @return None
  */
void USART2_IRQHandler(void) 
{ 
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)                          
	{
		u8 com_data;
		com_data = USART2->DR;
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		if(Rx_Adr==0)		//寻找帧头0XAAAF
		{
			if(com_data==0x8A)	
			{
				Rx_Buf[Rx_Act][0] = com_data;
				Rx_Adr = 1;
			}
		}
		
		else if(Rx_Adr==1)
		{
			if(com_data==0x8B)	
			{
				Rx_Buf[Rx_Act][1] = com_data;
				Rx_Adr = 2;
			}
			else
				Rx_Adr = 0;
		}
		
		else if(Rx_Adr==2)
		{
			if(com_data==0x1C)	
			{
				Rx_Buf[Rx_Act][2] = com_data;
				Rx_Adr = 3;
			}
			else
				Rx_Adr = 0;
		}
		else if(Rx_Adr==3)
		{
			if(com_data==0xAE)	
			{
				Rx_Buf[Rx_Act][3] = com_data;
				Rx_Adr = 4;
			}
			else
				Rx_Adr = 0;
		}
		else
		{
			Rx_Buf[Rx_Act][Rx_Adr] = com_data;
			Rx_Adr ++;
		}
		if(Rx_Adr==32)		//数据接收完毕
		{
			Rx_Adr = 0;
			if(Rx_Act)	
			{ 
				Rx_Act = 0; 			//切换缓存
				Rx_Ok = 1;
			}
			else 				
			{
				Rx_Act = 1;
				Rx_Ok = 0;
			}
			roll_p=(uint16_t)Rx_Buf[Rx_Ok][4]<<8|(uint16_t)Rx_Buf[Rx_Ok][5];
			roll_i=(uint16_t)Rx_Buf[Rx_Ok][6]<<8|(uint16_t)Rx_Buf[Rx_Ok][7];
			roll_d=(uint16_t)Rx_Buf[Rx_Ok][8]<<8|(uint16_t)Rx_Buf[Rx_Ok][9];
			
			pitch_p=(uint16_t)Rx_Buf[Rx_Ok][10]<<8|(uint16_t)Rx_Buf[Rx_Ok][11];
			pitch_i=(uint16_t)Rx_Buf[Rx_Ok][12]<<8|(uint16_t)Rx_Buf[Rx_Ok][13];
			pitch_d=(uint16_t)Rx_Buf[Rx_Ok][14]<<8|(uint16_t)Rx_Buf[Rx_Ok][15];
			
			yaw_p=(uint16_t)Rx_Buf[Rx_Ok][16]<<8|(uint16_t)Rx_Buf[Rx_Ok][17];
			yaw_i=(uint16_t)Rx_Buf[Rx_Ok][18]<<8|(uint16_t)Rx_Buf[Rx_Ok][19];
			yaw_d=(uint16_t)Rx_Buf[Rx_Ok][20]<<8|(uint16_t)Rx_Buf[Rx_Ok][21];
			
			upComputer_dataRecieve(roll_p, roll_i, roll_d, pitch_p, pitch_i, pitch_d, yaw_p, yaw_i, yaw_d);
		}
	}
}

/**
  * @name 	USART2_Send_Buf()
  * @brief 	串口2发送一组数据
  * @param	DataToSend:发送的数据首指针 
  * @param	data_num:发送的数据数量
  * @return None
  */
void USART2_Send_Buf(u8 *DataToSend , u8 data_num)
{
	int data_i=0;
	for(data_i=0;data_i<data_num;data_i++)
		USART2_Send_Data(*(DataToSend+data_i));
}

/**
  * @name 	USART2_Send_Data()
  * @brief 	串口2发送一个字节
  * @param	发送的字节内容
  * @return None
  */
void USART2_Send_Data(u8 data)
{
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET){}; 
		USART2->DR=(data & (uint16_t)0x01FF);
}

void Send_data8(u8 *dataddr,u8 len,u8 func_word)    //常用 
{ 
	/*更改了匿名上位机版本型号使用这个更方便*/
	u8 i,count;
	u8 now_word;
	u8 sc,ac;
	ac=sc=0;
	if(len>20) len=20;
	if((func_word>=1)&&(func_word<=10))
		now_word=0xF0|func_word;
	Send_Data_Buf[0]=0xAA;
	Send_Data_Buf[1]=0xFF;
	Send_Data_Buf[2]=now_word;
	Send_Data_Buf[3]=len;
	for(i=0;i<len;i++) 
		Send_Data_Buf[4+i] = *(dataddr+i);       //发送数据   
	for(i=0;i<Send_Data_Buf[3]+4;i++)
	{
		sc+=Send_Data_Buf[i];
		ac+=sc;
	}
	Send_Data_Buf[4+len]=sc;
	Send_Data_Buf[5+len]=ac;
	count=6+len;
//	u8 i;
//	if(len>28) len=28;
//	switch(func_word)
//	{
//		case 1:func_word=0xA1;break;
//		case 2:func_word=0xA2;break;
//		case 3:func_word=0xA3;break;
//		case 4:func_word=0xA4;break;
//	}
//	Send_Data_Buf[0] = 0x88;//以前是0x88
//	Send_Data_Buf[1] = func_word;                //发送fun
//	Send_Data_Buf[2] = len;                   //发送len
//	for(i=0;i<len;i++) Send_Data_Buf[3+i] = *(dataddr+i);       //发送数据   
//	Send_Data_Buf[3+len]=0;
//	for(i=0;i<len+3;i++) Send_Data_Buf[3+len]+=  Send_Data_Buf[i];
 	USART2_Send_Buf(Send_Data_Buf,count);
}

///重定向c库函数printf到串口DEBUG_USART，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到串口DEBUG_USART */
		USART_SendData(USART2, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

///重定向c库函数scanf到串口DEBUG_USART，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		/* 等待串口输入数据 */
		while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(USART2);
}
/*********************************************END OF FILE**********************/

