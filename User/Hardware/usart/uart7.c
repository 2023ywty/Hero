#include "uart7.h"
#include "INS_task.h"
#include "IMU_ext.h"
#include "Detect_Task.h"

static uint8_t decode_succ; 
extern raw_t rawa;


void uart7_init(void)
{
	//时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE , ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource7, GPIO_AF_UART7);	
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource8, GPIO_AF_UART7);
	
	GPIO_InitTypeDef GPIO_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef UART7_InitStruct;
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7| GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	UART7_InitStruct.USART_BaudRate = 115200;
	UART7_InitStruct.USART_WordLength = USART_WordLength_8b;
	UART7_InitStruct.USART_StopBits = USART_StopBits_1;
	UART7_InitStruct.USART_Parity = USART_Parity_No;
	UART7_InitStruct.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	UART7_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(UART7,&UART7_InitStruct);
	USART_Cmd(UART7,ENABLE);
	
	//NVIC配置
	NVIC_InitStructure.NVIC_IRQChannel						=	UART7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	3;
	NVIC_Init(&NVIC_InitStructure);
	USART_ITConfig(UART7, USART_IT_RXNE, ENABLE);
}


void UART7_IRQHandler(void)
{
	uint8_t ch;
	if(USART_GetITStatus(UART7, USART_IT_RXNE) != RESET)
    {
        ch = USART_ReceiveData(UART7);	
			
    }
    decode_succ=  ch_serial_input(&rawa, ch);//接收原始数据
} 

