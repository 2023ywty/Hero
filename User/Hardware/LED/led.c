#include "led.h"
#include "stm32f4xx.h"


//函数名称：led_configuration（）
//函数左右：初始化A板上的灯泡，用于错误检测
//入口参数：无
//返回  值：无
void led_configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//使能GPIOF GPIOE和GPIOG的时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOG, ENABLE); 
  //红绿灯
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	led_green_off();
	led_red_off();
  //8个错误检测灯
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	flow_led_off(GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8);
}


/******10个灯的控制函数******/
void led_green_off(void)
{
    GPIO_SetBits(GPIOF, GPIO_Pin_14);
}
void led_green_on(void)
{
    GPIO_ResetBits(GPIOF, GPIO_Pin_14);
}
void led_green_toggle(void)
{
    GPIO_ToggleBits(GPIOF, GPIO_Pin_14);
}

void led_red_off(void)
{
    GPIO_SetBits(GPIOE, GPIO_Pin_11);
}
void led_red_on(void)
{
    GPIO_ResetBits(GPIOE, GPIO_Pin_11);
}
void led_red_toggle(void)
{
    GPIO_ToggleBits(GPIOE, GPIO_Pin_11);
}

void flow_led_on(uint16_t num)
{
    GPIO_ResetBits(GPIOG, GPIO_Pin_8 >> num);
}
void flow_led_off(uint16_t num)
{
    GPIO_SetBits(GPIOG, GPIO_Pin_8 >> num);
}
void flow_led_toggle(uint16_t num)
{
    GPIO_ToggleBits(GPIOG, GPIO_Pin_8 >> num);
}


//函数名称：Electricpush_IO_Init（）
//函数左右：初始化电推杆所需的逻辑IO 
//入口参数：无
//返回  值：无
void Electricpush_IO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA, GPIO_Pin_5);
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	
	GPIO_ResetBits(GPIOC, GPIO_Pin_5);
	GPIO_SetBits(GPIOC, GPIO_Pin_1);
}


/*****电推杆控制函数*****/

void Electricpush_off(void)
{
	TIM_SetCompare3(TIM8, 200);
}

void Electricpush_on(void)
{
	TIM_SetCompare3(TIM8, 45);
}

void Electricpush2_off(void)
{
	GPIO_SetBits(GPIOC, GPIO_Pin_1);
	GPIO_ResetBits(GPIOC, GPIO_Pin_5);
}

void Electricpush2_on(void)
{
	GPIO_SetBits(GPIOC, GPIO_Pin_5);
	GPIO_ResetBits(GPIOC, GPIO_Pin_1);
}

