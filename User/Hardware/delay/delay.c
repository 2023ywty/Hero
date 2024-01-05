#include "delay.h"
#include "stm32f4xx.h"
#include "core_cm4.h"

void delay_init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //APB1总线时钟外设频率90Mhz

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM6, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM6, DISABLE);

    TIM_DeInit(TIM6);
    TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF; //单次最大延时值65535us
    TIM_TimeBaseInitStructure.TIM_Prescaler = 89; //90Mhz -> 1Mhz
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向下计数模式
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //时钟不分割（输入捕获滤波用）
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStructure);
    
    TIM_ClearFlag(TIM6, TIM_FLAG_Update); //清除更新标志位
    
    uint32_t reload = 0;
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);

    reload = SystemCoreClock / 1000 / 8;
    reload--;

    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    SysTick->LOAD = reload;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}


void delay_us(uint16_t us)
{
	//存在指令的大约0.5us延时
    TIM_SetCounter(TIM6, 0xFFFF - us + 1); //设置定时值
	TIM_Cmd(TIM6, ENABLE); //使能定时器
    while(TIM_GetFlagStatus(TIM6, TIM_FLAG_Update) != SET);
	TIM_ClearFlag(TIM6, TIM_FLAG_Update); //清除更新标志位
	TIM_Cmd(TIM6, DISABLE); //失能定时器
}


void delay_ms(uint16_t ms){
    TIM_SetCounter(TIM6, 0xFFFF - 1000 + 1); //设置定时值1ms
	TIM_Cmd(TIM6, ENABLE); //使能定时器
	for(uint16_t i=0; i<ms; i++){ //循环nms
        while(TIM_GetFlagStatus(TIM6, TIM_FLAG_Update) != SET);
		TIM_SetCounter(TIM6, 0xFFFF - 1000 + 1); //设置定时值1ms
		TIM_ClearFlag(TIM6, TIM_FLAG_Update); //清除更新标志位
	}
	TIM_Cmd(TIM6, DISABLE); //失能定时器
}


void smartDelayMs(uint16_t ms){
    if(xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
        delay_ms(ms);
    else
        vTaskDelay(ms);
}
