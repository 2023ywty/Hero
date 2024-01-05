#include "stm32f4xx.h"
#include "led.h"
#include "delay.h"
#include "robotConfig.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "power_ctrl.h"
#include "buzzer.h"
#include "laser.h"
#include "remote_control.h"
#include "timer.h"
#include "start_task.h"
#include "usart2.h"
#include "timer_send_task.h"
#include "main.h"
#include "can.h"
#include "robot.h"
#include "usart3_dma.h"
#include "usart6.h"
#include "oscillography.h"
#include "exti.h"
#include "spi5.h"
#include "IMU_onBoard.h"
#include "uart7.h"


void BSP_Init()
{
  //中断组 4
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	//初始化滴答时钟
	delay_init();
	//电推杆初始化
	Electricpush_IO_Init();
	//初始化LED
	led_configuration();
	//SPI、IMU、IST初始化
	Spi5_Init();
	MPU6500_Init();
	IST8310_Init();
	//串口7初始化 连接HI226
	uart7_init();
	//串口3初始化
	USART3_Init();
	//可控电源初始化
	power_ctrl_configuration();
  //舵机初始化
  TIM8_Init(2000-1,1800-1);
	TIM_SetCompare4(TIM8, 125);  //45高倍  180 低倍 135不使用倍镜
	TIM_SetCompare3(TIM8, 45);//90   220
  //激光IO初始化
  laser_configuration();
  //遥控器初始化
  remote_control_init();
	//定时器3初始化，用于检测CPU利用率
	TIM3_Init(5000, 1);
	//can1初始化
	CAN1_Init();
  //外部中断初始化
	exti_init();
	TIM12_Init(9000,5);//1ms触发一次中断
	//can2初始化
	CAN2_Init();
	USART6_Init();
}


int main()
{
	BSP_Init();
	delay_ms(100);
	robotInit();
	timerSendCreate();
	startTask();  
	vTaskStartScheduler();
	while(1)
	{

	}
}


