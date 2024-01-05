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
  //�ж��� 4
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	//��ʼ���δ�ʱ��
	delay_init();
	//���Ƹ˳�ʼ��
	Electricpush_IO_Init();
	//��ʼ��LED
	led_configuration();
	//SPI��IMU��IST��ʼ��
	Spi5_Init();
	MPU6500_Init();
	IST8310_Init();
	//����7��ʼ�� ����HI226
	uart7_init();
	//����3��ʼ��
	USART3_Init();
	//�ɿص�Դ��ʼ��
	power_ctrl_configuration();
  //�����ʼ��
  TIM8_Init(2000-1,1800-1);
	TIM_SetCompare4(TIM8, 125);  //45�߱�  180 �ͱ� 135��ʹ�ñ���
	TIM_SetCompare3(TIM8, 45);//90   220
  //����IO��ʼ��
  laser_configuration();
  //ң������ʼ��
  remote_control_init();
	//��ʱ��3��ʼ�������ڼ��CPU������
	TIM3_Init(5000, 1);
	//can1��ʼ��
	CAN1_Init();
  //�ⲿ�жϳ�ʼ��
	exti_init();
	TIM12_Init(9000,5);//1ms����һ���ж�
	//can2��ʼ��
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


