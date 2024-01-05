 
 #include "start_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "test_task.h"
#include "oscillography.h"
#include "INS_task.h"
#include "gimbal_task.h"
#include "detect_task.h"
#include "chassis_task.h"
#include "shoot_task.h"
#include "BoardB_link.h"

#define START_TASK_PRIO 1
#define START_STK_SIZE 512
static TaskHandle_t StartTask_Handler;

#define LED_TASK_PRIO 4
#define LED_STK_SIZE 128
static TaskHandle_t LEDTask_Handler;

#define BUZZER_TASK_PRIO 4
#define BUZZER_STK_SIZE 128
static TaskHandle_t BuzzerTask_Handler;

#define INS_TASK_PRIO 21
#define INS_STK_SIZE 512
static TaskHandle_t INSTask_Handler;

#define GIMBAL_TASK_PRIO 19
#define GIMBAL_STK_SIZE 512
static TaskHandle_t GIMBALTask_Handler;

#define CHASSIS_TASK_PRIO 18
#define CHASSIS_STK_SIZE 512
static TaskHandle_t CHASSISTask_Handler;

#define SHOOT_TASK_PRIO 20
#define SHOOT_STK_SIZE 512
static TaskHandle_t SHOOTTask_Handler;

#define DECT_TASK_PRIO 10
#define DECT_STK_SIZE 256
static TaskHandle_t DECTTask_Handler;

#define OSCILLOGRAPHY_TASK_PRIO 5
#define OSCILLOGRAPHY_STK_SIZE 256
static TaskHandle_t Oscillography_Handler;

#define BOARDBLINK_TASK_PRIO 3
#define BOARDBLINK_STK_SIZE 256
static TaskHandle_t BOARDBLINKTask_Handler;



void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();

    xTaskCreate((TaskFunction_t)INS_task,          //任务函数
                (const char *)"INS_task",          //任务名称
                (uint16_t)INS_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)INS_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&BuzzerTask_Handler); //任务句柄


    xTaskCreate((TaskFunction_t)gimbalTask,          //任务函数
                (const char *)"gimbalTask",          //任务名称
                (uint16_t)GIMBAL_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        		//传递给任务函数的参数
                (UBaseType_t)GIMBAL_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&GIMBALTask_Handler); //任务句柄
 
								
		xTaskCreate((TaskFunction_t)chassisTask,          //任务函数
								(const char *)"chassisTask",          //任务名称
								(uint16_t)CHASSIS_STK_SIZE,            //任务堆栈大小
								(void *)NULL,                        		//传递给任务函数的参数
								(UBaseType_t)CHASSIS_TASK_PRIO,        //任务优先级
								(TaskHandle_t *)&CHASSISTask_Handler); //任务句柄 
//								

    xTaskCreate((TaskFunction_t)shootTask,          //任务函数
                (const char *)"shootTask",          //任务名称
                (uint16_t)SHOOT_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        		//传递给任务函数的参数
                (UBaseType_t)SHOOT_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&SHOOTTask_Handler); //任务句柄	
								

    xTaskCreate((TaskFunction_t)DetectTask,          //任务函数
                (const char *)"DetectTask",          //任务名称
                (uint16_t)DECT_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)DECT_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&DECTTask_Handler); //任务句柄
								

    xTaskCreate((TaskFunction_t)oscillography_task,          //任务函数
                (const char *)"oscillography_task",          //任务名称
                (uint16_t)OSCILLOGRAPHY_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        		//传递给任务函数的参数
                (UBaseType_t)OSCILLOGRAPHY_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&Oscillography_Handler); //任务句柄

	  xTaskCreate((TaskFunction_t)led_task,          //任务函数
				        (const char *)"led_task",          //任务名称
				        (uint16_t)LED_STK_SIZE,            //任务堆栈大小
				        (void *)NULL,                        //传递给任务函数的参数
				        (UBaseType_t)LED_TASK_PRIO,        //任务优先级
				        (TaskHandle_t *)&LEDTask_Handler); //任务句柄

				
    xTaskCreate((TaskFunction_t)buzzer_task,          //任务函数
                (const char *)"buzzer_task",          //任务名称
                (uint16_t)BUZZER_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)BUZZER_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&BuzzerTask_Handler); //任务句柄
						
								
		 xTaskCreate((TaskFunction_t)boardB_link_Task,         //任务函数
				        (const char *)"boardB_link_Task",         //任务名称
				        (uint16_t)BOARDBLINK_STK_SIZE,            //任务堆栈大小
				        (void *)NULL,                        	  //传递给任务函数的参数
				        (UBaseType_t)BOARDBLINK_TASK_PRIO,        //任务优先级
			       	  (TaskHandle_t *)&BOARDBLINKTask_Handler); //任务句柄	

    vTaskDelete(StartTask_Handler); //删除开始任务

    taskEXIT_CRITICAL();            //退出临界区
}

void startTask(void)
{
    xTaskCreate((TaskFunction_t)start_task,          //任务函数
                (const char *)"start_task",          //任务名称
                (uint16_t)START_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)START_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&StartTask_Handler); //任务句柄
}