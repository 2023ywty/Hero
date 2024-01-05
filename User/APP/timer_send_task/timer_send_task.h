#ifndef TIME_SEND_TASK_H
#define TIME_SEND_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

void timerSendCreate(void);
void CAN1_Timer_Callback( TimerHandle_t xTimer );
void CAN2_Timer_Callback( TimerHandle_t xTimer );

extern QueueHandle_t	CAN1_Queue;					//CAN1消息队列句柄
extern QueueHandle_t	CAN2_Queue;					//CAN2消息队列句柄
extern QueueHandle_t	VISION_Send_Queue;	//视觉消息队列句柄

#define    TIME_STAMP_1MS        1
#define    TIME_STAMP_2MS        2
#define    TIME_STAMP_4MS        4
#define    TIME_STAMP_5MS        5
#define    TIME_STAMP_10MS      10
#define    TIME_STAMP_20MS      20
#define    TIME_STAMP_30MS      30
#define    TIME_STAMP_40MS      40
#define    TIME_STAMP_50MS      50
#define    TIME_STAMP_60MS      60
#define    TIME_STAMP_80MS      80
#define    TIME_STAMP_100MS    100
#define    TIME_STAMP_150MS    150
#define    TIME_STAMP_200MS    200
#define    TIME_STAMP_250MS    250
#define    TIME_STAMP_300MS    300
#define    TIME_STAMP_400MS    400
#define    TIME_STAMP_500MS    500
#define    TIME_STAMP_1000MS  1000
#define    TIME_STAMP_2000MS  2000
#define    TIME_STAMP_10S     10000

#endif