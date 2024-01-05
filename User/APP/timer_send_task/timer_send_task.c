#include "timer_send_task.h"
#include "stm32f4xx.h"
#include "can.h"
#include "usart6.h"
#include "gimbal_task.h"
#include "BoardB_link.h"
#include "shoot_task.h"

TimerHandle_t	CAN1_Timer_Handle; 			//周期定时器句柄					
TimerHandle_t	CAN2_Timer_Handle; 			//周期定时器句柄
TimerHandle_t	VISION_Timer_Handle; 		//周期定时器句柄

QueueHandle_t	CAN1_Queue;					//CAN1消息队列句柄
QueueHandle_t	CAN2_Queue;					//CAN2消息队列句柄
QueueHandle_t	VISION_Send_Queue;	//视觉消息队列句柄



/**
  * @brief  软件定时回调函数
  * @param  void
  * @retval void
  * @attention can队列禁用等待，禁用delay
  */
void CAN1_Timer_Callback( TimerHandle_t xTimer )
{
	CanTxMsg SendCanTxMsg;
	while(xQueueReceive(CAN1_Queue, &SendCanTxMsg, 0))//接收队列信息,禁止等待！！！
	{
		CAN_Transmit(CAN1, &SendCanTxMsg);//发送目标值
  }
}


/**
  * @brief  软件定时回调函数
  * @param  void
  * @retval void
  * @attention can队列禁用等待，禁用delay
  */
void CAN2_Timer_Callback( TimerHandle_t xTimer )
{
	CanTxMsg    SendCanTxMsg;

	while(xQueueReceive(CAN2_Queue, &SendCanTxMsg, 0))//接收队列信息,禁止等待！！！
	{	
		CAN_Transmit(CAN2, &SendCanTxMsg);//发送目标值
    }
}




/**
  * @brief  软件定时回调函数
  * @param  void
  * @retval void
  * @attention can队列禁用等待，禁用delay
  */
void Vision_Timer_Callback( TimerHandle_t xTimer )
{ 
	VisionSendData_t VisionSendData_Struct;
	VisionSendData_t VisionSendData_Struct_one;
	controlSysMsg_t  controlSysMsg_Struct;

/*****************测试部分***************/	
//	while(xQueueReceive(VISION_Send_Queue, &VisionSendData_Struct, 0))//接收队列信息,禁止等待！！！
//  {
//		//	Vision_Info_Send(&VisionSendData_Struct);//发送目标值
//  }
//	  for(int i=0;i<29;i++){
//    USART6_Send_Data(0XA1);}
//	  for(int i=0;i<29;i++){
//    USART6_Send_Data(0XB1);}
//	  for(int i=0;i<29;i++){
//    USART6_Send_Data(0XC1);}
	
	
   Vision_Rm2021_Cm0x0001_Send_Queue(9000,2,0);
//  	Vision_Rm2021_Cm0x0001_Send_Queue(10000, 2, 0);


//  for(int i=0;i<5;i++)
//	{Vision_Rm2021_Cm0x0001_Send_Queue(8, i, 0);}
//	if(gimbalTaskStructure.nowBehaviorName==AUTO)
//		{//云台处于自瞄模式
//			Vision_Rm2021_Cm0x0001_Send_Queue(BoardBLink.RefreeSyetem.shoot.s16_Shoot_speed, 1, 1);
	  	
	
//		if(gimbalTaskStructure.Auto_Mode.flag.if_rotate==TRUE/*打陀螺*/)
//		{			//双击自瞄打陀螺
//		   Vision_Rm2021_Cm0x0001_Send_Queue(BoardBLink.RefreeSyetem.shoot.s16_Shoot_speed, 0, 1);
//		}
//   else
//	  {
//	   
//			
//	  }
			
    
//	else if(gimbalTaskStructure.nowBehaviorName==SMALL_BUFF){//云台小幅模式
//			

//	}

		
		
//	else{//不请求，关闭视觉
//				
//      }
    
		
		
}









void timerSendCreate(void){
	
	taskENTER_CRITICAL(); 	//进入临界区   ？？？？？？？？？？？？？？？
	
	/*--------------------------数据接收--------------------------*/	

	//创建can1接收队列
	//can1接收到的报文存放在此队列中
	CAN1_Queue = xQueueCreate( 128, sizeof(CanTxMsg));//最多可保持64个CanTxMsg
	
	//创建can2接收队列
	//can2接收到的报文存放在此队列中
	CAN2_Queue = xQueueCreate( 128, sizeof(CanTxMsg));//
	
	//创建视觉发送队列
	//视觉发送到的报文存放在此队列中
	VISION_Send_Queue = xQueueCreate( 128, sizeof(VisionSendData_t));//
	
	//创建can1发送定时器
	 CAN1_Timer_Handle=xTimerCreate((const char*	)"CAN1_Timer",
									 (TickType_t 	)TIME_STAMP_1MS,//2ms
									 (UBaseType_t	)pdTRUE, 		//周期执行
									 (void *		)0,				//编号一般给0
									 (TimerCallbackFunction_t)CAN1_Timer_Callback);//回调函数
	
	//开启CAN1定时器,仅且仅能开启一次，否则会出错，不开启则不会发数据
	if( CAN1_Timer_Handle != NULL )
	{
		xTimerStart(CAN1_Timer_Handle,0);//不等待
	}
									 
	//创建can2发送定时器
	 CAN2_Timer_Handle=xTimerCreate((const char*	)"CAN2_Timer",
									 (TickType_t 	)TIME_STAMP_1MS,//1ms
									 (UBaseType_t	)pdTRUE, 		//周期执行
									 (void *		)1,				//编号一般给0
									 (TimerCallbackFunction_t)CAN2_Timer_Callback);//回调函数
										 
	//开启CAN2定时器,仅且仅能开启一次，否则会出错，不开启则不会发数据
	if( CAN2_Timer_Handle != NULL )
	{
		xTimerStart(CAN2_Timer_Handle,0);//不等待
	}	
					 
	
	//创建视觉接受定时器    
	 VISION_Timer_Handle=xTimerCreate((const char*	)"VISION_Timer",
									 (TickType_t 	)TIME_STAMP_10MS,//5ms
									 (UBaseType_t	)pdTRUE, 		//周期执行
									 (void *		)1,				//编号一般给0
									 (TimerCallbackFunction_t)Vision_Timer_Callback);//回调函数
										 
	//开启视觉定时器,仅且仅能开启一次，否则会出错，不开启则不会发数据
	if( VISION_Timer_Handle != NULL )
	{
		xTimerStart(VISION_Timer_Handle,0);//不等待
	}	
	
	
	taskEXIT_CRITICAL();	//退出临界区
}
