#include "can.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "timer_send_task.h"
#include "motor.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "Detect_Task.h"
#include "shoot_task.h"
#include "BoardB_link.h"
/**
  * @name	CAN1_Init()
  * @brief 	CAN1初始化函数
	
  * @param  None
  */
void CAN1_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    //使能相关时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能PORTA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟

    //初始化GPIO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1| GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化PA11,PA12

    //引脚复用映射配置
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_CAN1); //GPIOA11复用为CAN1
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_CAN1); //GPIOA12复用为CAN1


    //CAN单元设置
    CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式
    CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理
    CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
    CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送
    CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的
    CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定
    CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;	 //模式设置
    CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
    CAN_InitStructure.CAN_BS1=CAN_BS1_8tq; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
    CAN_InitStructure.CAN_BS2=CAN_BS2_6tq;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
    CAN_InitStructure.CAN_Prescaler=3;  //分频系数(Fdiv)为brp+1
    CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1

    //配置过滤器
    CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
    CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化

    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.

    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;     // 主优先级为1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @name	CAN2_Init()
  * @brief 	CAN2初始化函数
  * @param  None
  */
void CAN2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    //使能相关时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//使能CAN2时钟

    //初始化GPIO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12| GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PA11,PA12

    //引脚复用映射配置
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2); //GPIOB12复用为CAN2
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2); //GPIOB13复用为CAN2


    //CAN单元设置
    CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式
    CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理
    CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
    CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送
    CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的
    CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定
    CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;	 //模式设置
    CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
    CAN_InitStructure.CAN_BS1=CAN_BS1_8tq; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
    CAN_InitStructure.CAN_BS2=CAN_BS2_6tq;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
    CAN_InitStructure.CAN_Prescaler=3;  //分频系数(Fdiv)为brp+1
    CAN_Init(CAN2, &CAN_InitStructure);   // 初始化CAN2

    //配置过滤器
    CAN_FilterInitStructure.CAN_FilterNumber=27;	  //过滤器0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
    CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化

    CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.
//    CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);	//开启can2中断

    NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;     // 主优先级为1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}




/**
  * @name	CAN1_RX0_IRQHandler()
  * @brief 	CAN1接受中断函数,用于接收地盘电机 云台电机 拨弹电机“转子”速度位置反馈
  * @param  None
  */
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg rx_message;	
    if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
  	{
    CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
	  CAN_ClearFlag(CAN1, CAN_FLAG_FF0); 		
		CAN_Receive(CAN1, CAN_FIFO0, &rx_message);//读取数据	
		switch(rx_message.StdId)
		{//can1地盘四个、拨盘电机、Yaw轴一个、弹舱
			//			case 0x1000:
			{
				DetectHook(DETECT_BOARDB_LINK);
				break;
			}	
//=========================================================================
			case 0x102:
			{
				getMessage_102(&rx_message);
				DetectHook(DETECT_JUDGE_SYSTEM);
				break;
			}
			
			case 0x103:
			{
				getMessage_103(&rx_message);
				DetectHook(DETECT_JUDGE_SYSTEM);
				break;
			}
			
			case 0x104:
			{
			  getMessage_104(&rx_message);
				DetectHook(DETECT_JUDGE_SYSTEM);
				break;
			}
					
			case 0x201:
			{
				chassisMotorDataRecieve(&rx_message, &(chassisTaskStructure.motor[CHASSIS_CM1].baseInf));
				DetectHook(DETECT_CHASSIS_CM1_MOTOR);
				break;	
			}
			case 0x202:
			{
				chassisMotorDataRecieve(&rx_message, &(chassisTaskStructure.motor[CHASSIS_CM2].baseInf));
				DetectHook(DETECT_CHASSIS_CM2_MOTOR);
				break;	
			}
			case 0x203:
			{
				chassisMotorDataRecieve(&rx_message, &(chassisTaskStructure.motor[CHASSIS_CM3].baseInf));
				DetectHook(DETECT_CHASSIS_CM3_MOTOR);
				break;	
			}
			case 0x204:
			{
				chassisMotorDataRecieve(&rx_message, &(chassisTaskStructure.motor[CHASSIS_CM4].baseInf));
      	DetectHook(DETECT_CHASSIS_CM4_MOTOR);
				break;	
			}
			case 0x205:
			{
				gimbalMotorDataRecieve(&rx_message, &(gimbalTaskStructure.yawMotor.base_inf));
				getMotorSpeedByPosition(&gimbalYawMotorSpeedCalc, gimbalTaskStructure.yawMotor.base_inf.real_ecd, xTaskGetTickCount());
				DetectHook(YAW_MOTOR);
				break;
			}
			case 0x206:
			{
				reductMotorDataRecieve(&rx_message, &(shootTaskStructure.plateMotor.base_inf));
				DetectHook(DETECT_SHOOT_PLATE_MOTOR);
				break;
			}
			case 0x207:
			{
         chassisMotorDataRecieve(&rx_message, &(shootTaskStructure.leftFirMotor.base_inf));
         DetectHook(DETECT_SHOOT_LF_MOTOR);
         break;
				break;
			}
			case 0x208:
			{
         chassisMotorDataRecieve(&rx_message, &(shootTaskStructure.rightFirMotor.base_inf));
         DetectHook(DETECT_SHOOT_RF_MOTOR);
         break;
			}
			default:
				break;
		}
    }
}


/**
  * @name	CAN2_RX0_IRQHandler()
  * @brief 	CAN2接受中断，用于接收B板发送来的信息及CAN2总线上连接的电机“转子”的速度位置信息
  * @param  None
  * @return None
  */
void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;
    if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FF0);
				CAN_ClearFlag(CAN2, CAN_FLAG_FF0); 	
        CAN_Receive(CAN2, CAN_FIFO0, &rx_message);
        switch(rx_message.StdId)
        {

        case 0x206:
        {
            gimbalMotorDataRecieve(&rx_message, &(gimbalTaskStructure.pitchMotor.base_inf));
            getMotorSpeedByPosition(&gimbalPitMotorSpeedCalc, gimbalTaskStructure.pitchMotor.base_inf.real_ecd, xTaskGetTickCount());
            DetectHook(PITCH_MOTOR);
            break;
        }
        default:
            break;
        }
    }
}

void djiMotorCurrentSendQueue(CAN_TypeDef* CANx, uint32_t stdId, s16 *current, u8 len) {
    if(len > 4)
        len = 4;
    if(len < 0)
        len = 0;

    CanTxMsg can_msg;

    can_msg.StdId=stdId;
    can_msg.IDE=CAN_ID_STD;
    can_msg.RTR=CAN_RTR_DATA;	// 消息类型为数据帧，一帧8位
    can_msg.DLC=8;				// 发送8帧信息

    u8 i = 0;
    for(; i < len; i++)
    {
        can_msg.Data[2*i] = (u8)((int16_t)current[i]>>8);
        can_msg.Data[2*i+1]=(u8)((int16_t)current[i]);
    }

    if(CANx == CAN1)
    {
        xQueueSend(CAN1_Queue,&can_msg,1);//向队列中填充内容
    }
    else if(CANx == CAN2)
    {
        xQueueSend(CAN2_Queue,&can_msg,1);//向队列中填充内容
    }
}


/**
  * @name	CAN1_SendBoardB()
  * @brief 	向B板发送请求信息
  * @param  requestMsg:请求内容
  * @return None
  */
void CAN1_SendRequestMsg(u8 requestMsg)
{
    CanTxMsg TxMessage;    
    TxMessage.StdId = 0x0100;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 0x02;  
    TxMessage.Data[0] = (uint8_t)(requestMsg);
    CAN_Transmit(CAN1,&TxMessage);
}

/**
  * @name	CAN1_SendBoardB()
  * @brief 	向B板发送云台陀螺仪数据
  * @return None
  */
void CAN1_SendBoardB(float yawAngle, float yawSpeed, float pitchAngle, float pitchSpeed)
{
    CanTxMsg TxMessage;    
    TxMessage.StdId = 0x0001;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 0x08;  
	s16 YawAngle = yawAngle*10;
	s16 YawSpeed = yawSpeed*10;
	s16 PitchAngle = pitchAngle*10;
	s16 PitchSpeed = pitchSpeed*10;
   TxMessage.Data[0] = (uint8_t) YawAngle;
	TxMessage.Data[1] = (uint8_t)(YawAngle>>8);
    TxMessage.Data[2] = (uint8_t) YawSpeed;
	TxMessage.Data[3] = (uint8_t)(YawSpeed>>8);
    TxMessage.Data[4] = (uint8_t) PitchAngle;
	TxMessage.Data[5] = (uint8_t)(PitchAngle>>8);
    TxMessage.Data[6] = (uint8_t) PitchSpeed;
	TxMessage.Data[7] = (uint8_t)(PitchSpeed>>8);
	xQueueSend(CAN1_Queue,&TxMessage,1);//向队列中填充内容
}

/**
  * @name	CAN1_SendBoardB_AboutCapacitior()
  * @brief 向B板发送电容控制信息 < 测试可用 >
	* @param 控制信息：1 打开电容；0 关闭电容
  * @return None
  */
void CAN1_SendBoardB_AboutCapacitior(u8 status)
{
	  // 设置电容控制的信息
		CanTxMsg TxMessage;    
    TxMessage.StdId = 0x401; /*  CMDID 具体可在斟酌 */
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 0x01; 
	  TxMessage.Data[0] =(uint8_t)status; 
		CAN_Transmit(CAN1,&TxMessage);
}



