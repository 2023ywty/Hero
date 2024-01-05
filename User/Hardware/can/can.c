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
  * @brief 	CAN1��ʼ������
	
  * @param  None
  */
void CAN1_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    //ʹ�����ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��PORTAʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��

    //��ʼ��GPIO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1| GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��PA11,PA12

    //���Ÿ���ӳ������
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_CAN1); //GPIOA11����ΪCAN1
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_CAN1); //GPIOA12����ΪCAN1


    //CAN��Ԫ����
    CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ
    CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���
    CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
    CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ�����
    CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�
    CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������
    CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;	 //ģʽ����
    CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
    CAN_InitStructure.CAN_BS1=CAN_BS1_8tq; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
    CAN_InitStructure.CAN_BS2=CAN_BS2_6tq;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
    CAN_InitStructure.CAN_Prescaler=3;  //��Ƶϵ��(Fdiv)Ϊbrp+1
    CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1

    //���ù�����
    CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
    CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��

    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.

    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;     // �����ȼ�Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @name	CAN2_Init()
  * @brief 	CAN2��ʼ������
  * @param  None
  */
void CAN2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    //ʹ�����ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PORTAʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//ʹ��CAN2ʱ��

    //��ʼ��GPIO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12| GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PA11,PA12

    //���Ÿ���ӳ������
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2); //GPIOB12����ΪCAN2
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2); //GPIOB13����ΪCAN2


    //CAN��Ԫ����
    CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ
    CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���
    CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
    CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ�����
    CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�
    CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������
    CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;	 //ģʽ����
    CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
    CAN_InitStructure.CAN_BS1=CAN_BS1_8tq; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
    CAN_InitStructure.CAN_BS2=CAN_BS2_6tq;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
    CAN_InitStructure.CAN_Prescaler=3;  //��Ƶϵ��(Fdiv)Ϊbrp+1
    CAN_Init(CAN2, &CAN_InitStructure);   // ��ʼ��CAN2

    //���ù�����
    CAN_FilterInitStructure.CAN_FilterNumber=27;	  //������0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
    CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��

    CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.
//    CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);	//����can2�ж�

    NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;     // �����ȼ�Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}




/**
  * @name	CAN1_RX0_IRQHandler()
  * @brief 	CAN1�����жϺ���,���ڽ��յ��̵�� ��̨��� ���������ת�ӡ��ٶ�λ�÷���
  * @param  None
  */
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg rx_message;	
    if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
  	{
    CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
	  CAN_ClearFlag(CAN1, CAN_FLAG_FF0); 		
		CAN_Receive(CAN1, CAN_FIFO0, &rx_message);//��ȡ����	
		switch(rx_message.StdId)
		{//can1�����ĸ������̵����Yaw��һ��������
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
  * @brief 	CAN2�����жϣ����ڽ���B�巢��������Ϣ��CAN2���������ӵĵ����ת�ӡ����ٶ�λ����Ϣ
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
    can_msg.RTR=CAN_RTR_DATA;	// ��Ϣ����Ϊ����֡��һ֡8λ
    can_msg.DLC=8;				// ����8֡��Ϣ

    u8 i = 0;
    for(; i < len; i++)
    {
        can_msg.Data[2*i] = (u8)((int16_t)current[i]>>8);
        can_msg.Data[2*i+1]=(u8)((int16_t)current[i]);
    }

    if(CANx == CAN1)
    {
        xQueueSend(CAN1_Queue,&can_msg,1);//��������������
    }
    else if(CANx == CAN2)
    {
        xQueueSend(CAN2_Queue,&can_msg,1);//��������������
    }
}


/**
  * @name	CAN1_SendBoardB()
  * @brief 	��B�巢��������Ϣ
  * @param  requestMsg:��������
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
  * @brief 	��B�巢����̨����������
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
	xQueueSend(CAN1_Queue,&TxMessage,1);//��������������
}

/**
  * @name	CAN1_SendBoardB_AboutCapacitior()
  * @brief ��B�巢�͵��ݿ�����Ϣ < ���Կ��� >
	* @param ������Ϣ��1 �򿪵��ݣ�0 �رյ���
  * @return None
  */
void CAN1_SendBoardB_AboutCapacitior(u8 status)
{
	  // ���õ��ݿ��Ƶ���Ϣ
		CanTxMsg TxMessage;    
    TxMessage.StdId = 0x401; /*  CMDID ����������� */
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 0x01; 
	  TxMessage.Data[0] =(uint8_t)status; 
		CAN_Transmit(CAN1,&TxMessage);
}



