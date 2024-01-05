#include "BoardB_link.h"
#include "can.h"
#include "shoot_task.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "gimbal_behaviour.h"
#include "timer_send_task.h"
#include "string.h"
#include "Remote_Control.h"
#include "shoot_behaviour.h"
/*******************************�ⲿ�ӿ�����*******************************/
BoardBLink_t BoardBLink;
/*************************************************************************/

/**
  * @name	boardB_link_Init()
  * @brief 	B��ͨ�ų�ʼ��
  * @param  None
  */
void boardB_link_Init()
{
	BoardBLink.RefreeSyetem.power.u8_Cap_Power_Flag = 0;
	BoardBLink.RefreeSyetem.power.chassis_power_buff = 60;
}

/**
  * @name	boardB_link_Debug()
  * @brief 	B��ͨ������
  * @param  None
  */
void boardB_link_Debug()
{
	
}



/**
  * @name	boardB_link_Task()
  * @brief 	B��ͨ������
  * @param  None
  */
int A=0,b=0;
void boardB_link_Task(void *pvParameters)
{
	static int shoot_Speed=10;
	portTickType currentTime;
	vTaskDelay(10);
	currentTime = xTaskGetTickCount();
	while(1)
	{	
   
		//boardB_link_Debug();
		if(chassisTaskStructure.if_want_to_open_cap == CAP_CONFIG)
	  {
			CAN1_SendBoardB_AboutCapacitior(1);
			chassisTaskStructure.if_want_to_open_cap = CAP_UNCONFIG;
		}
	  else
		{
			CAN1_SendBoardB_AboutCapacitior(0);	
      chassisTaskStructure.if_want_to_open_cap = CAP_UNCONFIG;			
		}
		
	  SendToBoardB_aboutAngle(gimbalTaskStructure.yawMotor.angle[ENCONDE] ,gimbalTaskStructure.pitchMotor.angle[ENCONDE]);
	  SendToBoardB_aboutChassis( *(chassisTaskStructure.relativeAngle)<0?360+*(chassisTaskStructure.relativeAngle):*(chassisTaskStructure.relativeAngle));
		
		if(gimbalTaskStructure.nowBehaviorName==GIMBAL_NORMAL)
		{//��̨��������ģʽ
	    SendToBoardB_aboutFire(1,shootTaskStructure.shootspeed_set);	
		}	
		else if(gimbalTaskStructure.nowBehaviorName==AUTO)
		{//��̨��������ģʽ
	    SendToBoardB_aboutFire(2,shootTaskStructure.shootspeed_set);	
		}
		
//    SendToBoardB_aboutShoot(shootTaskStructure.shoot_sensor.Projectile_if_Prepare,shootTaskStructure.shootfirturn_flag);
//		SendToBoardB_aboutGongdan(shootTaskStructure.shoot_sensor.shootPlateflag);
		vTaskDelayUntil(&currentTime, 5);
	}
}




/**
  * @name	Game_State_Recieve()
  * @brief 	���ղ���ϵͳ������Ϣ
  * @param  rx_message:can�����������ݽṹ��
  */	
void Game_State_Recieve(CanRxMsg *rx_message)
{
	BoardBLink.RefreeSyetem.match.u8_state= rx_message->Data[0];
}

/**
  * @name	Robot_State_Recieve()
  * @brief 	���ղ���ϵͳ��������Ϣ
  * @param  rx_message:can�����������ݽṹ��
  */	
void Robot_State_Recieve(CanRxMsg *rx_message)
{
	BoardBLink.RefreeSyetem.robot.u8_level = rx_message->Data[1];
	BoardBLink.RefreeSyetem.robot.s16_remain_blood = (rx_message->Data[3]<<8) | rx_message->Data[2];
	BoardBLink.RefreeSyetem.robot.b_chassis_out = rx_message->Data[7] & 0x02;
	BoardBLink.RefreeSyetem.robot.b_shoot_out = rx_message->Data[7] & 0x04;
}


/**
  * @name	Robot_Power_Recieve()
  * @brief 	���չ�����Ϣ
  * @param  rx_message:can�����������ݽṹ��
  */	
void Robot_Power_Recieve(CanRxMsg *rx_message)
{
	float power = (float) ((rx_message->Data[2]<<8) | rx_message->Data[1]);
	BoardBLink.RefreeSyetem.power.chassis_power_limit = rx_message->Data[0];
	BoardBLink.RefreeSyetem.power.chassis_power = power/10;
	BoardBLink.RefreeSyetem.power.chassis_power_buff = (rx_message->Data[4]<<8) | rx_message->Data[3];
}

/**
  * @name	Robot_ShootHeat_Recieve()
  * @brief 	���շ���������Ϣ
  * @param  rx_message:can�����������ݽṹ��
  */	
void Robot_ShootHeat_Recieve(CanRxMsg *rx_message)
{
	BoardBLink.RefreeSyetem.shoot.s16_Shoot_heat = (rx_message->Data[1]<<8) | rx_message->Data[0];
	BoardBLink.RefreeSyetem.shoot.s16_Shoot_heat_limit = (rx_message->Data[3]<<8) | rx_message->Data[2];
	BoardBLink.RefreeSyetem.shoot.s16_Shoot_cooling_rate = (rx_message->Data[5]<<8) | rx_message->Data[4];
}

/**
  * @name	Robot_HURT_Recieve()
  * @brief 	���ջ���������״̬��Ϣ
  * @param  rx_message:can�����������ݽṹ��
  */	
void Robot_HURT_Recieve(CanRxMsg *rx_message)
{
	BoardBLink.RefreeSyetem.hurt.Robot_Hurt_State = rx_message->Data[0];
}

/**
  * @name	Robot_ShootSpeed_Recieve()
  * @brief 	���շ����ٶ���Ϣ
  * @param  rx_message:can�����������ݽṹ��
  */	
void Robot_ShootSpeed_Recieve(CanRxMsg *rx_message)
{
	BoardBLink.RefreeSyetem.shoot.s8_Shoot_speed_limit = rx_message->Data[0];
	BoardBLink.RefreeSyetem.shoot.s8_Shoot_frequency = rx_message->Data[1];
	BoardBLink.RefreeSyetem.shoot.s16_Shoot_speed =(rx_message->Data[3]<<8) | rx_message->Data[2];
}



///// �õ�����ϵͳ���е���Ϣ
void getMessage_102(CanRxMsg *rx_message)
{
	// u8 level, u8 state, u8 chasis_powerOutput, u8 shoot_powerOutput, u16 last_blood
	BoardBLink.RefreeSyetem.match.u8_state= (rx_message->Data[0] & 0x3C) >> 2;
	BoardBLink.RefreeSyetem.robot.u8_level = (rx_message->Data[0] & 0x03);			//����λΪ�ȼ���Ϣ
	BoardBLink.RefreeSyetem.robot.s16_remain_blood = (rx_message->Data[2] << 8) | rx_message->Data[1];
}


void getMessage_103(CanRxMsg *rx_message)
{
	// u8 power, u8 power_buff, u16 speed_17mm, u16 heart_17mm, u16 heart_42mm, u8 capEmpty_flag
	BoardBLink.RefreeSyetem.power.chassis_power      =  (rx_message->Data[1] << 8) | rx_message->Data[0];
	BoardBLink.RefreeSyetem.power.chassis_power_buff =  (rx_message->Data[3] << 8) | rx_message->Data[2];
//	BoardBLink.RefreeSyetem.shoot.s16_Shoot_heat = (rx_message->Data[3] << 8) | rx_message->Data[2];
//	BoardBLink.RefreeSyetem.shoot.s16_Shoot_speed =(rx_message->Data[5] << 8) | rx_message->Data[4];	
}


void getMessage_104(CanRxMsg *rx_message)
{
	BoardBLink.RefreeSyetem.shoot.s16_Shoot_speed = (rx_message->Data[2]<<8) | rx_message->Data[1];
	BoardBLink.RefreeSyetem.shoot.s16_Shoot_heat = (rx_message->Data[4]<<8) | rx_message->Data[3];
	BoardBLink.RefreeSyetem.shoot. s8_Shoot_frequency = (rx_message->Data[6]<<8) | rx_message->Data[5];  //��ȴֵ 
}




/**
  * @name	Restart_UsrInterFace()
  * @brief ��B�巢���û������������ƺ������㷢1
	* @param None
  * @return None
  */
void Restart_UsrInterFace(void){
		CanTxMsg TxMessage;    
    TxMessage.StdId = 0x503; /*  CMDID ����������� */
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 0x01; 
	  TxMessage.Data[0] =1;
		CAN_Transmit(CAN1,&TxMessage);
}

/**
  * @name	 
  * @brief ��B�巢��UI��Ϣ��yaw-angle��pitch-angle������ģʽ����㷢�������̽Ƕ�
	* @param ������Ϣ������float������ռ��λ
  * @return None
  */
void SendToBoardB_aboutAngle(float yaw_angle, float pitch_angle)
{
	// typedef unsigned char uint8_t;
	  // ���õ��ݿ��Ƶ���Ϣ
		CanTxMsg TxMessage; 
		float_u temp_forFloat;
    TxMessage.StdId = 0x501; /*  CMDID ����������� */
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 0x08; 

		temp_forFloat.f_data = yaw_angle;
	  TxMessage.Data[0] = temp_forFloat.data[0];
		TxMessage.Data[1] = temp_forFloat.data[1];
		TxMessage.Data[2] = temp_forFloat.data[2];
		TxMessage.Data[3] = temp_forFloat.data[3];
	
		temp_forFloat.f_data = pitch_angle;
	  TxMessage.Data[4] = temp_forFloat.data[0];
		TxMessage.Data[5] = temp_forFloat.data[1];
		TxMessage.Data[6] = temp_forFloat.data[2];
		TxMessage.Data[7] = temp_forFloat.data[3];
	
		CAN_Transmit(CAN1,&TxMessage);
}



/**
  * @name	 SendToBoardB_aboutChassis
  * @brief ��B�巢��UI��Ϣ������ģʽ����ʱ��㷢�������̽Ƕ�
	* @param ������Ϣ��һ��u8��һ��float������ռ��λ
  * @return None
  */
void SendToBoardB_aboutChassis(float chassis_angle)
{
		CanTxMsg TxMessage; 
		float_u temp_forFloat;
    TxMessage.StdId = 0x502; /*  CMDID ����������� */
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 0x04; 
		temp_forFloat.f_data = chassis_angle;
	  TxMessage.Data[0] = temp_forFloat.data[0];
		TxMessage.Data[1] = temp_forFloat.data[1];
		TxMessage.Data[2] = temp_forFloat.data[2];
		TxMessage.Data[3] = temp_forFloat.data[3];	
		CAN_Transmit(CAN1,&TxMessage);
}

/**
  * @name	 SendToBoardB_aboutChassis
  * @brief ��B�巢��UI��Ϣ������ģʽ����ʱ��㷢�������̽Ƕ�
	* @param ������Ϣ��һ��u8��һ��float������ռ��λ
  * @return None
  */
void SendToBoardB_aboutFire(u8 fire_mode,u8 shoot_speed)
{
		CanTxMsg TxMessage; 
		float_u temp_forFloat;
    TxMessage.StdId = 0x504; /*  CMDID ����������� */
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 0x02; 
		TxMessage.Data[0] = fire_mode;
	  TxMessage.Data[1] = shoot_speed; 

		CAN_Transmit(CAN1,&TxMessage);
}


/**
  * @name	 SendToBoardB_aboutChassis
  * @brief ��B�巢��UI��Ϣ������ģʽ����ʱ��㷢�������̽Ƕ�
	* @param ������Ϣ��һ��u8��һ��float������ռ��λ
  * @return None
  */
void SendToBoardB_aboutShoot(u8 sensor, u8 firmotor)
{
		CanTxMsg TxMessage; 
		float_u temp_forFloat;
    TxMessage.StdId = 0x505; 
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 0x02; 
		TxMessage.Data[0] = sensor;
	  TxMessage.Data[1] = firmotor; 

		CAN_Transmit(CAN1,&TxMessage);
}
/**
  * @name	 SendToBoardB_aboutChassis
  * @brief ��B�巢��UI��Ϣ������״̬
	* @param ������Ϣ��һ��u8��һ��float������ռ��λ
  * @return None
  */
void SendToBoardB_aboutGongdan(u8 dancang_zhuangtai)
{
		CanTxMsg TxMessage; 
		float_u temp_forFloat;
    TxMessage.StdId = 0x506; 
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 0x02; 
		TxMessage.Data[0] = dancang_zhuangtai;
		CAN_Transmit(CAN1,&TxMessage);
}
/*
   
  //��������

	// ���ձ���״̬���ݣ�state
	BoardBLink.RefreeSyetem.match.u8_state= rx_message->Data[0];
	
	// ���ձ���״̬���ݣ���ǰ�ȼ�
	//				 �����Ƿ������
	//				 �����Ƿ������
	BoardBLink.RefreeSyetem.robot.u8_level = rx_message->Data[1];
	BoardBLink.RefreeSyetem.robot.s16_remain_blood = (rx_message->Data[3]<<8) | rx_message->Data[2];
	BoardBLink.RefreeSyetem.robot.b_chassis_out = rx_message->Data[7] & 0x02;
	BoardBLink.RefreeSyetem.robot.b_shoot_out = rx_message->Data[7] & 0x04;	
	
	
	// ���ձ���״̬���ݣ����������
	//					 ��ǰ����	
	//					 ���̻���
	float power = (float) ((rx_message->Data[2]<<8) | rx_message->Data[1]);
	BoardBLink.RefreeSyetem.power.chassis_power_limit = rx_message->Data[0];
	BoardBLink.RefreeSyetem.power.chassis_power = power/10;
	BoardBLink.RefreeSyetem.power.chassis_power_buff = (rx_message->Data[4]<<8) | rx_message->Data[3];
	
	// ���ձ���״̬���ݣ�17mm����
	//					 17mm��������
	//					 ǹ��ÿ����ȴֵ	
	BoardBLink.RefreeSyetem.shoot.s16_17mmShoot_heat = (rx_message->Data[1]<<8) | rx_message->Data[0];
	BoardBLink.RefreeSyetem.shoot.s16_17mmShoot_heat_limit = (rx_message->Data[3]<<8) | rx_message->Data[2];
	BoardBLink.RefreeSyetem.shoot.s16_17mmShoot_cooling_rate = (rx_message->Data[5]<<8) | rx_message->Data[4];
	
	// ���ձ���״̬���ݣ�����������״̬
	BoardBLink.RefreeSyetem.hurt.Robot_Hurt_State = rx_message->Data[0];
	
	// ���ձ���״̬���ݣ�17mm��������
	// 		  		 17mm��Ƶ
	//	  			 17mm����
	BoardBLink.RefreeSyetem.shoot.s8_17mmShoot_speed_limit = rx_message->Data[0];
	BoardBLink.RefreeSyetem.shoot.s8_17mmShoot_frequency = rx_message->Data[1];
	BoardBLink.RefreeSyetem.shoot.s16_17mmShoot_speed =(rx_message->Data[3]<<8) | rx_message->Data[2];	

*/