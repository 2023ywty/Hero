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
/*******************************外部接口声明*******************************/
BoardBLink_t BoardBLink;
/*************************************************************************/

/**
  * @name	boardB_link_Init()
  * @brief 	B板通信初始化
  * @param  None
  */
void boardB_link_Init()
{
	BoardBLink.RefreeSyetem.power.u8_Cap_Power_Flag = 0;
	BoardBLink.RefreeSyetem.power.chassis_power_buff = 60;
}

/**
  * @name	boardB_link_Debug()
  * @brief 	B板通信任务
  * @param  None
  */
void boardB_link_Debug()
{
	
}



/**
  * @name	boardB_link_Task()
  * @brief 	B板通信任务
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
		{//云台处于自瞄模式
	    SendToBoardB_aboutFire(1,shootTaskStructure.shootspeed_set);	
		}	
		else if(gimbalTaskStructure.nowBehaviorName==AUTO)
		{//云台处于自瞄模式
	    SendToBoardB_aboutFire(2,shootTaskStructure.shootspeed_set);	
		}
		
//    SendToBoardB_aboutShoot(shootTaskStructure.shoot_sensor.Projectile_if_Prepare,shootTaskStructure.shootfirturn_flag);
//		SendToBoardB_aboutGongdan(shootTaskStructure.shoot_sensor.shootPlateflag);
		vTaskDelayUntil(&currentTime, 5);
	}
}




/**
  * @name	Game_State_Recieve()
  * @brief 	接收裁判系统比赛信息
  * @param  rx_message:can发送来的数据结构体
  */	
void Game_State_Recieve(CanRxMsg *rx_message)
{
	BoardBLink.RefreeSyetem.match.u8_state= rx_message->Data[0];
}

/**
  * @name	Robot_State_Recieve()
  * @brief 	接收裁判系统机器人信息
  * @param  rx_message:can发送来的数据结构体
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
  * @brief 	接收功率信息
  * @param  rx_message:can发送来的数据结构体
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
  * @brief 	接收发射热量信息
  * @param  rx_message:can发送来的数据结构体
  */	
void Robot_ShootHeat_Recieve(CanRxMsg *rx_message)
{
	BoardBLink.RefreeSyetem.shoot.s16_Shoot_heat = (rx_message->Data[1]<<8) | rx_message->Data[0];
	BoardBLink.RefreeSyetem.shoot.s16_Shoot_heat_limit = (rx_message->Data[3]<<8) | rx_message->Data[2];
	BoardBLink.RefreeSyetem.shoot.s16_Shoot_cooling_rate = (rx_message->Data[5]<<8) | rx_message->Data[4];
}

/**
  * @name	Robot_HURT_Recieve()
  * @brief 	接收机器人受伤状态信息
  * @param  rx_message:can发送来的数据结构体
  */	
void Robot_HURT_Recieve(CanRxMsg *rx_message)
{
	BoardBLink.RefreeSyetem.hurt.Robot_Hurt_State = rx_message->Data[0];
}

/**
  * @name	Robot_ShootSpeed_Recieve()
  * @brief 	接收发射速度信息
  * @param  rx_message:can发送来的数据结构体
  */	
void Robot_ShootSpeed_Recieve(CanRxMsg *rx_message)
{
	BoardBLink.RefreeSyetem.shoot.s8_Shoot_speed_limit = rx_message->Data[0];
	BoardBLink.RefreeSyetem.shoot.s8_Shoot_frequency = rx_message->Data[1];
	BoardBLink.RefreeSyetem.shoot.s16_Shoot_speed =(rx_message->Data[3]<<8) | rx_message->Data[2];
}



///// 得到裁判系统所有的信息
void getMessage_102(CanRxMsg *rx_message)
{
	// u8 level, u8 state, u8 chasis_powerOutput, u8 shoot_powerOutput, u16 last_blood
	BoardBLink.RefreeSyetem.match.u8_state= (rx_message->Data[0] & 0x3C) >> 2;
	BoardBLink.RefreeSyetem.robot.u8_level = (rx_message->Data[0] & 0x03);			//低两位为等级信息
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
	BoardBLink.RefreeSyetem.shoot. s8_Shoot_frequency = (rx_message->Data[6]<<8) | rx_message->Data[5];  //冷却值 
}




/**
  * @name	Restart_UsrInterFace()
  * @brief 向B板发送用户界面重启控制函数，恒发1
	* @param None
  * @return None
  */
void Restart_UsrInterFace(void){
		CanTxMsg TxMessage;    
    TxMessage.StdId = 0x503; /*  CMDID 具体可在斟酌 */
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 0x01; 
	  TxMessage.Data[0] =1;
		CAN_Transmit(CAN1,&TxMessage);
}

/**
  * @name	 
  * @brief 向B板发送UI信息：yaw-angle、pitch-angle，开火模式（随便发），底盘角度
	* @param 控制信息：俩个float，数据占四位
  * @return None
  */
void SendToBoardB_aboutAngle(float yaw_angle, float pitch_angle)
{
	// typedef unsigned char uint8_t;
	  // 设置电容控制的信息
		CanTxMsg TxMessage; 
		float_u temp_forFloat;
    TxMessage.StdId = 0x501; /*  CMDID 具体可在斟酌 */
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
  * @brief 向B板发送UI信息：开火模式（暂时随便发），底盘角度
	* @param 控制信息：一个u8，一个float，数据占四位
  * @return None
  */
void SendToBoardB_aboutChassis(float chassis_angle)
{
		CanTxMsg TxMessage; 
		float_u temp_forFloat;
    TxMessage.StdId = 0x502; /*  CMDID 具体可在斟酌 */
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
  * @brief 向B板发送UI信息：开火模式（暂时随便发），底盘角度
	* @param 控制信息：一个u8，一个float，数据占四位
  * @return None
  */
void SendToBoardB_aboutFire(u8 fire_mode,u8 shoot_speed)
{
		CanTxMsg TxMessage; 
		float_u temp_forFloat;
    TxMessage.StdId = 0x504; /*  CMDID 具体可在斟酌 */
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 0x02; 
		TxMessage.Data[0] = fire_mode;
	  TxMessage.Data[1] = shoot_speed; 

		CAN_Transmit(CAN1,&TxMessage);
}


/**
  * @name	 SendToBoardB_aboutChassis
  * @brief 向B板发送UI信息：开火模式（暂时随便发），底盘角度
	* @param 控制信息：一个u8，一个float，数据占四位
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
  * @brief 向B板发送UI信息：弹仓状态
	* @param 控制信息：一个u8，一个float，数据占四位
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
   
  //数据内容

	// 接收比赛状态数据：state
	BoardBLink.RefreeSyetem.match.u8_state= rx_message->Data[0];
	
	// 接收比赛状态数据：当前等级
	//				 底盘是否有输出
	//				 发射是否有输出
	BoardBLink.RefreeSyetem.robot.u8_level = rx_message->Data[1];
	BoardBLink.RefreeSyetem.robot.s16_remain_blood = (rx_message->Data[3]<<8) | rx_message->Data[2];
	BoardBLink.RefreeSyetem.robot.b_chassis_out = rx_message->Data[7] & 0x02;
	BoardBLink.RefreeSyetem.robot.b_shoot_out = rx_message->Data[7] & 0x04;	
	
	
	// 接收比赛状态数据：最大功率限制
	//					 当前功率	
	//					 底盘缓存
	float power = (float) ((rx_message->Data[2]<<8) | rx_message->Data[1]);
	BoardBLink.RefreeSyetem.power.chassis_power_limit = rx_message->Data[0];
	BoardBLink.RefreeSyetem.power.chassis_power = power/10;
	BoardBLink.RefreeSyetem.power.chassis_power_buff = (rx_message->Data[4]<<8) | rx_message->Data[3];
	
	// 接收比赛状态数据：17mm热量
	//					 17mm热量上限
	//					 枪管每秒冷却值	
	BoardBLink.RefreeSyetem.shoot.s16_17mmShoot_heat = (rx_message->Data[1]<<8) | rx_message->Data[0];
	BoardBLink.RefreeSyetem.shoot.s16_17mmShoot_heat_limit = (rx_message->Data[3]<<8) | rx_message->Data[2];
	BoardBLink.RefreeSyetem.shoot.s16_17mmShoot_cooling_rate = (rx_message->Data[5]<<8) | rx_message->Data[4];
	
	// 接收比赛状态数据：机器人受伤状态
	BoardBLink.RefreeSyetem.hurt.Robot_Hurt_State = rx_message->Data[0];
	
	// 接收比赛状态数据：17mm射速上限
	// 		  		 17mm射频
	//	  			 17mm射速
	BoardBLink.RefreeSyetem.shoot.s8_17mmShoot_speed_limit = rx_message->Data[0];
	BoardBLink.RefreeSyetem.shoot.s8_17mmShoot_frequency = rx_message->Data[1];
	BoardBLink.RefreeSyetem.shoot.s16_17mmShoot_speed =(rx_message->Data[3]<<8) | rx_message->Data[2];	

*/