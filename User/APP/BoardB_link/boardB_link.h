	/**************************************************************************
	 * @file     	board-b_link.h
	 * @brief    	通过CAN2向B板通信,包括裁判系统，pc，超级电容之间的通信
	 * @writer		宋立栋
	 * @Q Q			2296054658
	 **************************************************************************/
	#ifndef _BOARD_B_LINK_
	#define _BOARD_B_LINK_
	#include "stm32f4xx.h"
	#include "main.h"
	#define VisionBufferLength       255
	/*裁判系统数据结构体声明*/
	typedef struct
	{
		u8 u8_state;
	}Match;

	typedef struct
	{
		u8 u8_level;			//当前等级
		s16 s16_remain_blood;	//剩余血量
		u8 b_chassis_out;		//底盘是否有输出
		u8 b_shoot_out;			//发射是否有输出
	}Robot;

	typedef struct
	{
		s16 s16_Shoot_heat; 		//当前枪管热量
		s16 s16_Shoot_heat_limit; 	//枪管热量上限
		s16 s16_Shoot_cooling_rate;	//枪管每秒冷却值
		s8  s8_Shoot_speed_limit;	//枪口上限速度
		s8  s8_Shoot_frequency;		//子弹射频
		s16 s16_Shoot_speed;		//子弹射速	
	}Shoot;

	
	
	typedef struct
	{
		s8 chassis_power_limit;	//底盘最大功率
		fp32 chassis_power;		//底盘功率
		uint16_t chassis_power_buff;	//缓冲能量剩余值
		u8 u8_Cap_Power_Flag;		//1代表电容有电，0代表电容没电
	}Power;

	typedef struct
	{
		s8 Robot_Hurt_State;	//机器人受伤状态
	}Hurt;

	typedef struct
	{
		Power power;		//功率相关信息
		Shoot shoot;		//发射相关信息
		Robot robot;		//机器人相关信息
		Match match;		//比赛相关信息
		Hurt  hurt;			//机器人受伤状态
	}RefreeSyetem_t;

	/*B板发送数据结构体声明*/
	typedef struct
	{
		u8 ifGetAmor;				//是否获取装甲板位置
		u8 ifGetBuff;				//是否打符
		u8 ifOpen_SuperCapacity;	//是否打开电容
	}BoardBDataSend_t;

	/*--------------------------------2021视觉电控通信协议-------------------------------------*/

	
	typedef struct
	{
		
		BoardBDataSend_t BoardBDataSend;
		RefreeSyetem_t RefreeSyetem;
	}BoardBLink_t;

	typedef union
{
	u8 data[4];
	float f_data;
} float_u;

	\
	extern BoardBLink_t BoardBLink;
	void boardB_link_Init();					//B板通信初始化
	void boardB_link_Task();					//B板通信任务
	/*--------------------------------裁判系统数据读取函数-------------------------------------*/
	void Game_State_Recieve(CanRxMsg *rx_message);
	void Robot_State_Recieve(CanRxMsg *rx_message);
	void Robot_PowerOut_Recieve(CanRxMsg *rx_message);
	void Robot_Power_Recieve(CanRxMsg *rx_message);
	void Robot_ShootHeat_Recieve(CanRxMsg *rx_message);
	void Robot_HURT_Recieve(CanRxMsg *rx_message);
	void Robot_ShootSpeed_Recieve(CanRxMsg *rx_message);
void Restart_UsrInterFace(void);	// UI重启
void SendToBoardB_aboutAngle(float yaw_angle, float pitch_angle); // 发送角度
void SendToBoardB_aboutChassis(float chassis_angle);
void SendToBoardB_aboutFire(u8 fire_mode, u8 shoot_speed);
void SendToBoardB_aboutShoot(u8 fire_mode, u8 shoot_speed);
void SendToBoardB_aboutGongdan(uint8_t shootfla);

void getMessage_102(CanRxMsg *rx_message);
void getMessage_103(CanRxMsg *rx_message);
void getMessage_104(CanRxMsg *rx_message);
/*--------------------------------PC数据读取函数-------------------------------------*/
void Amor_Pos_Recieve(CanRxMsg *rx_message);
void Buff_Pos_Recieve(CanRxMsg *rx_message);
void PC_FeedBack(CanRxMsg *rx_message);
	
	
	#endif