	/**************************************************************************
	 * @file     	board-b_link.h
	 * @brief    	ͨ��CAN2��B��ͨ��,��������ϵͳ��pc����������֮���ͨ��
	 * @writer		������
	 * @Q Q			2296054658
	 **************************************************************************/
	#ifndef _BOARD_B_LINK_
	#define _BOARD_B_LINK_
	#include "stm32f4xx.h"
	#include "main.h"
	#define VisionBufferLength       255
	/*����ϵͳ���ݽṹ������*/
	typedef struct
	{
		u8 u8_state;
	}Match;

	typedef struct
	{
		u8 u8_level;			//��ǰ�ȼ�
		s16 s16_remain_blood;	//ʣ��Ѫ��
		u8 b_chassis_out;		//�����Ƿ������
		u8 b_shoot_out;			//�����Ƿ������
	}Robot;

	typedef struct
	{
		s16 s16_Shoot_heat; 		//��ǰǹ������
		s16 s16_Shoot_heat_limit; 	//ǹ����������
		s16 s16_Shoot_cooling_rate;	//ǹ��ÿ����ȴֵ
		s8  s8_Shoot_speed_limit;	//ǹ�������ٶ�
		s8  s8_Shoot_frequency;		//�ӵ���Ƶ
		s16 s16_Shoot_speed;		//�ӵ�����	
	}Shoot;

	
	
	typedef struct
	{
		s8 chassis_power_limit;	//���������
		fp32 chassis_power;		//���̹���
		uint16_t chassis_power_buff;	//��������ʣ��ֵ
		u8 u8_Cap_Power_Flag;		//1��������е磬0�������û��
	}Power;

	typedef struct
	{
		s8 Robot_Hurt_State;	//����������״̬
	}Hurt;

	typedef struct
	{
		Power power;		//���������Ϣ
		Shoot shoot;		//���������Ϣ
		Robot robot;		//�����������Ϣ
		Match match;		//���������Ϣ
		Hurt  hurt;			//����������״̬
	}RefreeSyetem_t;

	/*B�巢�����ݽṹ������*/
	typedef struct
	{
		u8 ifGetAmor;				//�Ƿ��ȡװ�װ�λ��
		u8 ifGetBuff;				//�Ƿ���
		u8 ifOpen_SuperCapacity;	//�Ƿ�򿪵���
	}BoardBDataSend_t;

	/*--------------------------------2021�Ӿ����ͨ��Э��-------------------------------------*/

	
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
	void boardB_link_Init();					//B��ͨ�ų�ʼ��
	void boardB_link_Task();					//B��ͨ������
	/*--------------------------------����ϵͳ���ݶ�ȡ����-------------------------------------*/
	void Game_State_Recieve(CanRxMsg *rx_message);
	void Robot_State_Recieve(CanRxMsg *rx_message);
	void Robot_PowerOut_Recieve(CanRxMsg *rx_message);
	void Robot_Power_Recieve(CanRxMsg *rx_message);
	void Robot_ShootHeat_Recieve(CanRxMsg *rx_message);
	void Robot_HURT_Recieve(CanRxMsg *rx_message);
	void Robot_ShootSpeed_Recieve(CanRxMsg *rx_message);
void Restart_UsrInterFace(void);	// UI����
void SendToBoardB_aboutAngle(float yaw_angle, float pitch_angle); // ���ͽǶ�
void SendToBoardB_aboutChassis(float chassis_angle);
void SendToBoardB_aboutFire(u8 fire_mode, u8 shoot_speed);
void SendToBoardB_aboutShoot(u8 fire_mode, u8 shoot_speed);
void SendToBoardB_aboutGongdan(uint8_t shootfla);

void getMessage_102(CanRxMsg *rx_message);
void getMessage_103(CanRxMsg *rx_message);
void getMessage_104(CanRxMsg *rx_message);
/*--------------------------------PC���ݶ�ȡ����-------------------------------------*/
void Amor_Pos_Recieve(CanRxMsg *rx_message);
void Buff_Pos_Recieve(CanRxMsg *rx_message);
void PC_FeedBack(CanRxMsg *rx_message);
	
	
	#endif