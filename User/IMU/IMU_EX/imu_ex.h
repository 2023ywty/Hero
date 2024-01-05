/**************************************************************************
 * @file     	imu_ex.h
 * @brief    	�ⲿ���������ݴ���,�������ͺţ�ά������WT931
 * @writer		������
 * @Q Q			2296054658
 **************************************************************************/
#ifndef _IMU_EX_H
#define _IMU_EX_H
#define _IMU_USE_USART
#include "stm32f4xx.h"

/*Ҫ�õ��Ľṹ������*/
typedef struct
{
	struct
	{
		union{
			u8 angle_8b[9];
			int16_t angle_16b[3];
		}angle_union;
		double pitch;//������
		double roll;//�����
		double yaw;//�����
	}angle;
	struct
	{		
		union{
			u8 v_8b[9];
			int16_t v_16b[3];
		}v_union;
		double pitch;//������
		double roll;//�����
		double yaw;//�����
	}v;
	double filler_k;		//��ͨ�˲�����
	double pre_yaw;	
	u8 resetFlag;
}MPU6050_t;

void IMUEX_Init();	//�����ǳ�ʼ��
void IMU_dataRecieve(unsigned char *gyro_rx_buffer);	//���ڽ��������Ƿ�����������
extern MPU6050_t MPUEX;	//����������������Ϣ
void IMU_Reset();
void IMU_ResetStep(u8 step);
#endif /*_IMU_EX_H*/
