/**************************************************************************
 * @file     	imu_ex.h
 * @brief    	外部陀螺仪数据处理,陀螺仪型号：维特智能WT931
 * @writer		宋立栋
 * @Q Q			2296054658
 **************************************************************************/
#ifndef _IMU_EX_H
#define _IMU_EX_H
#define _IMU_USE_USART
#include "stm32f4xx.h"

/*要用到的结构体声明*/
typedef struct
{
	struct
	{
		union{
			u8 angle_8b[9];
			int16_t angle_16b[3];
		}angle_union;
		double pitch;//俯仰角
		double roll;//横滚角
		double yaw;//航向角
	}angle;
	struct
	{		
		union{
			u8 v_8b[9];
			int16_t v_16b[3];
		}v_union;
		double pitch;//俯仰角
		double roll;//横滚角
		double yaw;//航向角
	}v;
	double filler_k;		//低通滤波参数
	double pre_yaw;	
	u8 resetFlag;
}MPU6050_t;

void IMUEX_Init();	//陀螺仪初始化
void IMU_dataRecieve(unsigned char *gyro_rx_buffer);	//用于接收陀螺仪发送来的数据
extern MPU6050_t MPUEX;	//保存陀螺仪数据信息
void IMU_Reset();
void IMU_ResetStep(u8 step);
#endif /*_IMU_EX_H*/
