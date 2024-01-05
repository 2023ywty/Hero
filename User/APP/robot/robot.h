#ifndef ROBOT_H
#define ROBOT_H

#include "main.h"



//#define MID_GIMBAL //��̨����ʹ������궨��
#define MID_GIMBAL_YAW_TYPE		GIMBAL_MOTOR_GYRO
#define MID_GIMBAL_PITCH_TYPE		GIMBAL_MOTOR_ENCONDE_SPEED
//#define MID_GIMBAL_PITCH_TYPE		GIMBAL_MOTOR_ENCONDE


#define DEBUG_GIMBAL //��̨����ʹ������궨��
#define DEBUG_GIMBAL_YAW_TYPE		  GIMBAL_MOTOR_GYRO
#define DEBUG_GIMBAL_PITCH_TYPE		GIMBAL_MOTOR_ENCONDE
//#define DEBUG_GIMBAL_PITCH_TYPE		GIMBAL_MOTOR_GYRO


#define DEBUG_CHASSIS //���̵���ʹ������궨��
#define DEBUG_CHSSIS_TYPE		CHASSIS_SPEED//CHASSIS_RAW


#define DEBUG_SHOOT //�������ʹ������궨��
#define DEBUG_SHOOT_LF_TYPE		SHOOT_MOTOR_SPEED//SHOOT_MOTOR_SPEED
#define DEBUG_SHOOT_RF_TYPE		SHOOT_MOTOR_SPEED
#define DEBUG_SHOOT_PL_TYPE		SHOOT_MOTOR_SPEED


#define GIMBAL_TASK_MS 3
#define CHASSIS_TASK_MS 4
#define IMU_TASK_MS 5


typedef enum 
{
	ROBOT_ZERO_FORCE = 0,
	ROBOT_INIT,
	ROBOT_NORMAL,
} robot_mode_e;


typedef enum 
{
	ROBOT_INIT_IMU = 0,
	ROBOT_INIT_GIMBAL,
	ROBOT_INIT_CHASSIS,
	ROBOT_INIT_LENGTH     //��
} robot_init_step;


typedef struct
{
	robot_mode_e robotMode;
	robot_init_step modeStep;
} robot_information_t;

extern robot_information_t robotInf;
void robotInit();


#endif 