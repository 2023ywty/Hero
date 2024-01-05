/**************************************************************************
 * @ file     	PID.h
 * @ brief    	PID算法
 * @ writer		宋立栋
 * @ Q Q		2296054658
 **************************************************************************/
#ifndef _PID_H
#define _PID_H

#include "stm32f4xx.h"

typedef enum
{
	SPEED = 0, 	
	POSITION_180,   
	POSITION_360,
} pid_mode_e;
typedef struct
{
    //PID 三参数
	float Kp;			//比例系数，主要控制相应速度，大了容易超调
	float Ki;			//积分系数，用于消除稳态误差，增大小误差范围内的反馈立
	float Kd;			//微分系数，实现具有预测功能，减小超调
	float Ka;			//低通滤波系数 属于[0,1),数越大滤波约明显，当出现明显震荡的时候将此参数调大

	float max_out;  		//最大输出
	float dead_band;		//PID偏差死区
	float intergral_band;	//积分区
	float intergral_maxOut;	//积分最大输出
	float max_input;		//最大输入
	pid_mode_e model;			//PID控制类型，0为速度控制，1为位置控制且最大输入对应180度（实际度数有正有负），2为位置控制且最大输入对应360度（实际度数都为正）
	
	//PID输出值
	float output;		//输出
	float P_output;		//P输出
	float I_output;		//I输出
	float D_output;		//D输出
	
	//误差
	float err;				//当前误差
	float err_last;			//上一次的误差
	float d_last;			//上一次的系数D
} PidTypeDef;

void PIDInit(PidTypeDef *pid,float kp,float ki,float kd,float ka,float max_out,float dead_band,float i_band,float max_input, float i_maxout, pid_mode_e model);
void PIDOutputClear(PidTypeDef *pid);
void PID_Calc(PidTypeDef *pid, float rel_val, float set_val);	//PID计算函数

#endif