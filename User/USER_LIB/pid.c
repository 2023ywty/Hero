/**************************************************************************
 * @ file     	PID.c
 * @ brief    	PID算法
 * @ writer		宋立栋
 * @ Q Q		2296054658
 **************************************************************************/
#include "pid.h"
#include "user_lib.h"

/**
  * @name 	PIDInit()
  * @brief 	PID参数初始化
  * @param	pid:所要初始化的结构体
  * @param	kp:系数p
  * @param	ki:系数i
  * @param	kd:系数d
  * @param	ka:系数a
  * @param	max_out:最大输出
  * @param	dead_band:死区，在此范围内不进行PID计算直接输出0
  * @param	i_band:积分区间，在此范围内进行积分
  * @param	max_input:期望值的最大输入
  * @param	e_max:计算时的最大误差
  * @param	i_maxout:i的最大输出
  * @return None
  */
void PIDInit(PidTypeDef *pid,float kp,float ki,float kd,float ka,float max_out,float dead_band,float i_band,float max_input, float i_maxout, pid_mode_e model)
{
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	
	if(ka < 0 || ka >= 1)
		pid->Ka = 0;
	else
		pid->Ka = ka;
	
	if(max_out < 0)
		pid->max_out = 65535/2 - 1;
	else
		pid->max_out = max_out;
	
	if(max_input < 0)
		pid->max_input = 65535/2-1;
	else
		pid->max_input = max_input;
	
	if(i_maxout < 0)
		pid->intergral_maxOut = pid->max_out;
	else
		pid->intergral_maxOut = i_maxout;
	
	if(dead_band < 0)
		pid->dead_band = 0;
	else
		pid->dead_band = dead_band;
	
	if(i_band < 0)
		pid->intergral_band = 2*pid->max_input;
	else
		pid->intergral_band = i_band;
	
	pid->model = model;
}

/**
  * @name 	PIDOutputClear()
  * @brief 	清空PID结构体的输出
  * @param	pid:所要清空的结构体
  * @return None
  */
void PIDOutputClear(PidTypeDef *pid){
	pid->output = 0;
	pid->I_output = 0;
}

/**
  * @name 	PID_Calc()
  * @brief 	PID计算函数，将输出结果保存在pid->output变量中
  * @param	pid:含有PID计算参数的结构体
  * @param	rel_val:PID计算的实际值值
  * @param	set_val:PID计算的期望值
  * @return None
  */
void PID_Calc(PidTypeDef *pid, float rel_val, float set_val)
{
	if(set_val > pid->max_input)
		set_val = pid->max_input;
	else if(set_val < -(pid->max_input))
		set_val = -(pid->max_input);

	pid->err = set_val - rel_val; //当前误差
	
	/*360转的PID位置计算过0处理*/
	if(pid->model == POSITION_360)
	{
		if(pid->err>(pid->max_input/2))
		{                                               //云台操作
			pid->err=pid->err- pid->max_input;
		}
		else if((-pid->err)>(pid->max_input/2))
		{
			pid->err=pid->max_input + pid->err;
		}
	}
	else if(pid->model == POSITION_180)
	{
		if(pid->err>(pid->max_input))
		{
			pid->err=pid->err- 2 * pid->max_input;        //云台操作
		}
		else if((-pid->err)>(pid->max_input))
		{
			pid->err=pid->max_input * 2 + pid->err;
		}
	}

	if(f_abs(pid->err) > pid->dead_band)
	{ //判断是否进入死区，没有进入则进行PID运算，进入则输出0
		
		//判断是否在积分区间内，如果在就进行积分，不在就使得积分逐渐归0
		if(f_abs(pid->err) <= pid->intergral_band)
			pid->I_output = pid->I_output + (pid->Ki) * (pid->err);
		else
			pid->I_output = pid->I_output*0.99;
		
		//控制积分最大值
		if(pid->I_output > pid->intergral_maxOut)
			pid->I_output = pid->intergral_maxOut;
		else if(pid->I_output < -pid->intergral_maxOut)
			pid->I_output= -pid->intergral_maxOut;
		
		//计算PID输出
		pid->P_output = pid->Kp * (pid->err);
		pid->D_output = pid->Kd * (1-pid->Ka)*(pid->err - pid->err_last) + (pid->Ka)*(pid->d_last);
		pid->d_last = pid->D_output;
		pid->output = pid->P_output+pid->I_output+pid->D_output;
		
		//是否超出最大输出
		if(pid->output > pid->max_out)
			pid->output = pid->max_out;
		if(pid->output < -(pid->max_out))
			pid->output = -(pid->max_out);
			
	}
	else
	{
		pid->output *= 0.99;
		pid->I_output *= 0.99;
	}
	
	pid->err_last = pid->err;
}