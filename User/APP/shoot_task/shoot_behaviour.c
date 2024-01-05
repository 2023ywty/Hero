#include "shoot_behaviour.h"
#include "robot.h"
#include "detect_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "Remote_Control.h"
#include "user_lib.h"
#include "oscillography.h"
#include "chassis_task.h"
#include "delay.h"
#include "led.h"
#include "gimbal_behaviour.h"

int beijing_fla=0;
/**********************************************DUBUGģʽ************************************************/
bool_t shootbehDebugEnterCondition()
{
	#ifdef DEBUG_SHOOT
		return TRUE;
	#endif
		
	#ifndef DEBUG_SHOOT
		return FALSE;
	#endif
}

bool_t shootbehDebugOutCondition()
	{
	#ifdef DEBUG_SHOOT
		return FALSE;
	#endif
		
	#ifndef DEBUG_SHOOT
		return TRUE;
	#endif
}

void shootbehDebugHandleFun(float *leftFirExp, float *rightFirExp, float*plateExp)
{
	if(IF_RC_SW2_UP && IF_RC_SW1_UP)
	{
		*rightFirExp = RAMP_float(-7, *rightFirExp, 18/((float)(1000/SHOOT_TASK_MS)));
	  *leftFirExp  = RAMP_float(7, *leftFirExp,  18/((float)(1000/SHOOT_TASK_MS)));
		*plateExp		 = (float)RC_CH3_LUD_OFFSET/660.0 * 40;
		if(RC_CH1_RUD_OFFSET > 500)
		{
		Electricpush_off(); 
		}
		else
		{
		Electricpush_on();
		}
	}
	else
	{
		*rightFirExp = RAMP_float(0, *rightFirExp, 18/((float)(1000/SHOOT_TASK_MS)));
	  *leftFirExp  = RAMP_float(0, *leftFirExp,  18/((float)(1000/SHOOT_TASK_MS)));
	}

}
/**********************************************DUBUGģʽ************************************************/


/**********************************************ZeroForceģʽ************************************************/
bool_t shootbehZeroForceEnterCondition()
{
	if(toe_is_error(DBUSTOE) || toe_is_error(DETECT_SHOOT_LF_MOTOR) || toe_is_error(DETECT_SHOOT_RF_MOTOR) || toe_is_error(DETECT_SHOOT_PLATE_MOTOR))
		return TRUE;
	return FALSE;
}

bool_t shootbehZeroForceOutCondition()
{
	if(shootbehZeroForceEnterCondition())
		return FALSE;
	return TRUE;
}

void shootbehZeroForceHandleFun(float *leftFirExp, float *rightFirExp, float *plateExp)
{
	*leftFirExp = 0;
	*rightFirExp = 0;
	*plateExp = 0;
   Electricpush_on();		
}
/**********************************************ZeroForceģʽ************************************************/
