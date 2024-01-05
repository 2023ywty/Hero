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
/**********************************************DUBUG模式************************************************/
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
/**********************************************DUBUG模式************************************************/


/**********************************************ZeroForce模式************************************************/
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
/**********************************************ZeroForce模式************************************************/


/**********************************************Sensor模式************************************************/
bool_t shootbehSensorEnterCondition() 
{
	return TRUE;
}

bool_t shootbehSensorOutCondition()
{
	return FALSE;
}

//#define SHOOTSPPED 18.6
#define SHOOTSPPED 19.6

int singleFlag = 0;
int singletime= 0;
int singletimeflag= 0;
int gongdan_tim=0;
uint8_t gongdan_fla=0;

int Electricpushflag= 0;
int Electricpushtime= 0;
int Electricpushtimeflag= 0;

int plateflag = 0;

int tuigan_flag;
int tuigan_tim;

int Electricpush2_flag = 0;
int douji_flag =0;	

uint8_t qiangzhi_gongdan_fla=0;
int qiangzhi_gongdan_tim=0;
//等于1强制供弹
uint8_t fan_zhuan=0;
int fan_zhuan_tim;
//等于1反转
unsigned char Firmortor_flag = 0;

uint8_t mocalun_flag=0;

void shootbehSensorHandleFun(float *leftFirExp, float *rightFirExp, float *plateExp)
{
	//C 控制摩擦轮电机
//	if(SHOOT_FIRST_PRESS_C)
//	{
//		Firmortor_flag++;
//	}
//	
//	if((Firmortor_flag%2)==0)
//	{
//		 shootTaskStructure.shootfirturn_flag = 1;
//    *rightFirExp = RAMP_float(-SHOOTSPPED, *rightFirExp, 18/((float)(1000/SHOOT_TASK_MS)));
//	  *leftFirExp  = RAMP_float(SHOOTSPPED, *leftFirExp,  18/((float)(1000/SHOOT_TASK_MS)));
//	}
//	else if((Firmortor_flag%2)!=0||IF_RC_SW1_DOWN)
//	{
//		 shootTaskStructure.shootfirturn_flag = 0;
//    *rightFirExp = RAMP_float(-0, *rightFirExp, 18/((float)(1000/SHOOT_TASK_MS)));
//	  *leftFirExp  = RAMP_float( 0, *leftFirExp,  18/((float)(1000/SHOOT_TASK_MS)));
//	}
	if(IF_RC_SW1_UP)
	{
		mocalun_flag=1;
		*rightFirExp = RAMP_float(-SHOOTSPPED, *rightFirExp, 18/((float)(1000/SHOOT_TASK_MS)));
	  *leftFirExp  = RAMP_float(SHOOTSPPED, *leftFirExp,  18/((float)(1000/SHOOT_TASK_MS)));
	}
	else 
	{
		mocalun_flag=0;
		*rightFirExp = RAMP_float(0, *rightFirExp, 18/((float)(1000/SHOOT_TASK_MS)));
	  *leftFirExp  = RAMP_float(0, *leftFirExp,  18/((float)(1000/SHOOT_TASK_MS)));
	}
	
/////////////////////
	static int Electricpush_flag1=1;
	static int Electricpush_flag2=1;
	int Plate_flag = 0;
	
	if(IF_RC_SW2_UP)
	{
		Plate_flag =1;
	}
	else if(IF_RC_SW2_MID ||IF_RC_SW2_DOWN)
		Plate_flag=0;
	//Z强制供弹
	 if(qiangzhi_gongdan_fla==1)
	{
		*plateExp=-10;
		if(qiangzhi_gongdan_tim>100)
		{
			*plateExp=0;
			qiangzhi_gongdan_fla=0;
			qiangzhi_gongdan_tim=0;
		}
		
	}
//		 if(gongdan_fla==1)
//	{
//		*plateExp=-15;	
//		if(gongdan_tim>300)
//		{
//			*plateExp=0;
//			gongdan_fla=0;
//			gongdan_tim=0;
//		}
//	}
	//X退弹
	if((SHOOT_FIRST_PRESS_X))
	{
		fan_zhuan=1;
	}
	if(fan_zhuan==1)
	{
		*plateExp=20;
		if(fan_zhuan_tim>200)
		{
			*plateExp=0;
		fan_zhuan=0;
		fan_zhuan_tim=0;
		}
	}
	//拨盘电机赋值
	if((shootTaskStructure.shoot_sensor.shootPlateflag == 0)&&( Plate_flag==1))
	{
		gongdan_fla=1;
	}
	//强制供弹
//	else if(qiangzhi_gongdan_fla==1)
//	{
//		*plateExp=-10;
//	}
//  else if(qiangzhi_gongdan_tim>100)
//	{
//		*plateExp=0;
//		qiangzhi_gongdan_fla=0;
//		qiangzhi_gongdan_tim=0;
//	}
	//反转
//	else
//	{
//		*plateExp=0;
//	}
//	

	//每次供弹供15ms
	 if(gongdan_fla==1)
	{
		*plateExp=-15;	
		if(gongdan_tim>15)
		{
			*plateExp=0;
			gongdan_fla=0;
			gongdan_tim=0;
		}
	}

	//电推杆发射
	if(((IF_RC_SW1_UP) &&(IF_RC_SW2_DOWN) )||( IF_MOUSE_PRESSED_LEFT && mocalun_flag==1))
	{
		singleFlag = 1;
	}
	
	if(singleFlag==1)
	{
		singleFlag = 0;
		Electricpush_off(); //电推杆推出
		//开始计时
		singletime = 0; 
		singletimeflag = 1;
		Electricpushflag = 1;
		tuigan_flag=1;
	}
	if(singletime >250)
	{
		Electricpush_on();	
		singletimeflag = 0;
		singletime = 0; 
//		*plateExp=0;
	}
	if((singletime>50)&&(singletime<250))
	{
		*plateExp=0;
	}
	//控制拨盘电机
	if(Electricpushflag == 1)
	{
		Electricpushflag = 0;
		*plateExp = 4;
		Electricpushtime = 0;
		Electricpushtimeflag = 1;
		plateflag = 1;
	}

	if(Electricpushtime > 20)
	{
		*plateExp = 0;
		Electricpushtimeflag = 0;
		Electricpushtime = 0;
		plateflag = 0;
	}
//	if((tuigan_tim>20)&&(tuigan_tim<300))
//	{
//		*plateExp = 0;
//	}
	if(tuigan_tim>450)
	{
		tuigan_flag=0;
		tuigan_tim=0;
	}
	if(tuigan_flag==1)
	{
		*plateExp = 0;
	}

	//按Q控制倍镜
	 if(IF_KEY_PRESSED_Q)
	{
		delay_ms(50);  
		if(IF_KEY_PRESSED_Q)
		{
			douji_flag++;
		if(douji_flag==3)
			douji_flag=0;
		}
	}	
	//鼠标右键倍镜快速回中
		if(IF_MOUSE_PRESSED_RIGH)
	{
		douji_flag=0;		//切换背景标志位
	}
	if(douji_flag==0)
	{
	  beijing_fla=0;//调节鼠标灵敏度标志位
		HEAD_CLOSE();
	}
 if(douji_flag==1)
	{
		beijing_fla=1;//调节鼠标灵敏度标志位
		HEAD_Low_OPEN();
	}	
 if(douji_flag==2)
	{
		beijing_fla=1;//调节鼠标灵敏度标志位
		HEAD_High_OPEN();
	}	
}
/**********************************************Sensor模式************************************************/


