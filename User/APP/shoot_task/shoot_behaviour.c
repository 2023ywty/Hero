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


/**********************************************Sensorģʽ************************************************/
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
//����1ǿ�ƹ���
uint8_t fan_zhuan=0;
int fan_zhuan_tim;
//����1��ת
unsigned char Firmortor_flag = 0;

uint8_t mocalun_flag=0;

void shootbehSensorHandleFun(float *leftFirExp, float *rightFirExp, float *plateExp)
{
	//C ����Ħ���ֵ��
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
	//Zǿ�ƹ���
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
	//X�˵�
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
	//���̵����ֵ
	if((shootTaskStructure.shoot_sensor.shootPlateflag == 0)&&( Plate_flag==1))
	{
		gongdan_fla=1;
	}
	//ǿ�ƹ���
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
	//��ת
//	else
//	{
//		*plateExp=0;
//	}
//	

	//ÿ�ι�����15ms
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

	//���Ƹ˷���
	if(((IF_RC_SW1_UP) &&(IF_RC_SW2_DOWN) )||( IF_MOUSE_PRESSED_LEFT && mocalun_flag==1))
	{
		singleFlag = 1;
	}
	
	if(singleFlag==1)
	{
		singleFlag = 0;
		Electricpush_off(); //���Ƹ��Ƴ�
		//��ʼ��ʱ
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
	//���Ʋ��̵��
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

	//��Q���Ʊ���
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
	//����Ҽ��������ٻ���
		if(IF_MOUSE_PRESSED_RIGH)
	{
		douji_flag=0;		//�л�������־λ
	}
	if(douji_flag==0)
	{
	  beijing_fla=0;//������������ȱ�־λ
		HEAD_CLOSE();
	}
 if(douji_flag==1)
	{
		beijing_fla=1;//������������ȱ�־λ
		HEAD_Low_OPEN();
	}	
 if(douji_flag==2)
	{
		beijing_fla=1;//������������ȱ�־λ
		HEAD_High_OPEN();
	}	
}
/**********************************************Sensorģʽ************************************************/


