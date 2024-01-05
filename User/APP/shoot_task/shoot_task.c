#include "shoot_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"
#include "Remote_control.h"
#include "can.h"
#include "user_lib.h"
#include "Detect_Task.h"
#include "robot.h"
#include "shoot_behaviour.h"
#include "gimbal_task.h"
#include "BoardB_link.h"
#include "delay.h"
#include "led.h"

/*�ṹ��ͺ�������*/
ShootCtrl_t shootTaskStructure;
void plateBackSpeed(float *speedSet, float speed, u16 frameMs);
void shootPidInit(void);
void shootPlateReset(void);
void shootBehaviourSelect(void);
void shootCtrlChangeHandle(void);
void shootFeedbackUpdata(void);
void shootHeatLimt(void);
void shootspeedlimit(void);
void shootsensorhandle(void);
void shootPidCalc(void);
void shootTuchuanProc(void);
void shootBehInit(shoot_behaviour_t *initBeh, shoot_motor_mode_e flm, shoot_motor_mode_e frm, shoot_motor_mode_e plm, shoot_behaviour_e num, 
					void (*behaviorHandleFun)(float *leftFirExp, float *rightFirExp, float *shootHzSet), bool_t (*enterBehaviorCondition)(void),
					bool_t (*outBehaviorCondition)(void), void (*enterBehaviorFun)(void), void (*outBehaviorFun)(void));
					
float l_speed_real,l_speed_set,r_speed_set,r_speed_real,bomb_real,l_speed_exp,r_speed_exp;
s16 PL_REAL,PL_EXP;		 //7060   4570	
s16 error_l_r;					
					
//����΢�����ص��ж�					
int sensorStatus = 0;
int bulletflag = 0;
int sFlagStatus;
int fla_jiance;
					
void EXTI15_10_IRQHandler()
{
	if (EXTI_GetITStatus(EXTI_Line12)== RESET)
  {	
		bulletflag = 1;				
		EXTI_ClearITPendingBit(EXTI_Line12);
	}
}
	
extern int singletime;
extern int singletimeflag;		
extern int Electricpushtimeflag;
extern int Electricpushtime;
extern uint8_t gongdan_fla;
extern int gongdan_tim;
extern uint8_t qiangzhi_gongdan_fla;
extern int qiangzhi_gongdan_tim;
extern uint8_t fan_zhuan;
extern int fan_zhuan_tim;
extern int tuigan_flag;
extern int tuigan_tim;
//��������������
void shootTask(void *pvParameters)
{
	//������س�ʼ��
	shootInit();
	vTaskDelay(1000);
	float *flExp,*frExp,*plExp;	
	TickType_t shootDelayTick = xTaskGetTickCount();
	while(1)
	{
		
		//Jscope�۲����
//		l_speed_exp =shootTaskStructure.leftFirMotor.speedSet;
//		r_speed_set =shootTaskStructure.rightFirMotor.speedSet;
//		l_speed_real=shootTaskStructure.leftFirMotor.speed;
		l_speed_real=shootTaskStructure.leftFirMotor.speed;
		r_speed_real=shootTaskStructure.rightFirMotor.speed;
		l_speed_exp=shootTaskStructure.leftFirMotor.speedSet;
		r_speed_exp=shootTaskStructure.rightFirMotor.speedSet;
		error_l_r=shootTaskStructure.leftFirMotor.speed+shootTaskStructure.rightFirMotor.speed;
		fla_jiance=READING_SENSOR();
		
				if(SHOOT_FIRST_PRESS_R)
	{
			__set_FAULTMASK(1);
			NVIC_SystemReset();
	}	
	
		if(singletimeflag == 1)
  	{
	    singletime ++ ;
	  }
		
		if(Electricpushtimeflag == 1)
		{
			Electricpushtime ++;
		}
		
		if(gongdan_fla==1)
		{
			gongdan_tim++;
		}
		//ǿ�ƹ���
		if(qiangzhi_gongdan_fla==1)
		{
			qiangzhi_gongdan_tim++;
		}
		//��ת
		if(fan_zhuan==1)
		{
			fan_zhuan_tim++;
		}	
		if(tuigan_flag==1)
		{
			tuigan_tim++;
		}
		
		//΢�����ش�������ش���
		shootsensorhandle();
//		shootTuchuanProc();
		//��������
		shootspeedlimit();
		//ѡ��ǰ�����ģʽ
		shootBehaviourSelect();
		//�������Ʒ�ʽ�л�������0
		shootCtrlChangeHandle();
		//ʵ�����ݸ���
		shootFeedbackUpdata();
	  //���ݲ���ģʽ�����̵����ֵ
		if(shootTaskStructure.plateMotor.mode == SHOOT_MOTOR_POSITION)
		{
			plExp = &(shootTaskStructure.plateMotor.position_set);
		}
		else if(shootTaskStructure.plateMotor.mode == SHOOT_MOTOR_SPEED)
		{
			plExp = &(shootTaskStructure.shootHzSet);
		}
		else if(shootTaskStructure.plateMotor.mode == SHOOT_MOTOR_RAW)
		{
			plExp = &(shootTaskStructure.plateMotor.rawCmdCurrent);
		}
		//��Ħ����ģʽѡ��
		if(shootTaskStructure.leftFirMotor.mode == SHOOT_MOTOR_SPEED)
		{
			flExp = &(shootTaskStructure.leftFirMotor.speedSet);
		}
		else if(shootTaskStructure.leftFirMotor.mode == SHOOT_MOTOR_RAW)
		{
			flExp = &(shootTaskStructure.leftFirMotor.rawCmdCurrent);
		}
		//��Ħ����ģʽѡ��
		if(shootTaskStructure.rightFirMotor.mode == SHOOT_MOTOR_SPEED)
		{
			frExp = &(shootTaskStructure.rightFirMotor.speedSet);
		}
		else if(shootTaskStructure.rightFirMotor.mode == SHOOT_MOTOR_RAW)
		{
			frExp = &(shootTaskStructure.rightFirMotor.rawCmdCurrent);
		}
		//����������ֵ
		shootTaskStructure.nowBeh->behaviorHandleFun(flExp, frExp, plExp);
		//����ң��������
		rcDataCopy(&(shootTaskStructure.rc.last));
		//��������
		shootHeatLimt();
		//PID����
		shootPidCalc();	
		
		//������н��з���
		GIMBAL_CanbusCtrlMotors(gimbalTaskStructure.pitchMotor.base_inf.given_current, 
														-gimbalTaskStructure.yawMotor.  base_inf.given_current,
														shootTaskStructure. plateMotor.base_inf.given_current,
		                        shootTaskStructure. leftFirMotor.base_inf.given_current, 
		                        shootTaskStructure. rightFirMotor.base_inf.given_current);	// �������ȼ����ߣ�ת������������
		//ʱ����ʱ
//		vTaskDelay(SHOOT_TASK_MS);
		vTaskDelayUntil(&shootDelayTick, SHOOT_TASK_MS);
	}
}

//���������ʼ��
void shootInit()
{
	//������ز�����ʼ��
	shootPlateReset();
	//��ȡң������ֵ
	shootTaskStructure.rc.now = get_remote_control_point();
	//������ٱ�����
	motorInit(&(shootTaskStructure.leftFirMotor.base_inf),  1);
	motorInit(&(shootTaskStructure.rightFirMotor.base_inf), 1);
	motorInit(&(shootTaskStructure.plateMotor.base_inf),   19);
	KalmanCreate(&(shootTaskStructure.leftFirMotor.klmFiller), 1, 5);
	KalmanCreate(&(shootTaskStructure.rightFirMotor.klmFiller), 1, 5);
	
	//���̷�����ز���
	shootTaskStructure.plateMotor.teethNum = 6;
	shootTaskStructure.plateMotor.shootcontinue = 1;
	shootTaskStructure.plateMotor.ifSensorOnLine=1;
	//����Ħ���ְ뾶����
	shootTaskStructure.rightFirMotor.radius = 30;
	shootTaskStructure.leftFirMotor.radius  = 30;
	//PID��ʼ��
	shootPidInit();
	HEAD_CLOSE();
	Electricpush2_off();
	Electricpush_on();
	//��Ϊ��ʼ��
	shootBehInit(shootTaskStructure.behList + SHOOT_DEBUG, DEBUG_SHOOT_LF_TYPE, DEBUG_SHOOT_RF_TYPE, DEBUG_SHOOT_PL_TYPE, SHOOT_DEBUG, shootbehDebugHandleFun, shootbehDebugEnterCondition, shootbehDebugOutCondition, NULL, NULL);
	shootBehInit(shootTaskStructure.behList + SHOOT_ZERO_FORCE, SHOOT_MOTOR_RAW, SHOOT_MOTOR_RAW, SHOOT_MOTOR_RAW, SHOOT_ZERO_FORCE, shootbehZeroForceHandleFun, shootbehZeroForceEnterCondition, shootbehZeroForceOutCondition, NULL, NULL);
	shootBehInit(shootTaskStructure.behList + SHOOT_SENSOR, SHOOT_MOTOR_SPEED, SHOOT_MOTOR_SPEED, SHOOT_MOTOR_SPEED, SHOOT_SENSOR, shootbehSensorHandleFun, shootbehSensorEnterCondition, shootbehSensorOutCondition, NULL, NULL);
	//shootBehInit(shootTaskStructure.behList + SHOOT_POSITION, SHOOT_MOTOR_SPEED, SHOOT_MOTOR_SPEED, SHOOT_MOTOR_POSITION, SHOOT_POSITION, shootbehPositionHandleFun, shootbehPositionEnterCondition, shootbehPositionOutCondition, NULL, NULL);	
//	shootBehInit(shootTaskStructure.behList + SHOOT_FIR_OFF, SHOOT_MOTOR_SPEED, SHOOT_MOTOR_SPEED, SHOOT_MOTOR_SPEED, SHOOT_FIR_OFF, shootbehFirOffHandleFun, shootbehFirOffEnterCondition, shootbehFirOffOutCondition, NULL, NULL);		
	//Ĭ������ģʽ
	shootBehChange(shootTaskStructure.behList + SHOOT_ZERO_FORCE);
}



//������ת������û̫����
void plateBackSpeed(float *speedSet, float speed, u16 frameMs)
{
	static u16 shoot_err_time = 0;
	static u16 shoot_back_time = 0;
	static float last_Speed;
	
	if(f_abs(*speedSet) > 3 &&  f_abs(speed) < f_abs(*speedSet)/3 && shoot_back_time == 0)
		shoot_err_time++;
	else if(shoot_err_time > 0 && shoot_back_time == 0)
		shoot_err_time--;
	
	if(shoot_err_time > 600/frameMs)
	{
		shoot_err_time = 0;
		shoot_back_time = 1600/SHOOT_TASK_MS;
		last_Speed = *speedSet;
	}
	
	if(shoot_back_time > 0)
	{
		*speedSet = 20;
		if(shoot_back_time == 1)
			*speedSet = last_Speed;
		
		shoot_back_time--;
	}
}


//ģʽ�л�����
void shootBehChange(shoot_behaviour_t *next)
{
	if(shootTaskStructure.nowBeh != NULL && shootTaskStructure.nowBeh->outBehaviorFun != NULL)
		shootTaskStructure.nowBeh->outBehaviorFun();

	shootTaskStructure.nowBeh = next;
	shootTaskStructure.plateMotor.mode = next->plateMode;
	shootTaskStructure.leftFirMotor.mode = next->firLeftMode;
	shootTaskStructure.rightFirMotor.mode = next->firRightMode;
	shootTaskStructure.shootNumSet = 0;
	
	shootPlateReset();
	
	if(next->enterBehaviorFun != NULL)
		next->enterBehaviorFun();
}


//������������
void shootsensorhandle(void)
{
	static int sensorTime = 0;  //��������ʱ����
	
	//��ȡ΢�����صĵ�ƽ
	sensorStatus = READING_SENSOR();   
	//���жϷ���֮��ʼ��ʱ
	if(sensorStatus == 0)
	{ 
		sensorTime += 2;
		//����Ҫ��ʱ����Ϊ��������ƽ״̬������ʱ�����ǲ��ȶ��ģ�ԭ�������밴������
		if(sensorTime == 8)
		{ 
			shootTaskStructure.shoot_sensor.shootPlateflag = 1;
			shootTaskStructure.shoot_sensor.Projectile_if_Prepare = 1;
			sensorTime = 0;
		}
	}	
	else if(sensorStatus == 1)
	{
		shootTaskStructure.shoot_sensor.shootPlateflag = 0;
	}
}


//ģʽ�䱸����
void shootBehInit(shoot_behaviour_t *initBeh, shoot_motor_mode_e flm, shoot_motor_mode_e frm, shoot_motor_mode_e plm, shoot_behaviour_e num, 
					void (*behaviorHandleFun)(float *leftFirExp, float *rightFirExp, float *shootHzSet), bool_t (*enterBehaviorCondition)(void),
					bool_t (*outBehaviorCondition)(void), void (*enterBehaviorFun)(void), void (*outBehaviorFun)(void))
{
	if(enterBehaviorCondition == NULL || outBehaviorCondition == NULL || behaviorHandleFun == NULL)
		return;
	
	initBeh->behaviorHandleFun = behaviorHandleFun;
	initBeh->enterBehaviorCondition = enterBehaviorCondition;
	initBeh->enterBehaviorFun = enterBehaviorFun;
	initBeh->firLeftMode = flm;
	initBeh->firRightMode = frm;
	initBeh->plateMode = plm;
	initBeh->num = num;
	initBeh->outBehaviorCondition = outBehaviorCondition;
	initBeh->outBehaviorFun = outBehaviorFun;
}


//������ز�����ʼ������
void shootPlateReset()
{
	shootTaskStructure.shootHzSet = 0;
	shootTaskStructure.plateMotor.speed_set = 0;
	shootTaskStructure.plateMotor.position_set = shootTaskStructure.plateMotor.base_inf.real_ecd;
	shootTaskStructure.plateMotor.rawCmdCurrent = 0;
	shootTaskStructure.shootNumSet = 0;
	shootTaskStructure.shoot_sensor.lastSensorStatus=BULLET_OFF;
}


//ʵ��ֵ���º���
void shootFeedbackUpdata()
{
	//��Ħ����ת��rpmת��ΪĦ���ֱ����ٶ�
//	shootTaskStructure.leftFirMotor.speed  = PI/1000.0*shootTaskStructure.leftFirMotor.base_inf.real_speed_rpm;
//	shootTaskStructure.rightFirMotor.speed = PI/1000.0*shootTaskStructure.rightFirMotor.base_inf.real_speed_rpm;
	shootTaskStructure.leftFirMotor.filterSpeed = KalmanFilter(&(shootTaskStructure.leftFirMotor.klmFiller), shootTaskStructure.leftFirMotor.base_inf.real_speed_rpm);
	shootTaskStructure.rightFirMotor.filterSpeed = KalmanFilter(&(shootTaskStructure.rightFirMotor.klmFiller), shootTaskStructure.rightFirMotor.base_inf.real_speed_rpm);
	shootTaskStructure.leftFirMotor.speed  = PI/1000.0*shootTaskStructure.leftFirMotor.filterSpeed;
	shootTaskStructure.rightFirMotor.speed = PI/1000.0*shootTaskStructure.rightFirMotor.filterSpeed;
	//��������������ݣ�ûд
}


//PID��ʼ������
void shootPidInit()
{
	//�����ٶȻ�&λ�û�
	PIDInit(&(shootTaskStructure.plateMotor.PIDParameter[INNER]), 130, 1.2, 0, 0, 10000, 50/36.0, 3100/36.0, -1, 5000, SPEED);
	PIDInit(&(shootTaskStructure.plateMotor.PIDParameter[OUTER]),   5, 0, 0, 0, -1, 0, -1, 360 , -1, POSITION_360);
	//����Ħ���ֵ���ٶȻ�
//	PIDInit(&(shootTaskStructure.leftFirMotor.PIDParameter)	, 900	, 2.00f, 2400	, 0, 12000, 0, 15, 29, 2000, SPEED);//450 2 100 (iband 9)
//	PIDInit(&(shootTaskStructure.rightFirMotor.PIDParameter), 1200, 2.50f, 2400	, 0, 12000, 0, 15, 29, 2000, SPEED);//390 2 75 (iband 9)
	PIDInit(&(shootTaskStructure.leftFirMotor.PIDParameter)	, 1500	, 5.00f, 4000	, 0, 16000, 0, 15, 29, 2000, SPEED);
	PIDInit(&(shootTaskStructure.rightFirMotor.PIDParameter), 1800	, 5.00f, 4000	, 0, 16000, 0, 15, 29, 2000, SPEED);
}


//�Ƕ�ת������
fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{ //�����ת��Ϊ��С���Ƿ���������� ��ǰʵ��ֵΪ8190������ֵΪ1������������Ľ����8189������ʵ�ʵĲ��Ӧ����-2   ����
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > 4096)
    {
        relative_ecd -= 8191;
    }
    else if (relative_ecd < -4096)
    {
        relative_ecd += 8191;
    }

    return relative_ecd * 0.04395068f;
}


//PID�ļ��㺯��
void shootPidCalc()
{
	//����1800��ʲô��˼
	float realPos = motor_ecd_to_angle_change(shootTaskStructure.plateMotor.base_inf.real_ecd,1800);
	float expPos  = motor_ecd_to_angle_change(shootTaskStructure.plateMotor.position_set,     1800);
	//���̵��λ�û�����
	if(shootTaskStructure.plateMotor.mode == SHOOT_MOTOR_POSITION)
	{
		//λ�û�����
		PID_Calc(shootTaskStructure.plateMotor.PIDParameter + OUTER, realPos, expPos);
		//λ�û������ֵ
		shootTaskStructure.plateMotor.speed_set = shootTaskStructure.plateMotor.PIDParameter[OUTER].output;
		//������ת
		plateBackSpeed(&(shootTaskStructure.plateMotor.speed_set), shootTaskStructure.plateMotor.base_inf.real_speed_rpm, SHOOT_TASK_MS);
		//�ٶȻ�����
		PID_Calc(shootTaskStructure.plateMotor.PIDParameter + INNER, shootTaskStructure.plateMotor.base_inf.real_speed_rpm, shootTaskStructure.plateMotor.speed_set);
		//�ٶȻ������ֵ
		shootTaskStructure.plateMotor.currentSet = shootTaskStructure.plateMotor.PIDParameter[INNER].output;
	}
	//���̵���ٶȻ�����
	else if(shootTaskStructure.plateMotor.mode == SHOOT_MOTOR_SPEED)
	{
		//��Ƶת��Ϊ����ת��
		shootTaskStructure.plateMotor.speed_set = shootTaskStructure.shootHzSet*60/shootTaskStructure.plateMotor.teethNum;
		//������ת
		plateBackSpeed(&(shootTaskStructure.plateMotor.speed_set), shootTaskStructure.plateMotor.base_inf.real_speed_rpm, SHOOT_TASK_MS);
		//�ٶȻ�����
		PID_Calc(shootTaskStructure.plateMotor.PIDParameter + INNER, shootTaskStructure.plateMotor.base_inf.real_speed_rpm, shootTaskStructure.plateMotor.speed_set);
		//�ٶȻ������ֵ
		shootTaskStructure.plateMotor.currentSet = shootTaskStructure.plateMotor.PIDParameter[INNER].output;		
	}
	//����ģʽ
	else if(shootTaskStructure.plateMotor.mode == SHOOT_MOTOR_RAW)
	{
		shootTaskStructure.plateMotor.currentSet = shootTaskStructure.plateMotor.rawCmdCurrent;
	}
	//���Ʋ��̵���������
	abs_limit(&(shootTaskStructure.plateMotor.currentSet), 10000);
	shootTaskStructure.plateMotor.base_inf.given_current = (s16)shootTaskStructure.plateMotor.currentSet;
	//Ħ�������ٶ�PID����
	if(shootTaskStructure.leftFirMotor.mode == SHOOT_MOTOR_SPEED)
	{
		PID_Calc(&(shootTaskStructure.leftFirMotor.PIDParameter),shootTaskStructure.leftFirMotor.speed, shootTaskStructure.leftFirMotor.speedSet);
		shootTaskStructure.leftFirMotor.currentSet = shootTaskStructure.leftFirMotor.PIDParameter.output;
	}		
	else if(shootTaskStructure.leftFirMotor.mode == SHOOT_MOTOR_RAW)
		shootTaskStructure.leftFirMotor.currentSet = shootTaskStructure.leftFirMotor.rawCmdCurrent;	
	
	if(shootTaskStructure.rightFirMotor.mode == SHOOT_MOTOR_SPEED)
	{
		PID_Calc(&(shootTaskStructure.rightFirMotor.PIDParameter), shootTaskStructure.rightFirMotor.speed, shootTaskStructure.rightFirMotor.speedSet);	
		shootTaskStructure.rightFirMotor.currentSet = shootTaskStructure.rightFirMotor.PIDParameter.output;
	}
	else if(shootTaskStructure.rightFirMotor.mode == SHOOT_MOTOR_RAW)
		shootTaskStructure.rightFirMotor.currentSet = shootTaskStructure.rightFirMotor.rawCmdCurrent;
	//���Ƶ���ֵ
	abs_limit(&(shootTaskStructure.rightFirMotor.currentSet), 16384);
	abs_limit(&(shootTaskStructure.leftFirMotor.currentSet),  16384);
  //����ֵ��ֵ
	shootTaskStructure.rightFirMotor.base_inf.given_current = (s16)shootTaskStructure.rightFirMotor.currentSet;	
	shootTaskStructure.leftFirMotor.base_inf.given_current  = (s16)shootTaskStructure.leftFirMotor.currentSet;
}


//ģʽ�л�����
void shootBehaviourSelect()
{
	for(shoot_behaviour_t *iterator = shootTaskStructure.behList; iterator < shootTaskStructure.nowBeh; iterator++)
	{ //�鿴���ȼ��ȵ�ǰ��Ϊ�ߵ���Ϊ�Ľ��������Ƿ����㣬���������
		if(iterator->enterBehaviorCondition != NULL && iterator->enterBehaviorCondition())
		{
			shootBehChange(iterator);
			return;
		}
	}
	if(shootTaskStructure.nowBeh->outBehaviorCondition != NULL && shootTaskStructure.nowBeh->outBehaviorCondition())
	{ //�����ǰ��Ϊ�ﵽ�˳���������Ѱ�����������������Ϊ��������
		for(shoot_behaviour_t *iterator = shootTaskStructure.nowBeh + 1; iterator < shootTaskStructure.behList + SHOOT_MODE_LENGTH; iterator++)
		{
			if(iterator->enterBehaviorCondition != NULL && iterator->enterBehaviorCondition())
			{
				shootBehChange(iterator);
				return;
			}
		}
		//�������behaviour�Ľ���������û�дﵽ�ͽ�������ģʽ
		shootBehChange(shootTaskStructure.behList + SHOOT_ZERO_FORCE);
	}
}


//ģʽ�л��Ĵ�����
void shootCtrlChangeHandle()
{
	static shoot_motor_mode_e lastPlateMode;
	
	if(lastPlateMode != shootTaskStructure.plateMotor.mode)  //�����ϴε�ģʽ����ε�ģʽ��ͬ
	{
		if(shootTaskStructure.plateMotor.mode == SHOOT_MOTOR_POSITION)
		{
			shootTaskStructure.plateMotor.position_set = shootTaskStructure.plateMotor.base_inf.real_ecd;  //�����ڵı�����ֵ�������ֵ
		}
		else if(shootTaskStructure.plateMotor.mode == SHOOT_MOTOR_SPEED)
		{
			shootTaskStructure.shootHzSet = 0;   
			shootTaskStructure.plateMotor.speed_set = 0;			
		}
		else if(shootTaskStructure.plateMotor.mode == SHOOT_MOTOR_RAW)
		{
			shootTaskStructure.plateMotor.rawCmdCurrent = 0;
		}
		shootTaskStructure.shootNumSet = 0;
		
		lastPlateMode = shootTaskStructure.plateMotor.mode;
	}
}
  

//�������ƺ���
void shootHeatLimt()
{
	switch(BoardBLink.RefreeSyetem.robot.u8_level)
	{
		case 0:
		case 1:
					shootTaskStructure.heatLimit.maxHeat=MAX_HEAT_LEVEL1;
					break;
		case 2:
					shootTaskStructure.heatLimit.maxHeat=MAX_HEAT_LEVEL2;
					break;
		case 3:
					shootTaskStructure.heatLimit.maxHeat=MAX_HEAT_LEVEL3;
					break;
	}
	shootTaskStructure.heatLimit.heatNow=BoardBLink.RefreeSyetem.shoot.s16_Shoot_heat;//��ǰ����
	if(shootTaskStructure.heatLimit.maxHeat-shootTaskStructure.heatLimit.heatNow<100)
  {			 
				shootTaskStructure.plateMotor.position_set= shootTaskStructure.plateMotor.base_inf.real_ecd;//position������ֵΪ��ǰֵ	
	}
}


//�������ƺ���
static int shoot_speed_change=0;
static int choice_mode=0;

void shootspeedlimit(void)
{
    switch(BoardBLink.RefreeSyetem.robot.u8_level)
	  {
		case 1:
					shootTaskStructure.shootspeed_set=SHOOT_SPEED_LOW;
					break;
		case 2:
					shootTaskStructure.shootspeed_set=SHOOT_SPEED_MID;
					break;
		case 3:
					shootTaskStructure.shootspeed_set=SHOOT_SPEED_HIGH;
					break;
   	}
}


//void shootTuchuanProc(void)
//{
//	if(IF_KEY_PRESSED_Q)
//	{
//		Electricpush2_flag ++ ;
//	}
//	else if(IF_MOUSE_PRESSED_RIGH)
//	{
//		douji_flag ++ ;
//	}	
//	
//	if((Electricpush2_flag%2)==0)
//	{
//		Electricpush2_off();
//	}
//	else if((Electricpush2_flag%2)==0)
//	{
//		Electricpush2_on();
//	}
//	
//	if((douji_flag%2)==0)
//	{
//		HEAD_CLOSE();
//	}
//	else if((douji_flag%2)==0)
//	{
//		HEAD_OPEN();
//	}	
//}





