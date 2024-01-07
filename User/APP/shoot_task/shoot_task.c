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
void shootPidCalc(void);
void shootBehInit(shoot_behaviour_t *initBeh, shoot_motor_mode_e flm, shoot_motor_mode_e frm, shoot_motor_mode_e plm, shoot_behaviour_e num, 
					void (*behaviorHandleFun)(float *leftFirExp, float *rightFirExp, float *shootHzSet), bool_t (*enterBehaviorCondition)(void),
					bool_t (*outBehaviorCondition)(void), void (*enterBehaviorFun)(void), void (*outBehaviorFun)(void));
					
	float view_shoot_left, view_shoot_right, view_shoot_plate; //��̨�۲�
	float *flExp,*frExp,*plExp;

//��������������
void shootTask(void *pvParameters)
{
	//������س�ʼ��
	shootInit();
	vTaskDelay(1000);
	TickType_t shootDelayTick = xTaskGetTickCount();
	while(1)
	{
		//��̨�۲�
		view_shoot_left = shootTaskStructure.leftFirMotor.speed;
		view_shoot_right = shootTaskStructure.rightFirMotor.speed;
		view_shoot_plate = shootTaskStructure.plateMotor.base_inf.real_speed_rpm;
		
		//ѡ��ǰ�����ģʽ
		shootBehaviourSelect();
		//�������Ʒ�ʽ�л�������0
		shootCtrlChangeHandle();
		//ʵ�����ݸ���
		shootFeedbackUpdata();

			plExp = &(shootTaskStructure.shootHzSet);
			flExp = &(shootTaskStructure.leftFirMotor.speedSet);
			frExp = &(shootTaskStructure.rightFirMotor.speedSet);

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
		                        shootTaskStructure. rightFirMotor.base_inf.given_current);
		
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
}


//ʵ��ֵ���º���
void shootFeedbackUpdata()
{
	//��Ħ����ת��rpmת��ΪĦ���ֱ����ٶȣ��������˲���
	shootTaskStructure.leftFirMotor.filterSpeed = KalmanFilter(&(shootTaskStructure.leftFirMotor.klmFiller), shootTaskStructure.leftFirMotor.base_inf.real_speed_rpm);
	shootTaskStructure.rightFirMotor.filterSpeed = KalmanFilter(&(shootTaskStructure.rightFirMotor.klmFiller), shootTaskStructure.rightFirMotor.base_inf.real_speed_rpm);
	shootTaskStructure.leftFirMotor.speed  = PI/1000.0*shootTaskStructure.leftFirMotor.filterSpeed;
	shootTaskStructure.rightFirMotor.speed = PI/1000.0*shootTaskStructure.rightFirMotor.filterSpeed;
}


//PID��ʼ������
void shootPidInit()
{
	//�����ٶȻ�&λ�û�
	PIDInit(&(shootTaskStructure.plateMotor.PIDParameter[INNER]), 130, 1.2, 0, 0, 10000, 50/36.0, 3100/36.0, -1, 5000, SPEED);
	PIDInit(&(shootTaskStructure.plateMotor.PIDParameter[OUTER]),   5, 0, 0, 0, -1, 0, -1, 360 , -1, POSITION_360);
	//����Ħ���ֵ���ٶȻ�
	PIDInit(&(shootTaskStructure.leftFirMotor.PIDParameter)	, 1500	, 5.00f, 4000	, 0, 16000, 0, 15, 29, 2000, SPEED);
	PIDInit(&(shootTaskStructure.rightFirMotor.PIDParameter), 1800	, 5.00f, 4000	, 0, 16000, 0, 15, 29, 2000, SPEED);
}


//��������ֵ �Ż�ת�����ӻ�
fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{ 
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

