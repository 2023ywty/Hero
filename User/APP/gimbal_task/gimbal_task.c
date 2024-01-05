#include "gimbal_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"
#include "remote_control.h"
#include "user_lib.h"
#include "can.h"
#include "Detect_Task.h"
#include "pid.h"
#include "motor.h"
#include "oscillography.h"
#include "robot.h"
#include "gimbal_behaviour.h"
#include "shoot_task.h"
#include "BoardB_link.h"


/*�ṹ���ʼ���ͺ�������*/
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
GimbalCtrl_t gimbalTaskStructure;
motor_speed_calc_t gimbalPitMotorSpeedCalc;
motor_speed_calc_t gimbalLeftPitMotorSpeedCalc;
motor_speed_calc_t gimbalYawMotorSpeedCalc;
void gimbalInit();
void gimbalPidInit();
void gimbalBehaviourSelect();
void gimbalMotorCtrlChangeHandle();
void gimbalFeedbackUpdate(GimbalCtrl_t *gimbal_feedback_update);
void gimbalAngleLimit();
void GimbalPidCalc(gimbal_motor_mode_e pitchCtrlType, gimbal_motor_mode_e yawCtrlType, gimbal_motor_t *pitchMotor, gimbal_motor_t *yawMotor);
void GIMBAL_CanbusCtrlMotors(s16 pitchCurrent, s16 yawCurrent, s16 plCurrent,s16 flCurrent,s16 frCurrent);
void gimbalBehaviourInit(gimbal_behaviour_t *gbh, gimbal_motor_mode_e pitchMode, gimbal_motor_mode_e yawMode, void (*behaviorHandleFun)(float *yawExp, float *pitExp),
							bool_t (*enterBehaviorCondition)(void), bool_t (*outBehaviorCondition)(void), void (*enterBehaviorFun)(void), void (*outBehaviorFun)(void));

int Pit_exp_speed,Pit_real_speed,Pit_exp_angle,Pit_real_angle;
int Yaw_exp_speed,Yaw_real_speed,Yaw_exp_angle,Yaw_real_angle;
int pit_postion,yaw_postion;
int pitch,yaw,roll;
	


//�������ƣ�gimbalTask()
//�������ã���̨����������
//��ڲ�������
//����  ֵ����
void gimbalTask(void *pvParameters)
{
	portTickType currentTime;
	//��̨��ʼ��
	gimbalInit();
	//FreeRTOS����ϵͳ�����ʱ������Ϊ���������ʼ���ṩʱ��
	vTaskDelay(2);
	fp32 *pitchAngleSet, *yawAngleSet;
	u8    pitchCtrlType,  yawCtrlType;
	//��ȡ��ǰʱ�䣬�ṩ��������ʱ������
	currentTime = xTaskGetTickCount();
	while(1)
	{
		//��ȡ����ֵ����ʵֵ������Jscope�������
//		Yaw_exp_speed = gimbalTaskStructure.yawMotor.speed_set[GYRO];
//		Yaw_real_speed = gimbalTaskStructure.yawMotor.speed[GYRO];
		Yaw_exp_angle = gimbalTaskStructure.yawMotor.angle_set[GYRO];
		Yaw_real_angle = gimbalTaskStructure.yawMotor.angle[GYRO];
		Pit_exp_speed   = gimbalTaskStructure.pitchMotor.angle_set[ENCONDE] ;
		Pit_real_speed  = gimbalTaskStructure.pitchMotor.angle[ENCONDE] ;
		Pit_real_angle=gimbalTaskStructure.pitchMotor.angle[GYRO];
		yaw_postion=gimbalTaskStructure.pitchMotor.base_inf.real_ecd;
//		//��B����UI���棬ͨ��B�巢����
//		if(IF_KEY_PRESSED_B)     
//		   Restart_UsrInterFace();
		//ѭ������ģʽ�л��������ȵ��õ�ǰģʽ
		gimbalBehaviourSelect();
	  //���ģʽ����
		gimbalMotorCtrlChangeHandle();
		//����ʵ��ֵ����
		gimbalFeedbackUpdate(&gimbalTaskStructure);
		//��������ֵ
		pitchCtrlType = gimbalTaskStructure.behaviorNow->motorPitCtrlType;
		yawCtrlType   = gimbalTaskStructure.behaviorNow->motorYawCtrlType;
		//��yaw���pitch����������ֵ��ֵ
//		if(yawCtrlType == GIMBAL_MOTOR_ENCONDE || yawCtrlType == GIMBAL_MOTOR_GYRO)
//			yawAngleSet = gimbalTaskStructure.yawMotor.angle_set + gimbalTaskStructure.behaviorNow->motorYawCtrlType;
			yawAngleSet = gimbalTaskStructure.yawMotor.angle_set + yawCtrlType;
//		else if(yawCtrlType == GIMBAL_MOTOR_RAW)
//			yawAngleSet = &(gimbalTaskStructure.yawMotor.rawCmdCurrent);
//		else if(yawCtrlType == GIMBAL_MOTOR_ENCONDE_SPEED)
//			yawAngleSet = gimbalTaskStructure.yawMotor.speed_set + ENCONDE;
//		else if(yawCtrlType == GIMBAL_MOTOR_GYRO_SPEED)
//			yawAngleSet = gimbalTaskStructure.yawMotor.speed_set + GYRO;
		
		if(pitchCtrlType == GIMBAL_MOTOR_ENCONDE || pitchCtrlType == GIMBAL_MOTOR_GYRO)
			pitchAngleSet = gimbalTaskStructure.pitchMotor.angle_set + gimbalTaskStructure.behaviorNow->motorPitCtrlType;
		else if(pitchCtrlType == GIMBAL_MOTOR_RAW)
			pitchAngleSet = &(gimbalTaskStructure.pitchMotor.rawCmdCurrent);
		else if(pitchCtrlType == GIMBAL_MOTOR_ENCONDE_SPEED)
			pitchAngleSet = gimbalTaskStructure.pitchMotor.speed_set + ENCONDE;
		else if(pitchCtrlType == GIMBAL_MOTOR_GYRO_SPEED)
			pitchAngleSet = gimbalTaskStructure.pitchMotor.speed_set + GYRO;
		//����������ֵ����ֵ
		gimbalTaskStructure.behaviorNow->behaviorHandleFun(yawAngleSet, pitchAngleSet);
		//��λ
		gimbalAngleLimit();
		//����ң����last
		rcDataCopy(&(gimbalTaskStructure.rc.last));
		//pid����
		GimbalPidCalc(pitchCtrlType, yawCtrlType, &(gimbalTaskStructure.pitchMotor), &(gimbalTaskStructure.yawMotor));
//		//������н��з���
//		GIMBAL_CanbusCtrlMotors(gimbalTaskStructure.pitchMotor.base_inf.given_current, 
//														-gimbalTaskStructure.yawMotor.  base_inf.given_curbehaviorHandleFunrent,
//														shootTaskStructure. plateMotor.base_inf.given_current,
//		                        shootTaskStructure. leftFirMotor.base_inf.given_current, 
//		                        shootTaskStructure. rightFirMotor.base_inf.given_current);	// �������ȼ����ߣ�ת������������
		//������ʱ
		vTaskDelayUntil(&currentTime, GIMBAL_TASK_MS);
		}
}



void gimbalInit()
{
	//��ȡ�����ǵĽǶ�ֵ
	gimbalTaskStructure.imu = &imu_;
	//�õ������˵ĵ�ǰģʽ�ͳ�ʼ��״̬
	gimbalTaskStructure.robotMode     = &(robotInf.robotMode);
	gimbalTaskStructure.robotModeStep = &(robotInf.modeStep);
	//��ȡң�����ĵ�ǰֵ����һ�ε�ֵ
	gimbalTaskStructure.rc.now = get_remote_control_point();
	rcDataCopy(&(gimbalTaskStructure.rc.last));
	//������
	motorSpeedCalcInit(&gimbalPitMotorSpeedCalc, &(gimbalTaskStructure.pitchMotor.base_inf), 0, MOTOR_ECD_FEEDBACL_RANGE, xTaskGetTickCount(), 8);
	motorSpeedCalcInit(&gimbalYawMotorSpeedCalc, &(gimbalTaskStructure.yawMotor.base_inf),   0, MOTOR_ECD_FEEDBACL_RANGE, xTaskGetTickCount(), 8);	
	//pitch��̨�����ʼ��(6020)
	gimbalTaskStructure.pitchMotor.mode = GIMBAL_MOTOR_RAW;
	gimbalTaskStructure.pitchMotor.lastMode = GIMBAL_MOTOR_RAW;//Ĭ��ֱ�ӷ�����ģʽ
	gimbalTaskStructure.pitchMotor.offsetEcd = GIMBAL_PITCH_MOTOR_OFFSET_ECD;  //�ϵ����ֵ����������
	gimbalTaskStructure.pitchMotor.maxRelativeAngle = RELATIVE_PITCH_ANGLE_MAX;//���ĽǶȣ������ǣ�
	gimbalTaskStructure.pitchMotor.minRelativeAngle = RELATIVE_PITCH_ANGLE_MIN;//��С�ĽǶȣ������ǣ�
	//yaw��̨�����ʼ��(6020)
	gimbalTaskStructure.yawMotor.mode = GIMBAL_MOTOR_RAW;
	gimbalTaskStructure.yawMotor.lastMode = GIMBAL_MOTOR_RAW;//Ĭ��ֱ�ӷ�����ģʽ
	gimbalTaskStructure.yawMotor.offsetEcd = GIMBAL_YAW_MOTOR_OFFSET_ECD;  //�ϵ����ֵ����������    
	gimbalTaskStructure.yawMotor.maxRelativeAngle = RELATIVE_YAW_MAX_RANGE;//���ĽǶȣ������ǣ�
	gimbalTaskStructure.yawMotor.minRelativeAngle = RELATIVE_YAW_MIN_RANGE;//��С�ĽǶȣ������ǣ�
  //��̨ʵ��ֵ����
	gimbalFeedbackUpdate(&gimbalTaskStructure);
	//��̨����ֵ����(��ʼ��Ϊʵ��ֵ)
	gimbalTaskStructure.yawMotor.angle_set[ENCONDE]   = gimbalTaskStructure.yawMotor.angle[ENCONDE];
	gimbalTaskStructure.yawMotor.angle_set[GYRO]      = gimbalTaskStructure.yawMotor.angle[GYRO];
	gimbalTaskStructure.pitchMotor.angle_set[ENCONDE] = gimbalTaskStructure.pitchMotor.angle[ENCONDE];
	gimbalTaskStructure.pitchMotor.angle_set[GYRO]    = gimbalTaskStructure.pitchMotor.angle[GYRO];
  //�Ƿ���DEBUGģʽ
	#ifdef DEBUG_GIMBAL
	gimbalBehaviourInit(gimbalTaskStructure.behaviorList + GIMBAL_DEBUG, DEBUG_GIMBAL_PITCH_TYPE, DEBUG_GIMBAL_YAW_TYPE, gimbalDebugBehaviourHandleFun, gimbalDebugBehaviourEnterCondition, gimbalDebugBehaviourOutCondition, NULL, NULL);
	#endif
	//�Ƿ���MID_GIMBALģʽ
	#ifdef MID_GIMBAL
	gimbalBehaviourInit(gimbalTaskStructure.behaviorList + GIMBAL_MID, MID_GIMBAL_PITCH_TYPE, MID_GIMBAL_YAW_TYPE, gimbalMidBehaviourHandleFun, gimbalMidBehaviourEnterCondition, gimbalMidBehaviourOutCondition, NULL, NULL);
	#endif
	//��̨���PID��ʼ��
	gimbalPidInit();
	/*���鿨�����˲�,���׳�ʼ��*/
	/*****������ģʽ�䱸������*****/
	//����״̬
	gimbalBehaviourInit(gimbalTaskStructure.behaviorList + GIMBAL_ZERO_FORCE, GIMBAL_MOTOR_RAW, GIMBAL_MOTOR_RAW, gimbalZeroForceBehaviourHandleFun, gimbalZeroForceBehaviourEnterCondition, gimbalZeroForceBehaviourOutCondition, NULL, NULL);
	//��ʼ״̬����
	gimbalBehaviourInit(gimbalTaskStructure.behaviorList + GIMBAL_INIT, GIMBAL_MOTOR_ENCONDE, GIMBAL_MOTOR_ENCONDE, gimbalInitBehaviourHandleFun, gimbalInitBehaviourEnterCondition, gimbalInitBehaviourOutCondition, NULL, NULL);
	//��ͨ����ģʽ����
	gimbalBehaviourInit(gimbalTaskStructure.behaviorList + GIMBAL_NORMAL, GIMBAL_MOTOR_GYRO, GIMBAL_MOTOR_GYRO, gimbalNormalBehaviourHandleFun, gimbalNormalBehaviourEnterCondition, gimbalNormalBehaviourOutCondition, NULL, NULL);
//	//����ģʽ
//	gimbalBehaviourInit(gimbalTaskStructure.behaviorList + GIMBAL_Dropshot, GIMBAL_MOTOR_ENCONDE, GIMBAL_MOTOR_ENCONDE, gimbalDropshotBehaviourHandleFun, gimbalDropshotBehaviourEnterCondition, gimbalADropshotBehaviourOutCondition, NULL, NULL);
//	//����
//	gimbalBehaviourInit(gimbalTaskStructure.behaviorList + AUTO, GIMBAL_MOTOR_GYRO, GIMBAL_MOTOR_GYRO, gimbalAutoBehaviourHandleFun, gimbalAutoBehaviourEnterCondition, gimbalAutoBehaviourOutCondition, NULL, NULL);
  //Ĭ��Ϊ����ģʽ
	gimbalTaskStructure.behaviorNow = gimbalTaskStructure.behaviorList + GIMBAL_ZERO_FORCE;
}


//�������ƣ�gimbalPidInit()
//�������ã�������̨���������PID��ʼ��
//��ڲ�������
//����  ֵ����
void gimbalPidInit()
{
//void PIDInit(PidTypeDef *pid,float kp,float ki,float kd,float ka,float max_out,float dead_band,float i_band,float max_input, float i_maxout, pid_mode_e model)
  /****Yaw����****/
//	PIDInit(*(gimbalTaskStructure.yawMotor.PIDParameterList + GYRO) + INNER		, 600,2.5, 0, 0, 30000, 0, 1000, 1000, 10000, SPEED);
//	PIDInit(*(gimbalTaskStructure.yawMotor.PIDParameterList + GYRO) + OUTER		, 12, 0, 1.5, 0, 10000, 0, 0, 180, 0, POSITION_180);
	PIDInit(*(gimbalTaskStructure.yawMotor.PIDParameterList + GYRO) + INNER		, 500, 2.5, 250, 0, 30000, 0, 1000, 1000, 10000, SPEED);
	PIDInit(*(gimbalTaskStructure.yawMotor.PIDParameterList + GYRO) + OUTER		, 4, 0.5,8, 0, 500, 0, 0, 180, 0, POSITION_180);
	
	PIDInit(*(gimbalTaskStructure.yawMotor.PIDParameterList + ENCONDE) + INNER, 400, 2.5, 0, 0, 30000, 0, 300, 10000, 10000, SPEED);
	PIDInit(*(gimbalTaskStructure.yawMotor.PIDParameterList + ENCONDE) + OUTER, 5, 0, 2, 0, 500, 0, 0, 180, 0, POSITION_180);
  /****Pitch����****/
	PIDInit(*(gimbalTaskStructure.pitchMotor.PIDParameterList + GYRO) + INNER, 5000, 100, 1000, 0, 30000, 0, 2, 5, 10000, SPEED);
	PIDInit(*(gimbalTaskStructure.pitchMotor.PIDParameterList + GYRO) + OUTER, 0.2, 0.05, 0.8, 0, 5, 0, 0, 180, 0, POSITION_180);
	
	PIDInit(*(gimbalTaskStructure.pitchMotor.PIDParameterList + ENCONDE) + INNER, 150, 0.5, 0, 0, 30000, 0, 2000, 2000, 13000, SPEED); //150 0.5 200
	PIDInit(*(gimbalTaskStructure.pitchMotor.PIDParameterList + ENCONDE) + OUTER, 20, 0.2, 35, 0, 100, 0, 0, 180, 0, POSITION_180);
}


void gimbalBehaviourInit(gimbal_behaviour_t *gbh, gimbal_motor_mode_e pitchMode, gimbal_motor_mode_e yawMode, void (*behaviorHandleFun)(float *yawExp, float *pitExp),
							bool_t (*enterBehaviorCondition)(void), bool_t (*outBehaviorCondition)(void), void (*enterBehaviorFun)(void), void (*outBehaviorFun)(void))
{
	if(behaviorHandleFun == NULL || enterBehaviorCondition == NULL || outBehaviorCondition == NULL)
	{
		return;
	}
	gbh->behaviorHandleFun = behaviorHandleFun;
	gbh->enterBehaviorCondition = enterBehaviorCondition;
	gbh->outBehaviorCondition = outBehaviorCondition;
	gbh->enterBehaviorFun = enterBehaviorFun;
	gbh->outBehaviorFun = outBehaviorFun;
	
	gbh->motorPitCtrlType = pitchMode;
	gbh->motorYawCtrlType = yawMode;
}


//�������ƣ�gimbalBehaviourSelect()
//�������ã��л���̨��ģʽ
//��ڲ�������
//����  ֵ����
void gimbalBehaviourSelect()
{
	for(gimbal_behaviour_t *gbIterator = gimbalTaskStructure.behaviorList; 
		gbIterator != gimbalTaskStructure.behaviorNow && gbIterator != gimbalTaskStructure.behaviorList + GIMBAL_MODE_LENGTH;
		gbIterator++)
	{ //�ȱ����ȵ�ǰ���ȼ��ߵ�behaviour������������������������л�
			
		if(gbIterator->enterBehaviorCondition != NULL && gbIterator->enterBehaviorCondition())
		{ //�ﵽ����Ϊ���л�������������Ϊ�л�
			gimbalBehaviourChange(gbIterator);
			return;
		}
	}
		
	if(gimbalTaskStructure.behaviorNow->outBehaviorCondition != NULL && gimbalTaskStructure.behaviorNow->outBehaviorCondition())
	{ //������㵱ǰģʽ�˳������������˳���ǰģʽѡ��������ģʽ
		for(gimbal_behaviour_t *gbIterator = gimbalTaskStructure.behaviorNow;
			gbIterator != gimbalTaskStructure.behaviorList + GIMBAL_MODE_LENGTH;
			gbIterator++)
			if(gbIterator->enterBehaviorCondition != NULL && gbIterator->enterBehaviorCondition()){
				gimbalBehaviourChange(gbIterator);
				return;
			}
		
		//������е�������û�����㣬�Ǿ��л�������ģʽ
		gimbalBehaviourChange(gimbalTaskStructure.behaviorList + GIMBAL_ZERO_FORCE);
	}	
}


void gimbalMotorCtrlChangeHandle()
{
	static u8 pitchLastMode = 255;
	static u8 yawLastMode = 255;
	
	//����yaw/pitch������Ʒ�ʽ���л���Ҫ�޸��л����ģʽ������ֵΪ��ǰ��ʵ��ֵ����ֹ��̨˦ͷ
	if(gimbalTaskStructure.pitchMotor.mode != pitchLastMode)
	{
		u8 nowMode = gimbalTaskStructure.pitchMotor.mode;
		if(nowMode == GIMBAL_MOTOR_ENCONDE || nowMode == GIMBAL_MOTOR_GYRO)
		{
			gimbalTaskStructure.pitchMotor.angle_set[nowMode] = gimbalTaskStructure.pitchMotor.angle[nowMode];
		}
		else if(nowMode == GIMBAL_MOTOR_ENCONDE_SPEED)
			gimbalTaskStructure.pitchMotor.speed_set[ENCONDE] = 0;
		else if(nowMode == GIMBAL_MOTOR_GYRO_SPEED)
			gimbalTaskStructure.pitchMotor.speed_set[GYRO] = 0;
		
		pitchLastMode = nowMode;
	}
	
	if(gimbalTaskStructure.yawMotor.mode != yawLastMode)
	{
		u8 nowMode = gimbalTaskStructure.yawMotor.mode;
		
		if(nowMode == GIMBAL_MOTOR_ENCONDE || nowMode == GIMBAL_MOTOR_GYRO)
		{
			gimbalTaskStructure.yawMotor.angle_set[nowMode] = gimbalTaskStructure.yawMotor.angle[nowMode];
		}
		else if(nowMode == GIMBAL_MOTOR_ENCONDE_SPEED)
			gimbalTaskStructure.yawMotor.speed_set[ENCONDE] = 0;
		else if(nowMode == GIMBAL_MOTOR_GYRO_SPEED)
			gimbalTaskStructure.yawMotor.speed_set[GYRO] = 0;
		
		yawLastMode = nowMode;
	}
}


//�涨����ֵ��������Ϊ�Ӳ������ӽǿ������Һ�����Ϊ��, �Ƕȵĵ�λ�Ƕ� �ٶȵĵ�λ�Ƕ�/S
void gimbalFeedbackUpdate(GimbalCtrl_t *gimbal_feedback_update)
{ 
    if (gimbal_feedback_update == NULL)
    {
        return;
    }
		/***Pitch��***/
    //��̨�Ƕ����ݸ���          
    gimbal_feedback_update->pitchMotor.angle[GYRO]    = gimbal_feedback_update->imu->pit; //�Ե�λ�û�ȡ�������ǵ�ֵ��
    gimbal_feedback_update->pitchMotor.angle[ENCONDE] = -motor_ecd_to_angle_change(gimbal_feedback_update->pitchMotor.base_inf.real_ecd, //���λ�û�ȡ�������ֵ��offest��_ecd������������ֵ�����
                                                                                   gimbal_feedback_update->pitchMotor.offsetEcd);
		//�ٶ�ֵ�����õ������ǵ��ٶ�ֵ
    gimbal_feedback_update->pitchMotor.speed[GYRO] =  (gimbal_feedback_update->imu->gyro[0]);
	  //��ȡ�����ǵĽ��ٶ�
    gimbal_feedback_update->pitchMotor.speed[ENCONDE] = gimbal_feedback_update->pitchMotor.speed[GYRO];
	  /***Yaw��***/
//		//��̨�Ƕ����ݸ���   
//		gimbal_feedback_update->yawMotor.angle[GYRO] = -gimbal_feedback_update->imu->yaw;			//�Ե�λ�û�ȡ�������ǵ�ֵ��	
//    gimbal_feedback_update->yawMotor.angle[ENCONDE] = -motor_ecd_to_angle_change(gimbal_feedback_update->yawMotor.base_inf.real_ecd,	//���λ�û�ȡ�������ֵ��offest��_ecd������������ֵ�����
//                                                                                 gimbal_feedback_update->yawMotor.offsetEcd);
		//��̨�Ƕ����ݸ���   
		gimbal_feedback_update->yawMotor.angle[GYRO] = -motor_ecd_to_angle_change(gimbal_feedback_update->yawMotor.base_inf.real_ecd,gimbal_feedback_update->yawMotor.offsetEcd);
    gimbal_feedback_update->yawMotor.angle[ENCONDE] = -motor_ecd_to_angle_change(gimbal_feedback_update->yawMotor.base_inf.real_ecd,	//���λ�û�ȡ�������ֵ��offest��_ecd������������ֵ�����
                                                                                 gimbal_feedback_update->yawMotor.offsetEcd);
		//�ٶ�ֵ�����õ������ǵ��ٶ�ֵ
    gimbal_feedback_update->yawMotor.speed[GYRO] 		= (gimbal_feedback_update->imu->gyro[2]) * 30;
		gimbal_feedback_update->yawMotor.speed[ENCONDE] = gimbal_feedback_update->yawMotor.speed[GYRO];
}


//������ԽǶ�
fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{ //�����ת��Ϊ��С���Ƿ���������� ��ǰʵ��ֵΪ8190������ֵΪ1������������Ľ����8189������ʵ�ʵĲ��Ӧ����-2  
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= MAX_ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += MAX_ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_360;
}


//�������ƣ�void gimbalBehaviourChange(gimbal_behaviour_t* next)
//�������ã�����ģʽ�л�
//��ڲ�������һ��ģʽ�Ľṹ��
//����  ֵ����
void gimbalBehaviourChange(gimbal_behaviour_t* next)
{ 
	//�����л�,��Ҫ�л���ǰ��Ϊָ�룬��ǰ��Ϊ�����Լ���������Ŀ��Ʒ�ʽ
	if(gimbalTaskStructure.behaviorNow->outBehaviorFun != NULL)
		gimbalTaskStructure.behaviorNow->outBehaviorFun();
	
	gimbalTaskStructure.behaviorNow = next;
	gimbalTaskStructure.behaviourStep = 0;
	gimbalTaskStructure.yawMotor.mode = next->motorYawCtrlType;
	gimbalTaskStructure.pitchMotor.mode = next->motorPitCtrlType;
	
	for(gimbal_behaviour_e i = GIMBAL_DEBUG; i < GIMBAL_MODE_LENGTH; i++)
	{
		if(gimbalTaskStructure.behaviorList + i == next)
		{
			gimbalTaskStructure.nowBehaviorName = i;
			break;
		}
	}
	
	if(gimbalTaskStructure.behaviorNow->enterBehaviorFun != NULL)
		gimbalTaskStructure.behaviorNow->enterBehaviorFun();
}




//�������ƣ�GimbalPidCalc(gimbal_motor_mode_e pitchCtrlType, gimbal_motor_mode_e yawCtrlType, gimbal_motor_t *pitchMotor, gimbal_motor_t *yawMotor)
//�������ã�����̨�������PID����
//��ڲ�����Pit������ģʽ�ͽṹ�壬Yaw������ģʽ�ͽṹ��
//����  ֵ����
void GimbalPidCalc(gimbal_motor_mode_e pitchCtrlType, gimbal_motor_mode_e yawCtrlType, gimbal_motor_t *pitchMotor, gimbal_motor_t *yawMotor)
{
	if(pitchCtrlType == GIMBAL_MOTOR_RAW) //����ֱ�ӿ��Ʋ���Ҫ��PID����
		pitchMotor->currentSet = pitchMotor->rawCmdCurrent;
	else if(pitchCtrlType == GIMBAL_MOTOR_ENCONDE || pitchCtrlType == GIMBAL_MOTOR_GYRO)
	{ //�����ǻ��߱��������Ʒ�ʽ����˫���Ŀ��Ʒ���
		PID_Calc(pitchMotor->PIDParameterList[pitchCtrlType]+OUTER, pitchMotor->angle[pitchCtrlType], pitchMotor->angle_set[pitchCtrlType]);
		pitchMotor->speed_set[pitchCtrlType] = pitchMotor->PIDParameterList[pitchCtrlType][OUTER].output;
		PID_Calc(pitchMotor->PIDParameterList[pitchCtrlType]+INNER, pitchMotor->speed[pitchCtrlType],-pitchMotor->speed_set[pitchCtrlType]);
		pitchMotor->currentSet = pitchMotor->PIDParameterList[pitchCtrlType][INNER].output;
	}
	else if(pitchCtrlType == GIMBAL_MOTOR_GYRO_SPEED)
	{
		PID_Calc(pitchMotor->PIDParameterList[GYRO]+INNER, pitchMotor->speed[GYRO], pitchMotor->speed_set[GYRO]);
		pitchMotor->currentSet = pitchMotor->PIDParameterList[GYRO][INNER].output;		
	}
	else if(pitchCtrlType == GIMBAL_MOTOR_ENCONDE_SPEED)
	{
		PID_Calc(pitchMotor->PIDParameterList[ENCONDE]+INNER, pitchMotor->speed[ENCONDE], -pitchMotor->speed_set[ENCONDE]);
		pitchMotor->currentSet = pitchMotor->PIDParameterList[ENCONDE][INNER].output;	
	}
	pitchMotor->base_inf.given_current = (s16)pitchMotor->currentSet;
	
	if(yawCtrlType == GIMBAL_MOTOR_RAW)
	{ 
		yawMotor->currentSet = yawMotor->rawCmdCurrent;
	}
	else if(yawCtrlType == GIMBAL_MOTOR_ENCONDE || yawCtrlType == GIMBAL_MOTOR_GYRO)
	{
		PID_Calc(yawMotor->PIDParameterList[yawCtrlType]+OUTER, yawMotor->angle[yawCtrlType], yawMotor->angle_set[yawCtrlType]);
		yawMotor->speed_set[yawCtrlType] = yawMotor->PIDParameterList[yawCtrlType][OUTER].output;
		PID_Calc(yawMotor->PIDParameterList[yawCtrlType]+INNER, yawMotor->speed[yawCtrlType], yawMotor->speed_set[yawCtrlType]);//yawMotor->PIDParameterList[yawCtrlType][OUTER].output;
		yawMotor->currentSet = yawMotor->PIDParameterList[yawCtrlType][INNER].output;
	}
	else if(yawCtrlType == GIMBAL_MOTOR_ENCONDE_SPEED)
	{
		PID_Calc(yawMotor->PIDParameterList[ENCONDE]+INNER, yawMotor->speed[ENCONDE], yawMotor->speed_set[ENCONDE]);
		yawMotor->currentSet = yawMotor->PIDParameterList[ENCONDE][INNER].output;
	}
	else if(yawCtrlType == GIMBAL_MOTOR_GYRO_SPEED)
	{
		PID_Calc(yawMotor->PIDParameterList[GYRO]+INNER, yawMotor->speed[GYRO], yawMotor->speed_set[GYRO]);
		yawMotor->currentSet = yawMotor->PIDParameterList[GYRO][INNER].output;			
	}
	yawMotor->base_inf.given_current = (s16)(yawMotor->currentSet);
}


//�������ƣ�gimbalAngleLimit()
//�������ã�����̨�������������λ
//��ڲ�������
//����  ֵ����
void gimbalAngleLimit()
{ 
	//yaw��û����λ��Ҫ��������ֵ��Ҫ����ʵ��ֵ��Χ���ɣ���Ҫ��pitch��λ
	gimbal_motor_mode_e yawMode = gimbalTaskStructure.yawMotor.mode;
	
	if(yawMode == GIMBAL_MOTOR_ENCONDE || yawMode == GIMBAL_MOTOR_GYRO)
		gimbalTaskStructure.yawMotor.angle_set[yawMode] = loop_fp32_constrain(gimbalTaskStructure.yawMotor.angle_set[yawMode],gimbalTaskStructure.yawMotor.minRelativeAngle, gimbalTaskStructure.yawMotor.maxRelativeAngle) - 30;
	//Pitch������λ
	s16 maxRange, minRange;
	fp32 maxAngleSet;
	fp32 minAngleSet;
	fp32 *nowAngleSet;
	if(gimbalTaskStructure.pitchMotor.mode != GIMBAL_MOTOR_ENCONDE && gimbalTaskStructure.pitchMotor.mode != GIMBAL_MOTOR_GYRO)
		return;
	if(gimbalTaskStructure.pitchMotor.mode == GIMBAL_MOTOR_ENCONDE)
	{
		maxRange = RELATIVE_PITCH_MAX_RANGE;
		minRange = RELATIVE_PITCH_MIN_RANGE;
		maxAngleSet = (gimbalTaskStructure.pitchMotor.maxRelativeAngle);
		minAngleSet = (gimbalTaskStructure.pitchMotor.minRelativeAngle);
		nowAngleSet = gimbalTaskStructure.pitchMotor.angle_set + GIMBAL_MOTOR_ENCONDE;
	}
	else if(gimbalTaskStructure.pitchMotor.mode == GIMBAL_MOTOR_GYRO)
	{ 
		//����������ǿ��Ʒ�������Ҫ���ݵ�ǰ�����Ƿ����ͱ�����������ʵ��ֵ����ԭ�еı�����������ת��Ϊ������ʵ��ֵ��������
		nowAngleSet = gimbalTaskStructure.pitchMotor.angle_set + GIMBAL_MOTOR_GYRO;
		maxRange = ABS_PITCH_MAX_RANGE;
		minRange = ABS_PITCH_MIN_RANGE;
		//���㵱ǰλ�õ�λ�����޺�λ�����޵ĽǶȲ�
		float relativeMax, relativeMin;
		relativeMax = loop_fp32_constrain(gimbalTaskStructure.pitchMotor.maxRelativeAngle - gimbalTaskStructure.pitchMotor.angle[GIMBAL_MOTOR_ENCONDE],RELATIVE_PITCH_MIN_RANGE, RELATIVE_PITCH_MAX_RANGE);
		relativeMin = loop_fp32_constrain(gimbalTaskStructure.pitchMotor.minRelativeAngle - gimbalTaskStructure.pitchMotor.angle[GIMBAL_MOTOR_ENCONDE],RELATIVE_PITCH_MIN_RANGE, RELATIVE_PITCH_MAX_RANGE);
		//���ǶȲ��ɱ����������̵���Ϊ�����ǵ�����
		relativeMax = relativeMax/(RELATIVE_PITCH_MAX_RANGE-RELATIVE_PITCH_MIN_RANGE)*(ABS_PITCH_MAX_RANGE-ABS_PITCH_MIN_RANGE);
		relativeMin = relativeMin/(RELATIVE_PITCH_MAX_RANGE-RELATIVE_PITCH_MIN_RANGE)*(ABS_PITCH_MAX_RANGE-ABS_PITCH_MIN_RANGE);
		maxAngleSet = loop_fp32_constrain(gimbalTaskStructure.pitchMotor.angle[GYRO] + relativeMax, ABS_PITCH_MIN_RANGE, ABS_PITCH_MAX_RANGE);
		minAngleSet = loop_fp32_constrain(gimbalTaskStructure.pitchMotor.angle[GYRO] + relativeMin, ABS_PITCH_MIN_RANGE, ABS_PITCH_MAX_RANGE);
		if(maxAngleSet < minAngleSet)
		{
			fp32 temp = maxAngleSet;
			maxAngleSet = minAngleSet;
			minAngleSet = temp;
		}
	}
	s16 midRange = (maxRange+minRange)/2;
	if(maxAngleSet - minAngleSet > (maxRange-minRange)/2)
	{ //�����λ����С��λ֮����һ�������ٽ�����䣬��Ҫ���⴦��
		if(*nowAngleSet > minAngleSet && *nowAngleSet < midRange)
			*nowAngleSet = minAngleSet;
		else if(*nowAngleSet < maxAngleSet && *nowAngleSet > midRange)
			*nowAngleSet = maxAngleSet;
	}
	else
	{
		*nowAngleSet = fp32_constrain(*nowAngleSet, minAngleSet, maxAngleSet);
	}
}

//�������ƣ�GIMBAL_CanbusCtrlMotors(s16 pitchCurrent, s16 yawCurrent,s16 bomb_Curreent, s16 frCurrent, s16 plCurrent)
//�������ã�ͨ��CAN��������͵���ֵ
//��ڲ�����pitch yaw bomb��plate����ֵ
//����  ֵ����
void GIMBAL_CanbusCtrlMotors(s16 pitchCurrent, s16 yawCurrent, s16 plCurrent,s16 flCurrent,s16 frCurrent)
{
	s16 can1_Current[4];
	s16 can2_Current[4];
	
  can1_Current[0]=yawCurrent;
	can1_Current[1]=plCurrent;
	can1_Current[2]=flCurrent;
	can1_Current[3]=frCurrent;
	
	can2_Current[1]=pitchCurrent;
	
  if(toe_is_error(DBUSTOE))
	{
		can1_Current[0]=0;
		can1_Current[1]=0;
		can1_Current[2]=0;
		can2_Current[0]=0;
		can2_Current[1]=0;
		can2_Current[2]=0;
		can2_Current[3]=0;
	}
		
	djiMotorCurrentSendQueue(CAN1, GIMBAL_CANBUS_SEND_HEADER,can1_Current, 4);
	djiMotorCurrentSendQueue(CAN2, GIMBAL_CANBUS_SEND_HEADER,can2_Current, 4);
}

