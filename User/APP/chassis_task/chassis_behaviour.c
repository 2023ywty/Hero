#include "chassis_behaviour.h"
#include "detect_task.h"
#include "robot.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "Remote_Control.h"


/*************************************DEBUGģʽ*************************************************/
bool_t enterChassisDebugCondition()
{
#ifdef DEBUG_CHASSIS
    return TRUE;
#endif

#ifndef DEBUG_CHASSIS
    return FALSE;
#endif
}

bool_t outChassisDebugCondition()
{
#ifdef DEBUG_CHASSIS
    return FALSE;
#endif

#ifndef DEBUG_CHASSIS
    return TRUE;
#endif
}


void chassisDebugHandleFun(float *vx, float *vy, float *vz, float *none)
{
	if(toe_is_error(DBUSTOE)||vx == NULL || vy == NULL || vz == NULL)
	{
			*vx = 0;
			*vy = 0;
			*vz = 0;
			return;
	}
	
		if(IF_RC_SW1_DOWN && IF_RC_SW2_DOWN)
	{
  *vx = -CHASSIS_NORMAL_MAX_VX*(RC_CH0_RLR_OFFSET/660.0);
	*vy = CHASSIS_NORMAL_MAX_VY*(RC_CH1_RUD_OFFSET/660.0);
	*vz = -CHASSIS_NORMAL_MAX_VX*(RC_CH2_LLR_OFFSET/660.0);
	}
}

/*************************************DEBUGģʽ*************************************************/



/*************************************����ģʽ*************************************************/
bool_t enterZeroForceCondition()
{
   if(toe_is_error(DBUSTOE) || toe_is_error(DETECT_CHASSIS_CM1_MOTOR) || toe_is_error(DETECT_CHASSIS_CM2_MOTOR) || toe_is_error(DETECT_CHASSIS_CM3_MOTOR) || toe_is_error(DETECT_CHASSIS_CM4_MOTOR))
    {  
        return TRUE;
    }

    {
    if(robotInf.modeStep < ROBOT_INIT_CHASSIS && robotInf.robotMode == ROBOT_INIT)
        return TRUE;
    }

    if(robotInf.robotMode == ROBOT_ZERO_FORCE)
        return TRUE;

    return FALSE;
}

bool_t outZeroForceCondition()
{
    if(enterZeroForceCondition())
        return FALSE;
    return TRUE;
}

void zeroForceHandleFun(float *raw_cm1, float *raw_cm2, float *raw_cm3, float *raw_cm4)
{
    *raw_cm1 = 0;
    *raw_cm2 = 0;
    *raw_cm3 = 0;
    *raw_cm4 = 0;
}
/*************************************����ģʽ*************************************************/




/*************************************��ʼ��ģʽ*************************************************/
bool_t enterChassisInitCondition()
{
  if((robotInf.modeStep == ROBOT_INIT_CHASSIS && robotInf.robotMode == ROBOT_INIT)||gimbalTaskStructure.nowBehaviorName ==GIMBAL_INIT)
		return TRUE;
	return FALSE;	
}


bool_t outChassisInitCondition()
{
    if(enterChassisInitCondition())
        return FALSE;
    return TRUE;
}


void chassisInitHandleFun(float *raw_cm1, float *raw_cm2, float *raw_cm3, float *raw_cm4)
{
    *raw_cm1 = 0;
    *raw_cm2 = 0;
    *raw_cm3 = 0;
    *raw_cm4 = 0;
    robotInf.modeStep = ROBOT_INIT_CHASSIS + 1;
}
/*************************************��ʼ��ģʽ*************************************************/



/*************************************����ģʽ************************************************/
//���룺��F��   ����  sw1̧�߲���sw2̧��
//�˳�����G��   ����  sw1̧�߲���sw2����
//w a s d�����ƶ�
bool_t enterChassisRotateCondition()
{
    if((IF_KEY_PRESSED_F)||(IF_RC_SW1_DOWN&&CHASSIS_FIRST_S2_DOWN))
		return TRUE;
	else
		return FALSE;
}

bool_t outChassisRotateCondition()
{
  if((IF_KEY_PRESSED_G)||(IF_RC_SW1_DOWN&&CHASSIS_FIRST_S2_MID))
		return TRUE;
	else
		return FALSE;
}


void chassisRotateHandleFun(float* vx, float *vy, float* vz, float* none)
{  
  if(vx == NULL || vy == NULL || vz == NULL)
		return;
	static float Key_ROTATE_Vx; static float Key_ROTATE_Vy;
	static float Rotate_speed_changbytime_flag;//�Ƿ�Ϊ�������ݱ�־λ
	/****ƽ�ƿ���****/
	if(IF_KEY_PRESSED_W)
		Key_ROTATE_Vx+=2;
	else if(IF_KEY_PRESSED_S)
		Key_ROTATE_Vx-=2;
	else
		Key_ROTATE_Vx=0;
	
  if(IF_KEY_PRESSED_A)
		Key_ROTATE_Vy-=2;
  else if(IF_KEY_PRESSED_D)
		Key_ROTATE_Vy+=2;
	else
		Key_ROTATE_Vy=0;
	/****��ת����****/
	if(IF_KEY_PRESSED_F)
		Rotate_speed_changbytime_flag=1;
	else 
		Rotate_speed_changbytime_flag=0;
	
	if(Rotate_speed_changbytime_flag==0)
		*vz = CHASSIS_ROTATE_VZ;
	else
    *vz=CHASSIS_ROTATE_VZ*cos(*(chassisTaskStructure.relativeAngle)/180.0*PI);//������
    //����ģʽ��ƽ�Ƽ���
    *vx = CHASSIS_NORMAL_MAX_VX/2*(RC_CH1_RUD_OFFSET/660.0)+Key_ROTATE_Vx;
    *vy = CHASSIS_NORMAL_MAX_VY/2*(RC_CH0_RLR_OFFSET/660.0)+Key_ROTATE_Vy;
}



/*************************************����������ģʽ************************************************/
/*��Z�������������ģʽ   ��X�˳�����������ģʽ*/
/*��W��ǰ�ߣ���S�����*/
/*��A�����ߣ���D������*/
//��z�������������ģʽ
bool_t enterChassisAloneCondition()
{
   if(IF_KEY_PRESSED_Z)
		return TRUE;
	return FALSE;
}

bool_t outChassisAloneCondition()
{
    if(IF_KEY_PRESSED_G)
		return TRUE;
	return FALSE;
}

void chassisAloneHandleFun(float* vx, float *vy, float* vz, float* none)
{
	static float Key_Alone_Vx; static float Key_Alone_Vy;
	if(IF_KEY_PRESSED_W)
		Key_Alone_Vx+=5;
	else if(IF_KEY_PRESSED_S)
		Key_Alone_Vx-=5;
	else
	  Key_Alone_Vx=0;
	//���ҷ���
  if(IF_KEY_PRESSED_A)
	  Key_Alone_Vy-=5;
  else if(IF_KEY_PRESSED_D)
    Key_Alone_Vy+=5;
	else
		Key_Alone_Vy=0;	
	*vx = CHASSIS_NORMAL_MAX_VX*(RC_CH1_RUD_OFFSET/660.0)+Key_Alone_Vx;
	*vy = CHASSIS_NORMAL_MAX_VY*(RC_CH0_RLR_OFFSET/660.0)+Key_Alone_Vy;
	*vz = 0;
}
/*************************************����������ģʽ************************************************/




/*************************************����ģʽ************************************************/
//Ĭ��Ϊ����ģʽ
bool_t enterChassisFollowCondition()
{
		return TRUE;
}

bool_t outChassisFollowCondition()
{
		return FALSE;
}
 
void chassisFollowHandleFun(float *vx, float *vy, float *vz, float *none)
{
  if(vx == NULL || vy == NULL || vz == NULL)
	return;
	static float Key_Follow_Vx; static float Key_Follow_Vy;//���̿����ٶȴ�С
	//ǰ����
	if(IF_KEY_PRESSED_W)
		Key_Follow_Vx+=2;
	else if(IF_KEY_PRESSED_S)
		Key_Follow_Vx-=2;
	else
		Key_Follow_Vx=0;
	//���ҷ���
  if(IF_KEY_PRESSED_A)
		Key_Follow_Vy-=2;
  else if(IF_KEY_PRESSED_D)
		Key_Follow_Vy+=2;
	else
		Key_Follow_Vy=0;	
	
	//б�º�����CHASSIS_TASK_MS�Ǹ�����ٶȣ�
	chassisTaskStructure.relativeAngleSet=loop_ramp_float(0, chassisTaskStructure.relativeAngleSet, 180.0/1000*CHASSIS_TASK_MS, chassisTaskStructure.minRelativeAngle, chassisTaskStructure.maxRelativeAngle);
	//�����ĽǶ���Զ��0����Ϊ����Ҫ������̨
	PID_Calc(&(chassisTaskStructure.anglePidParameter), *(chassisTaskStructure.relativeAngle), chassisTaskStructure.relativeAngleSet);
	//б�º�������
	*vx = RAMP_float(*vx,CHASSIS_NORMAL_MAX_VX*(RC_CH1_RUD_OFFSET/660.0)+Key_Follow_Vx,180.0/1000*CHASSIS_TASK_MS);
	*vy = RAMP_float(*vy,CHASSIS_NORMAL_MAX_VY*(RC_CH0_RLR_OFFSET/660.0)+Key_Follow_Vy,180.0/1000*CHASSIS_TASK_MS);
	*vz = RAMP_float(*vz,chassisTaskStructure.anglePidParameter.output,180.0/1000*CHASSIS_TASK_MS);
}

