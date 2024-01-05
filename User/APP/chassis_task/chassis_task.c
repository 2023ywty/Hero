#include "chassis_task.h"
#include "gimbal_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"
#include "can.h"
#include "math.h"
#include "chassis_behaviour.h"
#include "Detect_Task.h"
#include "BoardB_link.h"
#include "led.h"


/*�������Ʊ���ϵ������*/
//δװ����ϵͳ��ʱ���һ�������ó�1(�ϲ���ϵͳ֮���1�ĳ�0)
const float power_data[61]=
{
	   1  ,  0.0030  ,  0.0060  ,  0.0090  ,  0.0120  ,  0.0150  ,  0.0180  ,  0.0210  ,  0.0240  ,  0.0270  ,  0.0300   ,  0.0390  ,  
0.0480  ,  0.0570  ,  0.0660  ,  0.0750  ,  0.0840  ,  0.0930  ,  0.1020  ,  0.1110  ,  0.1200  ,  0.1460  ,  0.1720   ,  0.1980  ,
0.2240  ,  0.2500  ,  0.3000  ,  0.3500  ,  0.4000  ,  0.4500  ,  0.5000  ,  0.5500  ,  0.6000  ,  0.6500  ,  0.7000   ,  0.7500  ,
0.7760  ,  0.8020  ,  0.8280  ,  0.8540  ,  0.8800  ,  0.8890  ,  0.8980  ,  0.9070  ,  0.9160  ,  0.9250  ,  0.9340   ,  0.9430  ,
0.9520  ,  0.9610  ,  0.9700  ,  0.9730  ,  0.9760  ,  0.9790  ,  0.9820  ,  0.9850  ,  0.9880  ,  0.9910  ,  0.9940   ,  0.9970  ,  
1.0000 
}; 


/*�ṹ���ʼ���ͺ�������*/
ChassisCtrl_t chassisTaskStructure;
void chassisBehaviourInit(chassis_behaviour_t *initBehavior, chassis_behaviour_e num, chassis_mode_e mode, 	bool_t (*enterBehaviorCondition)(void),
								bool_t (*outBehaviorCondition)(void), void (*enterBehaviorFun)(void), void (*outBehaviorFun)(void), void (*behaviorHandleFun)(float* vx, float* vy, float* vz, float* raw));
void chassisBehaviorSelect();
void chassisBehaviorChange(chassis_behaviour_t *next);
void chassisModeChangeHandle();
void chassisFeedBackUpdate();
void chassisSpeedLimit();
void chassisPidCalc();
void chassisCanbuscCtrlMotors(s16 cm1Current, s16 cm2Current, s16 cm3Current, s16 cm4Current);
void chassisPowerLimt();

u16 real_date,exp_date;
u16 real_agle,exp_agle;


float view_chassis_1 = 0;
float view_chassis_2 = 0;
float view_chassis_3 = 0;
float view_chassis_4 = 0;

//�������ƣ�chassisTask()
//�������ã���������������
//��ڲ�������
//����  ֵ����
void chassisTask(void *pvParameters)
{
	portTickType currentTime;
	//���̳�ʼ��
	chassisTaskInit();
	//FreeRTOS����ϵͳ�����ʱ������Ϊ���������ʼ���ṩʱ��
	vTaskDelay(4);
	//��ȡ��ǰʱ�䣬�ṩ��������ʱ������
	currentTime = xTaskGetTickCount();
	while(1)
	{
	view_chassis_1 = chassisTaskStructure.motor[CHASSIS_CM1].filterSpeed;
	view_chassis_2 = chassisTaskStructure.motor[CHASSIS_CM2].filterSpeed;
	view_chassis_3 = chassisTaskStructure.motor[CHASSIS_CM3].filterSpeed;
	view_chassis_4 = chassisTaskStructure.motor[CHASSIS_CM4].filterSpeed;
		
		real_date = BoardBLink.RefreeSyetem.power.chassis_power;
		exp_date  = BoardBLink.RefreeSyetem.power.chassis_power_buff;
		
		real_agle=*chassisTaskStructure.relativeAngle;
		exp_date=chassisTaskStructure.relativeAngleSet;
		
//		TIM_SetCompare4(TIM8, 50);
//		
//		Electricpush2_off();
		
		//ѡ����̿���ģʽ
		chassisBehaviorSelect();
		//���ģʽ����
		chassisModeChangeHandle();
		//����ʵ��ֵ������ʵ��ֵ�����˲�
		chassisFeedBackUpdate();
		/*****��ֵvx,vy,vz�����ٶ�******/
		//ֱ�ӷ�����ģʽ
		if(chassisTaskStructure.behaviorNow->mode == CHASSIS_RAW) 
			chassisTaskStructure.behaviorNow->behaviorHandleFun(&(chassisTaskStructure.motor[CHASSIS_CM1].rawCmdCurrent),
																                          &(chassisTaskStructure.motor[CHASSIS_CM2].rawCmdCurrent),
															                          	&(chassisTaskStructure.motor[CHASSIS_CM3].rawCmdCurrent),
																                          &(chassisTaskStructure.motor[CHASSIS_CM4].rawCmdCurrent));
		
		//��̨�н�ģʽ����vx��vy��vzģʽ
		else if(chassisTaskStructure.behaviorNow->mode == CHASSIS_ANGLE || chassisTaskStructure.behaviorNow->mode == CHASSIS_SPEED)
		{
			chassisTaskStructure.behaviorNow->behaviorHandleFun(chassisTaskStructure.relativeSpeedSet + VX, chassisTaskStructure.relativeSpeedSet + VY, chassisTaskStructure.relativeSpeedSet + VZ, NULL);
		}
		//�������ٶ��޷�
		chassisSpeedLimit();
		//����ң����(��ֵ�;�ֵ)
		rcDataCopy(&(chassisTaskStructure.rc.last));
		//vx vy���λ��ת����������pid����
		chassisPidCalc();
		//������Ϣ����
		chassisTaskStructure.powerLimit.kMaxOutput = power_data[*chassisTaskStructure.powerLimit.buffer];
		//��������
 	  chassisPowerLimt();
	 	//���з���
		chassisCanbuscCtrlMotors(chassisTaskStructure.motor[CHASSIS_CM1].baseInf.given_current *chassisTaskStructure.powerLimit.kMaxOutput, 
		                         chassisTaskStructure.motor[CHASSIS_CM2].baseInf.given_current *chassisTaskStructure.powerLimit.kMaxOutput,
								             chassisTaskStructure.motor[CHASSIS_CM3].baseInf.given_current *chassisTaskStructure.powerLimit.kMaxOutput,
								             chassisTaskStructure.motor[CHASSIS_CM4].baseInf.given_current *chassisTaskStructure.powerLimit.kMaxOutput);
		chassisCanbuscCtrlMotors(chassisTaskStructure.motor[CHASSIS_CM1].baseInf.given_current, 
		                         chassisTaskStructure.motor[CHASSIS_CM2].baseInf.given_current,
								             chassisTaskStructure.motor[CHASSIS_CM3].baseInf.given_current,
								             chassisTaskStructure.motor[CHASSIS_CM4].baseInf.given_current);

		//freeRTOS����ϵͳ������ʱ����
		vTaskDelayUntil(&currentTime, CHASSIS_TASK_MS);
	}
}


//�������ƣ�chassisTaskInit()
//�������ã����̳�ʼ��
//��ڲ�������
//����  ֵ����
void chassisTaskInit()
{
	//���̹��ʺͻ����������
	chassisTaskStructure.powerLimit.power  = &BoardBLink.RefreeSyetem.power.chassis_power;     //���̹���
	chassisTaskStructure.powerLimit.buffer = &BoardBLink.RefreeSyetem.power.chassis_power_buff;//��������ʣ
	//��ȡң��������
	chassisTaskStructure.rc.now =get_remote_control_point();
	//��ʼ���ٶ����Ʊ���  400(���Ϊ473)
	chassisTaskStructure.relativeSpeedMax[VX] = CHASSIS_NORMAL_MAX_VX;
	chassisTaskStructure.relativeSpeedMax[VY] = CHASSIS_NORMAL_MAX_VY;
	chassisTaskStructure.relativeSpeedMax[VZ] = CHASSIS_NORMAL_MAX_VZ;
  //��ʼ���������Ʊ���
	chassisTaskStructure.powerLimit.warnBuffer = CHASSIS_WARN_BUFF;
	chassisTaskStructure.powerLimit.maxOutput  = CHASSIS_NORMAL_MAX_OUTPUT;
	chassisTaskStructure.powerLimit.capState   = CLOSE;
	//��ʼ���������ģʽ��Ĭ��ֱ�ӷ�����ģʽ��
	chassisTaskStructure.mode     = CHASSIS_RAW;
	chassisTaskStructure.lastMode = CHASSIS_RAW;
	//��ʼ�������ĸ�����ļ��ٱ� 19  3508���
	chassisTaskStructure.motor[CHASSIS_CM1].baseInf.reduction_ratio = CHASSIS_MOTOR_REDUCTION_RATIO;
	chassisTaskStructure.motor[CHASSIS_CM2].baseInf.reduction_ratio = CHASSIS_MOTOR_REDUCTION_RATIO;
	chassisTaskStructure.motor[CHASSIS_CM3].baseInf.reduction_ratio = CHASSIS_MOTOR_REDUCTION_RATIO;
	chassisTaskStructure.motor[CHASSIS_CM4].baseInf.reduction_ratio = CHASSIS_MOTOR_REDUCTION_RATIO;
  //���̸�����̨ģʽ��λ��ʽPID
	PIDInit(&(chassisTaskStructure.anglePidParameter), 6, 0, 10, 0.8, CHASSIS_FOLLOW_MAX_VZ, 0, 4.4, chassisTaskStructure.maxRelativeAngle, -1, POSITION_180);
  //�����ĸ�������ٶȵ���PID
	PIDInit(&(chassisTaskStructure.motor[CHASSIS_CM1].pidParameter), 60, 0.25, 100, 0, 16000, -1, 500/19, -1, -1, SPEED);
	PIDInit(&(chassisTaskStructure.motor[CHASSIS_CM2].pidParameter), 60, 0.25, 100, 0, 16000, -1, 500/19, -1, -1, SPEED);
	PIDInit(&(chassisTaskStructure.motor[CHASSIS_CM3].pidParameter), 60, 0.25, 100, 0, 16000, -1, 500/19, -1, -1, SPEED);
	PIDInit(&(chassisTaskStructure.motor[CHASSIS_CM4].pidParameter), 60, 0.25, 100, 0, 16000, -1, 500/19, -1, -1, SPEED);
	//������Ӧ�Ŀ������˲�
	KalmanCreate(&(chassisTaskStructure.motor[CHASSIS_CM1].klmFiller), 1, 40);
	KalmanCreate(&(chassisTaskStructure.motor[CHASSIS_CM2].klmFiller), 1, 40);
	KalmanCreate(&(chassisTaskStructure.motor[CHASSIS_CM3].klmFiller), 1, 40);
	KalmanCreate(&(chassisTaskStructure.motor[CHASSIS_CM4].klmFiller), 1, 40);
	//���õ��̸���ģʽ����ز���
	chassisTaskStructure.relativeAngle    = gimbalTaskStructure.yawMotor.angle + ENCONDE;//������ģʽ�ĽǶ�
	chassisTaskStructure.maxRelativeAngle = RELATIVE_YAW_MAX_RANGE;
	chassisTaskStructure.minRelativeAngle = RELATIVE_YAW_MIN_RANGE;
	/*��ÿһ��ģʽ�䱸��Ӧ�Ĵ�����*/
	//DEBUGģʽ
	chassisBehaviourInit(chassisTaskStructure.behaviorList + CHASSIS_DEBUG     ,CHASSIS_DEBUG     ,DEBUG_CHSSIS_TYPE,enterChassisDebugCondition ,outChassisDebugCondition ,NULL,NULL,chassisDebugHandleFun);
	//����ģʽ
	chassisBehaviourInit(chassisTaskStructure.behaviorList + CHASSIS_ZERO_FORCE,CHASSIS_ZERO_FORCE,CHASSIS_RAW      ,enterZeroForceCondition    ,outZeroForceCondition    ,NULL,NULL,zeroForceHandleFun);
	//��ʼ��ģʽ 
	chassisBehaviourInit(chassisTaskStructure.behaviorList + CHASSIS_INIT      ,CHASSIS_INIT      ,CHASSIS_RAW      ,enterChassisInitCondition  ,outChassisInitCondition  ,NULL,NULL,chassisInitHandleFun);
  //����ģʽ
	chassisBehaviourInit(chassisTaskStructure.behaviorList + CHASSIS_ROTATE    ,CHASSIS_ROTATE    ,CHASSIS_SPEED    ,enterChassisRotateCondition,outChassisRotateCondition,NULL,NULL,chassisRotateHandleFun);
	//����������ģʽ
	chassisBehaviourInit(chassisTaskStructure.behaviorList + CHASSIS_ALONE     ,CHASSIS_ALONE     ,CHASSIS_SPEED    ,enterChassisAloneCondition ,outChassisAloneCondition ,NULL,NULL,chassisAloneHandleFun);
	//����ģʽ
	chassisBehaviourInit(chassisTaskStructure.behaviorList + CHASSIS_FOLLOW    ,CHASSIS_FOLLOW    ,CHASSIS_ANGLE    ,enterChassisFollowCondition,outChassisFollowCondition,NULL,NULL,chassisFollowHandleFun);
	//Ĭ��Ϊ����ģʽ
	chassisTaskStructure.behaviorNow = chassisTaskStructure.behaviorList + CHASSIS_ZERO_FORCE;
}


//�������ƣ�chassisBehaviourInit()
//�������ã����initBehavior�ṹ��
//��ڲ�����ģʽ��������������
//����  ֵ����
//bool_t (*enterBehaviorCondition)(void)��Ϊһ������ָ�룬ָ��һ��������������һ��bool_t��ֵ
//�������ӵ�ַָ���Ǻ����ĵ�ַ
void chassisBehaviourInit(chassis_behaviour_t *initBehavior, chassis_behaviour_e num, chassis_mode_e mode, 	bool_t (*enterBehaviorCondition)(void),
								          bool_t (*outBehaviorCondition)(void), void (*enterBehaviorFun)(void), void (*outBehaviorFun)(void), void (*behaviorHandleFun)(float* vx, float* vy, float* vz, float* raw))
{
	if(initBehavior == NULL || num < 0 || num >= CHASSIS_BEHAVOUR_LENGTH || enterBehaviorCondition == NULL || outBehaviorCondition == NULL || behaviorHandleFun == NULL)
	return;
	//�������
	initBehavior->num = num;
	initBehavior->behaviorHandleFun = behaviorHandleFun;
	initBehavior->enterBehaviorCondition = enterBehaviorCondition;
	initBehavior->outBehaviorCondition = outBehaviorCondition;
	initBehavior->enterBehaviorFun = enterBehaviorFun;
	initBehavior->outBehaviorFun = outBehaviorFun;
	initBehavior->mode = mode;
}


//�������ƣ�chassisBehaviorSelect()
//�������ã����̿����л�ģʽ����
//��ڲ�������
//����  ֵ����
void chassisBehaviorSelect()
{
	//�鿴��û�����ȼ��ȵ���Ϊ�ߵ���Ϊ�Ľ��������õ�����
	for(chassis_behaviour_t *iterator = chassisTaskStructure.behaviorList;iterator<chassisTaskStructure.behaviorNow;iterator++)
	{ 
		//��������ȼ��ߵĽ�����Ϊ�õ����㣬������Ǹ���Ϊ��ģʽ
		if(iterator->enterBehaviorCondition())
		{
			chassisBehaviorChange(iterator);
			break;
		}
	}
	//�鿴��ǰ��Ϊ�Ƿ��Ѿ��ﵽ���˳�������������ﵽ����Ѱ����һ�����Խ������Ϊ
	if(chassisTaskStructure.behaviorNow->outBehaviorCondition())
	{ 
		for(chassis_behaviour_t *iterator=chassisTaskStructure.behaviorNow;iterator<chassisTaskStructure.behaviorList+CHASSIS_BEHAVOUR_LENGTH;iterator++)
		{
			//�������ȼ���������������Ϊ����ô���������Ϊ
			if(iterator->enterBehaviorCondition())
			{
				chassisBehaviorChange(iterator);
				return;
			}
		}
	}
	else
		return;
	//���û������������Ϊ�Ľ�����������ô���̽�������ģʽ
	chassisBehaviorChange(chassisTaskStructure.behaviorList + CHASSIS_ZERO_FORCE);
}


//�������ƣ�chassisBehaviorChange()
//�������ã���������ģʽ
//��ڲ�������һ�����ģʽ
//����  ֵ����
void chassisBehaviorChange(chassis_behaviour_t *next)
{
	//������˳�����Ϊ�Ĵ���������ִ��
	if(chassisTaskStructure.behaviorNow->outBehaviorFun != NULL)
	   chassisTaskStructure.behaviorNow->outBehaviorFun();
	//����ǰ��Ϊ�л��ɺ����������Ϊ
	chassisTaskStructure.behaviorNow   = next;
	chassisTaskStructure.behaviourStep = 0;
	chassisTaskStructure.mode = chassisTaskStructure.behaviorNow->mode;
	//����н������Ϊ�Ĵ���������ִ��
	if(chassisTaskStructure.behaviorNow->enterBehaviorFun != NULL)
		 chassisTaskStructure.behaviorNow->enterBehaviorFun();
}


//�������ƣ�chassisModeChangeHandle()
//�������ã����̵��ģʽ����
//��ڲ�������
//����  ֵ����
void chassisModeChangeHandle()
{
	//�жϵ��ģʽ�Ƿ����˸ı�
	if(chassisTaskStructure.mode != chassisTaskStructure.lastMode)
	{ 
		//���ϴ�ģʽ��ֵ��lastMode
		chassisTaskStructure.lastMode = chassisTaskStructure.mode;
		//�л�������ģʽ��ʱ�򣬽���ǰ�ĽǶ�ƫ����Ϊ��������ֹ�л������ģʽ�����˦ͷ
		if(chassisTaskStructure.mode == CHASSIS_ANGLE)
			chassisTaskStructure.relativeAngleSet = *(chassisTaskStructure.relativeAngle);
	}
}


//�������ƣ�chassisSpeedLimit()
//�������ã����̵���ٶ��޷�
//��ڲ�������
//����  ֵ����
void chassisSpeedLimit()
{
	//���ݹ���
	if((IF_KEY_PRESSED_SHIFT)||(IF_KEY_PRESSED_CTRL)||( IF_RC_SW1_UP && IF_RC_SW2_DOWN ))
	  chassisTaskStructure.if_want_to_open_cap = CAP_CONFIG; 
	/*****����޷�*****/
	//������ݿ����������������
	if(chassisTaskStructure.if_want_to_open_cap == CAP_CONFIG)
	{
		chassisTaskStructure.powerLimit.maxOutput = CHASSIS_CAP_MAX_OUTPUT;	
		chassisTaskStructure.relativeSpeedMax[VX] = CHASSIS_CAP_MAX_VX;
		chassisTaskStructure.relativeSpeedMax[VY] = CHASSIS_CAP_MAX_VY;	
	}
	//������ݲ�����������޷�������
	else
	{
		chassisTaskStructure.powerLimit.maxOutput = CHASSIS_NORMAL_MAX_OUTPUT;	
		chassisTaskStructure.relativeSpeedMax[VX] = CHASSIS_NORMAL_MAX_VX;
		chassisTaskStructure.relativeSpeedMax[VY] = CHASSIS_NORMAL_MAX_VY;
	}
	//������ת��������
	chassisTaskStructure.relativeSpeedSet[VZ] = fp32_constrain(chassisTaskStructure.relativeSpeedSet[VZ], -chassisTaskStructure.relativeSpeedMax[VZ], chassisTaskStructure.relativeSpeedMax[VZ]);
	//�����ת���������ܴ�����ƽ���ٶȷ�ֹת��뾶����
//	if(f_abs(chassisTaskStructure.relativeSpeedSet[VZ]) > WARN_SPEED_VZ)
	if(0)
	{
		float k = (f_abs(chassisTaskStructure.chassisSpeedSet[VZ]) - WARN_SPEED_VZ)*(f_abs(chassisTaskStructure.chassisSpeedSet[VZ]) - WARN_SPEED_VZ)/(chassisTaskStructure.relativeSpeedMax[VZ]*chassisTaskStructure.relativeSpeedMax[VZ]);
		k = fp32_constrain(k, 0, 1);
		chassisTaskStructure.relativeSpeedMax[VY] *= k;
		chassisTaskStructure.relativeSpeedMax[VX] *= k;
	}
	//����ƽ�Ʒ�������
	chassisTaskStructure.relativeSpeedSet[VX] = fp32_constrain(chassisTaskStructure.relativeSpeedSet[VX], -chassisTaskStructure.relativeSpeedMax[VX], chassisTaskStructure.relativeSpeedMax[VX]);
	chassisTaskStructure.relativeSpeedSet[VY] = fp32_constrain(chassisTaskStructure.relativeSpeedSet[VY], -chassisTaskStructure.relativeSpeedMax[VY], chassisTaskStructure.relativeSpeedMax[VY]);
}


//�������ƣ�chassisFeedBackUpdate()
//�������ã�����ʵ��ֵ������ʵ��ֵ�����˲�
//��ڲ�������
//����  ֵ����
void chassisFeedBackUpdate()
{
	chassisTaskStructure.motor[CHASSIS_CM1].filterSpeed = KalmanFilter(&(chassisTaskStructure.motor[CHASSIS_CM1].klmFiller), chassisTaskStructure.motor[CHASSIS_CM1].baseInf.real_speed_rpm);
	chassisTaskStructure.motor[CHASSIS_CM2].filterSpeed = KalmanFilter(&(chassisTaskStructure.motor[CHASSIS_CM2].klmFiller), chassisTaskStructure.motor[CHASSIS_CM2].baseInf.real_speed_rpm);
	chassisTaskStructure.motor[CHASSIS_CM3].filterSpeed = KalmanFilter(&(chassisTaskStructure.motor[CHASSIS_CM3].klmFiller), chassisTaskStructure.motor[CHASSIS_CM3].baseInf.real_speed_rpm);
	chassisTaskStructure.motor[CHASSIS_CM4].filterSpeed = KalmanFilter(&(chassisTaskStructure.motor[CHASSIS_CM4].klmFiller), chassisTaskStructure.motor[CHASSIS_CM4].baseInf.real_speed_rpm);
}



//�������ƣ�chassisPidCalc()
//�������ã�vx vy���λ��ת����������pid����
//��ڲ�������
//����  ֵ����
void chassisPidCalc()
{	
	//�����ֱ�Ӹ�������͵���������Ҫ����PID���㣬ֱ�ӷ��ͼ���
	if(chassisTaskStructure.mode == CHASSIS_RAW)
	{ 
		for(u8 i = CHASSIS_CM1; i <= CHASSIS_CM4; i++)
		{
			chassisTaskStructure.motor[i].currentSet = chassisTaskStructure.motor[i].rawCmdCurrent;
		}
	}
	//�������̨�н�ģʽ��vx��vy��vzģʽ����Ҫ����PID����
	else if(chassisTaskStructure.mode == CHASSIS_ANGLE || chassisTaskStructure.mode == CHASSIS_SPEED)
	{
		//��ȡ��̨������ԽǶ�(������)
		float relationAngle = (*(chassisTaskStructure.relativeAngle)+125)/180.0*PI;
		//�����ٶȷֽ�
		chassisTaskStructure.chassisSpeedSet[VX] = cos(relationAngle)*chassisTaskStructure.relativeSpeedSet[VX] - sin(relationAngle)*chassisTaskStructure.relativeSpeedSet[VY];
		chassisTaskStructure.chassisSpeedSet[VY] = sin(relationAngle)*chassisTaskStructure.relativeSpeedSet[VX] + cos(relationAngle)*chassisTaskStructure.relativeSpeedSet[VY];
		chassisTaskStructure.chassisSpeedSet[VZ] = chassisTaskStructure.relativeSpeedSet[VZ];
		//������������ٶȷֽ⵽������
		chassisTaskStructure.motor[CHASSIS_CM1].speedSet = +chassisTaskStructure.chassisSpeedSet[VX] - chassisTaskStructure.chassisSpeedSet[VY] - chassisTaskStructure.chassisSpeedSet[VZ];
		chassisTaskStructure.motor[CHASSIS_CM2].speedSet = +chassisTaskStructure.chassisSpeedSet[VX] + chassisTaskStructure.chassisSpeedSet[VY] - chassisTaskStructure.chassisSpeedSet[VZ];
		chassisTaskStructure.motor[CHASSIS_CM3].speedSet = -chassisTaskStructure.chassisSpeedSet[VX] + chassisTaskStructure.chassisSpeedSet[VY] - chassisTaskStructure.chassisSpeedSet[VZ];
		chassisTaskStructure.motor[CHASSIS_CM4].speedSet = -chassisTaskStructure.chassisSpeedSet[VX] - chassisTaskStructure.chassisSpeedSet[VY] - chassisTaskStructure.chassisSpeedSet[VZ];
		//б��ʹʵ��ֵ���������ٶ�����ֵ
		RAMP_float(chassisTaskStructure.motor[CHASSIS_CM1].speedSet,chassisTaskStructure.motor[CHASSIS_CM1].filterSpeed,100);	
		RAMP_float(chassisTaskStructure.motor[CHASSIS_CM2].speedSet,chassisTaskStructure.motor[CHASSIS_CM2].filterSpeed,100);	
		RAMP_float(chassisTaskStructure.motor[CHASSIS_CM3].speedSet,chassisTaskStructure.motor[CHASSIS_CM3].filterSpeed,100);	
	  RAMP_float(chassisTaskStructure.motor[CHASSIS_CM4].speedSet,chassisTaskStructure.motor[CHASSIS_CM4].filterSpeed,100);	
		//�ٶȻ�PID�ļ��㲿��
		PID_Calc(&(chassisTaskStructure.motor[CHASSIS_CM1].pidParameter), chassisTaskStructure.motor[CHASSIS_CM1].filterSpeed,chassisTaskStructure.motor[CHASSIS_CM1].speedSet);
		PID_Calc(&(chassisTaskStructure.motor[CHASSIS_CM2].pidParameter), chassisTaskStructure.motor[CHASSIS_CM2].filterSpeed,chassisTaskStructure.motor[CHASSIS_CM2].speedSet);
		PID_Calc(&(chassisTaskStructure.motor[CHASSIS_CM3].pidParameter), chassisTaskStructure.motor[CHASSIS_CM3].filterSpeed,chassisTaskStructure.motor[CHASSIS_CM3].speedSet);
		PID_Calc(&(chassisTaskStructure.motor[CHASSIS_CM4].pidParameter), chassisTaskStructure.motor[CHASSIS_CM4].filterSpeed,chassisTaskStructure.motor[CHASSIS_CM4].speedSet);
		//��PID�ļ�������ֵ�����
		for(u8 i = CHASSIS_CM1;i <= CHASSIS_CM4; i++)
		{
			chassisTaskStructure.motor[i].currentSet = chassisTaskStructure.motor[i].pidParameter.output;
		}		
	}
  //�����ģʽѡ�񣬽��ĸ����ӵĵ���ֵ����Ϊ0
	else
	{
		for(u8 i = CHASSIS_CM1; i <= CHASSIS_CM4; i++)
		{
			chassisTaskStructure.motor[i].currentSet = 0;
		}			
	}
	//�����ĸ�PID��output
	float outputSum = f_abs(chassisTaskStructure.motor[CHASSIS_CM1].currentSet) + f_abs(chassisTaskStructure.motor[CHASSIS_CM2].currentSet) + f_abs(chassisTaskStructure.motor[CHASSIS_CM3].currentSet) + f_abs(chassisTaskStructure.motor[CHASSIS_CM4].currentSet);
	if(outputSum > chassisTaskStructure.powerLimit.maxOutput*chassisTaskStructure.powerLimit.kMaxOutput)
	{
		float k = chassisTaskStructure.powerLimit.maxOutput*chassisTaskStructure.powerLimit.kMaxOutput/outputSum;
		
		for(u8 i = CHASSIS_CM1; i <= CHASSIS_CM4; i++)
			chassisTaskStructure.motor[i].pidParameter.output = chassisTaskStructure.motor[i].pidParameter.output*k;
	}
	for(u8 i = CHASSIS_CM1; i <= CHASSIS_CM4; i++)
	{
		chassisTaskStructure.motor[i].baseInf.given_current = (s16)chassisTaskStructure.motor[i].currentSet;
	}
}



//�������ƺ���
void chassisPowerLimt()//��������2022������������
{
	float WARNING_REMAIN_POWER=60;
	
	float Joule_Residue=0;//ʣ�๦�ʻ����� 
	
	Joule_Residue=*chassisTaskStructure.powerLimit.buffer;
	
	if(Joule_Residue < WARNING_REMAIN_POWER)
	{
		chassisTaskStructure.powerLimit.kMaxOutput = (float)(Joule_Residue / WARNING_REMAIN_POWER) * (float)(Joule_Residue / WARNING_REMAIN_POWER);
	}
	else chassisTaskStructure.powerLimit.kMaxOutput =1;
}



void chassisCanbuscCtrlMotors(s16 cm1Current, s16 cm2Current, s16 cm3Current, s16 cm4Current)
{
	s16 chassisCurrent[4];
	chassisCurrent[0] = cm1Current;
	chassisCurrent[1] = cm2Current;
	chassisCurrent[2] = cm3Current;
	chassisCurrent[3] = cm4Current;
	if(toe_is_error(DBUSTOE))
	{
		chassisCurrent[0] = 0;   
		chassisCurrent[1] = 0;
		chassisCurrent[2] = 0;
		chassisCurrent[3] = 0;    
	}    
	djiMotorCurrentSendQueue(CAN1, CHASSIS_CANBUS_SEND_HEADER, chassisCurrent, 4);
}

