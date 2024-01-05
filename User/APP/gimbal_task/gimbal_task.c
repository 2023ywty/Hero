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


/*结构体初始化和函数声明*/
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
	


//函数名称：gimbalTask()
//函数作用：云台任务主函数
//入口参数：无
//返回  值：无
void gimbalTask(void *pvParameters)
{
	portTickType currentTime;
	//云台初始化
	gimbalInit();
	//FreeRTOS操作系统相对延时函数，为其他任务初始化提供时间
	vTaskDelay(2);
	fp32 *pitchAngleSet, *yawAngleSet;
	u8    pitchCtrlType,  yawCtrlType;
	//获取当前时间，提供给绝对延时函数上
	currentTime = xTaskGetTickCount();
	while(1)
	{
		//获取期望值和真实值，用于Jscope检测数据
//		Yaw_exp_speed = gimbalTaskStructure.yawMotor.speed_set[GYRO];
//		Yaw_real_speed = gimbalTaskStructure.yawMotor.speed[GYRO];
		Yaw_exp_angle = gimbalTaskStructure.yawMotor.angle_set[GYRO];
		Yaw_real_angle = gimbalTaskStructure.yawMotor.angle[GYRO];
		Pit_exp_speed   = gimbalTaskStructure.pitchMotor.angle_set[ENCONDE] ;
		Pit_real_speed  = gimbalTaskStructure.pitchMotor.angle[ENCONDE] ;
		Pit_real_angle=gimbalTaskStructure.pitchMotor.angle[GYRO];
		yaw_postion=gimbalTaskStructure.pitchMotor.base_inf.real_ecd;
//		//按B开启UI界面，通过B板发数据
//		if(IF_KEY_PRESSED_B)     
//		   Restart_UsrInterFace();
		//循环调用模式切换函数，先调用当前模式
		gimbalBehaviourSelect();
	  //电机模式过渡
		gimbalMotorCtrlChangeHandle();
		//更新实际值数据
		gimbalFeedbackUpdate(&gimbalTaskStructure);
		//更新期望值
		pitchCtrlType = gimbalTaskStructure.behaviorNow->motorPitCtrlType;
		yawCtrlType   = gimbalTaskStructure.behaviorNow->motorYawCtrlType;
		//给yaw轴和pitch轴电机的期望值赋值
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
		//给处理函数赋值期望值
		gimbalTaskStructure.behaviorNow->behaviorHandleFun(yawAngleSet, pitchAngleSet);
		//限位
		gimbalAngleLimit();
		//更新遥控器last
		rcDataCopy(&(gimbalTaskStructure.rc.last));
		//pid计算
		GimbalPidCalc(pitchCtrlType, yawCtrlType, &(gimbalTaskStructure.pitchMotor), &(gimbalTaskStructure.yawMotor));
//		//加入队列进行发送
//		GIMBAL_CanbusCtrlMotors(gimbalTaskStructure.pitchMotor.base_inf.given_current, 
//														-gimbalTaskStructure.yawMotor.  base_inf.given_curbehaviorHandleFunrent,
//														shootTaskStructure. plateMotor.base_inf.given_current,
//		                        shootTaskStructure. leftFirMotor.base_inf.given_current, 
//		                        shootTaskStructure. rightFirMotor.base_inf.given_current);	// 发射优先级更高，转移至发射任务
		//绝对延时
		vTaskDelayUntil(&currentTime, GIMBAL_TASK_MS);
		}
}



void gimbalInit()
{
	//获取陀螺仪的角度值
	gimbalTaskStructure.imu = &imu_;
	//得到机器人的当前模式和初始化状态
	gimbalTaskStructure.robotMode     = &(robotInf.robotMode);
	gimbalTaskStructure.robotModeStep = &(robotInf.modeStep);
	//获取遥控器的当前值和上一次的值
	gimbalTaskStructure.rc.now = get_remote_control_point();
	rcDataCopy(&(gimbalTaskStructure.rc.last));
	//？？？
	motorSpeedCalcInit(&gimbalPitMotorSpeedCalc, &(gimbalTaskStructure.pitchMotor.base_inf), 0, MOTOR_ECD_FEEDBACL_RANGE, xTaskGetTickCount(), 8);
	motorSpeedCalcInit(&gimbalYawMotorSpeedCalc, &(gimbalTaskStructure.yawMotor.base_inf),   0, MOTOR_ECD_FEEDBACL_RANGE, xTaskGetTickCount(), 8);	
	//pitch云台电机初始化(6020)
	gimbalTaskStructure.pitchMotor.mode = GIMBAL_MOTOR_RAW;
	gimbalTaskStructure.pitchMotor.lastMode = GIMBAL_MOTOR_RAW;//默认直接发电流模式
	gimbalTaskStructure.pitchMotor.offsetEcd = GIMBAL_PITCH_MOTOR_OFFSET_ECD;  //上电的中值（编码器）
	gimbalTaskStructure.pitchMotor.maxRelativeAngle = RELATIVE_PITCH_ANGLE_MAX;//最大的角度（陀螺仪）
	gimbalTaskStructure.pitchMotor.minRelativeAngle = RELATIVE_PITCH_ANGLE_MIN;//最小的角度（陀螺仪）
	//yaw云台电机初始化(6020)
	gimbalTaskStructure.yawMotor.mode = GIMBAL_MOTOR_RAW;
	gimbalTaskStructure.yawMotor.lastMode = GIMBAL_MOTOR_RAW;//默认直接发电流模式
	gimbalTaskStructure.yawMotor.offsetEcd = GIMBAL_YAW_MOTOR_OFFSET_ECD;  //上电的中值（编码器）    
	gimbalTaskStructure.yawMotor.maxRelativeAngle = RELATIVE_YAW_MAX_RANGE;//最大的角度（陀螺仪）
	gimbalTaskStructure.yawMotor.minRelativeAngle = RELATIVE_YAW_MIN_RANGE;//最小的角度（陀螺仪）
  //云台实际值更新
	gimbalFeedbackUpdate(&gimbalTaskStructure);
	//云台期望值更新(初始化为实际值)
	gimbalTaskStructure.yawMotor.angle_set[ENCONDE]   = gimbalTaskStructure.yawMotor.angle[ENCONDE];
	gimbalTaskStructure.yawMotor.angle_set[GYRO]      = gimbalTaskStructure.yawMotor.angle[GYRO];
	gimbalTaskStructure.pitchMotor.angle_set[ENCONDE] = gimbalTaskStructure.pitchMotor.angle[ENCONDE];
	gimbalTaskStructure.pitchMotor.angle_set[GYRO]    = gimbalTaskStructure.pitchMotor.angle[GYRO];
  //是否用DEBUG模式
	#ifdef DEBUG_GIMBAL
	gimbalBehaviourInit(gimbalTaskStructure.behaviorList + GIMBAL_DEBUG, DEBUG_GIMBAL_PITCH_TYPE, DEBUG_GIMBAL_YAW_TYPE, gimbalDebugBehaviourHandleFun, gimbalDebugBehaviourEnterCondition, gimbalDebugBehaviourOutCondition, NULL, NULL);
	#endif
	//是否用MID_GIMBAL模式
	#ifdef MID_GIMBAL
	gimbalBehaviourInit(gimbalTaskStructure.behaviorList + GIMBAL_MID, MID_GIMBAL_PITCH_TYPE, MID_GIMBAL_YAW_TYPE, gimbalMidBehaviourHandleFun, gimbalMidBehaviourEnterCondition, gimbalMidBehaviourOutCondition, NULL, NULL);
	#endif
	//云台电机PID初始化
	gimbalPidInit();
	/*自瞄卡尔曼滤波,二阶初始化*/
	/*****给各个模式配备处理函数*****/
	//无力状态
	gimbalBehaviourInit(gimbalTaskStructure.behaviorList + GIMBAL_ZERO_FORCE, GIMBAL_MOTOR_RAW, GIMBAL_MOTOR_RAW, gimbalZeroForceBehaviourHandleFun, gimbalZeroForceBehaviourEnterCondition, gimbalZeroForceBehaviourOutCondition, NULL, NULL);
	//初始状态设置
	gimbalBehaviourInit(gimbalTaskStructure.behaviorList + GIMBAL_INIT, GIMBAL_MOTOR_ENCONDE, GIMBAL_MOTOR_ENCONDE, gimbalInitBehaviourHandleFun, gimbalInitBehaviourEnterCondition, gimbalInitBehaviourOutCondition, NULL, NULL);
	//普通跟随模式设置
	gimbalBehaviourInit(gimbalTaskStructure.behaviorList + GIMBAL_NORMAL, GIMBAL_MOTOR_GYRO, GIMBAL_MOTOR_GYRO, gimbalNormalBehaviourHandleFun, gimbalNormalBehaviourEnterCondition, gimbalNormalBehaviourOutCondition, NULL, NULL);
//	//吊射模式
//	gimbalBehaviourInit(gimbalTaskStructure.behaviorList + GIMBAL_Dropshot, GIMBAL_MOTOR_ENCONDE, GIMBAL_MOTOR_ENCONDE, gimbalDropshotBehaviourHandleFun, gimbalDropshotBehaviourEnterCondition, gimbalADropshotBehaviourOutCondition, NULL, NULL);
//	//自瞄
//	gimbalBehaviourInit(gimbalTaskStructure.behaviorList + AUTO, GIMBAL_MOTOR_GYRO, GIMBAL_MOTOR_GYRO, gimbalAutoBehaviourHandleFun, gimbalAutoBehaviourEnterCondition, gimbalAutoBehaviourOutCondition, NULL, NULL);
  //默认为无力模式
	gimbalTaskStructure.behaviorNow = gimbalTaskStructure.behaviorList + GIMBAL_ZERO_FORCE;
}


//函数名称：gimbalPidInit()
//函数作用：用于云台两个电机的PID初始化
//入口参数：无
//返回  值：无
void gimbalPidInit()
{
//void PIDInit(PidTypeDef *pid,float kp,float ki,float kd,float ka,float max_out,float dead_band,float i_band,float max_input, float i_maxout, pid_mode_e model)
  /****Yaw轴电机****/
//	PIDInit(*(gimbalTaskStructure.yawMotor.PIDParameterList + GYRO) + INNER		, 600,2.5, 0, 0, 30000, 0, 1000, 1000, 10000, SPEED);
//	PIDInit(*(gimbalTaskStructure.yawMotor.PIDParameterList + GYRO) + OUTER		, 12, 0, 1.5, 0, 10000, 0, 0, 180, 0, POSITION_180);
	PIDInit(*(gimbalTaskStructure.yawMotor.PIDParameterList + GYRO) + INNER		, 500, 2.5, 250, 0, 30000, 0, 1000, 1000, 10000, SPEED);
	PIDInit(*(gimbalTaskStructure.yawMotor.PIDParameterList + GYRO) + OUTER		, 4, 0.5,8, 0, 500, 0, 0, 180, 0, POSITION_180);
	
	PIDInit(*(gimbalTaskStructure.yawMotor.PIDParameterList + ENCONDE) + INNER, 400, 2.5, 0, 0, 30000, 0, 300, 10000, 10000, SPEED);
	PIDInit(*(gimbalTaskStructure.yawMotor.PIDParameterList + ENCONDE) + OUTER, 5, 0, 2, 0, 500, 0, 0, 180, 0, POSITION_180);
  /****Pitch轴电机****/
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


//函数名称：gimbalBehaviourSelect()
//函数作用：切换云台的模式
//入口参数：无
//返回  值：无
void gimbalBehaviourSelect()
{
	for(gimbal_behaviour_t *gbIterator = gimbalTaskStructure.behaviorList; 
		gbIterator != gimbalTaskStructure.behaviorNow && gbIterator != gimbalTaskStructure.behaviorList + GIMBAL_MODE_LENGTH;
		gbIterator++)
	{ //先遍历比当前优先级高的behaviour，如果满足进入条件，则进行切换
			
		if(gbIterator->enterBehaviorCondition != NULL && gbIterator->enterBehaviorCondition())
		{ //达到该行为的切换条件，进行行为切换
			gimbalBehaviourChange(gbIterator);
			return;
		}
	}
		
	if(gimbalTaskStructure.behaviorNow->outBehaviorCondition != NULL && gimbalTaskStructure.behaviorNow->outBehaviorCondition())
	{ //如果满足当前模式退出的条件，则退出当前模式选择其他的模式
		for(gimbal_behaviour_t *gbIterator = gimbalTaskStructure.behaviorNow;
			gbIterator != gimbalTaskStructure.behaviorList + GIMBAL_MODE_LENGTH;
			gbIterator++)
			if(gbIterator->enterBehaviorCondition != NULL && gbIterator->enterBehaviorCondition()){
				gimbalBehaviourChange(gbIterator);
				return;
			}
		
		//如果所有的条件都没有满足，那就切换到无力模式
		gimbalBehaviourChange(gimbalTaskStructure.behaviorList + GIMBAL_ZERO_FORCE);
	}	
}


void gimbalMotorCtrlChangeHandle()
{
	static u8 pitchLastMode = 255;
	static u8 yawLastMode = 255;
	
	//发生yaw/pitch电机控制方式的切换，要修改切换后的模式的期望值为当前的实际值，防止云台甩头
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


//规定反馈值的正方向为从操作手视角看，向右和向上为正, 角度的单位是度 速度的单位是度/S
void gimbalFeedbackUpdate(GimbalCtrl_t *gimbal_feedback_update)
{ 
    if (gimbal_feedback_update == NULL)
    {
        return;
    }
		/***Pitch轴***/
    //云台角度数据更新          
    gimbal_feedback_update->pitchMotor.angle[GYRO]    = gimbal_feedback_update->imu->pit; //对地位置获取（陀螺仪的值）
    gimbal_feedback_update->pitchMotor.angle[ENCONDE] = -motor_ecd_to_angle_change(gimbal_feedback_update->pitchMotor.base_inf.real_ecd, //相对位置获取相对于中值（offest—_ecd）编码器反馈值的误差
                                                                                   gimbal_feedback_update->pitchMotor.offsetEcd);
		//速度值都是用的陀螺仪的速度值
    gimbal_feedback_update->pitchMotor.speed[GYRO] =  (gimbal_feedback_update->imu->gyro[0]);
	  //读取陀螺仪的角速度
    gimbal_feedback_update->pitchMotor.speed[ENCONDE] = gimbal_feedback_update->pitchMotor.speed[GYRO];
	  /***Yaw轴***/
//		//云台角度数据更新   
//		gimbal_feedback_update->yawMotor.angle[GYRO] = -gimbal_feedback_update->imu->yaw;			//对地位置获取（陀螺仪的值）	
//    gimbal_feedback_update->yawMotor.angle[ENCONDE] = -motor_ecd_to_angle_change(gimbal_feedback_update->yawMotor.base_inf.real_ecd,	//相对位置获取相对于中值（offest—_ecd）编码器反馈值的误差
//                                                                                 gimbal_feedback_update->yawMotor.offsetEcd);
		//云台角度数据更新   
		gimbal_feedback_update->yawMotor.angle[GYRO] = -motor_ecd_to_angle_change(gimbal_feedback_update->yawMotor.base_inf.real_ecd,gimbal_feedback_update->yawMotor.offsetEcd);
    gimbal_feedback_update->yawMotor.angle[ENCONDE] = -motor_ecd_to_angle_change(gimbal_feedback_update->yawMotor.base_inf.real_ecd,	//相对位置获取相对于中值（offest—_ecd）编码器反馈值的误差
                                                                                 gimbal_feedback_update->yawMotor.offsetEcd);
		//速度值都是用的陀螺仪的速度值
    gimbal_feedback_update->yawMotor.speed[GYRO] 		= (gimbal_feedback_update->imu->gyro[2]) * 30;
		gimbal_feedback_update->yawMotor.speed[ENCONDE] = gimbal_feedback_update->yawMotor.speed[GYRO];
}


//计算相对角度
fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{ //将误差转换为最小的那方面的误差比如 当前实际值为8190，期望值为1，正常算出来的结果是8189，但是实际的差距应该是-2  
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


//函数名称：void gimbalBehaviourChange(gimbal_behaviour_t* next)
//函数作用：进行模式切换
//入口参数：下一个模式的结构体
//返回  值：无
void gimbalBehaviourChange(gimbal_behaviour_t* next)
{ 
	//进行切换,主要切换当前行为指针，当前行为步骤以及两个电机的控制方式
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




//函数名称：GimbalPidCalc(gimbal_motor_mode_e pitchCtrlType, gimbal_motor_mode_e yawCtrlType, gimbal_motor_t *pitchMotor, gimbal_motor_t *yawMotor)
//函数作用：对云台电机进行PID计算
//入口参数：Pit轴电机的模式和结构体，Yaw轴电机的模式和结构体
//返回  值：无
void GimbalPidCalc(gimbal_motor_mode_e pitchCtrlType, gimbal_motor_mode_e yawCtrlType, gimbal_motor_t *pitchMotor, gimbal_motor_t *yawMotor)
{
	if(pitchCtrlType == GIMBAL_MOTOR_RAW) //电流直接控制不需要做PID计算
		pitchMotor->currentSet = pitchMotor->rawCmdCurrent;
	else if(pitchCtrlType == GIMBAL_MOTOR_ENCONDE || pitchCtrlType == GIMBAL_MOTOR_GYRO)
	{ //陀螺仪或者编码器控制方式采用双环的控制方法
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


//函数名称：gimbalAngleLimit()
//函数作用：给云台两个电机进行限位
//入口参数：无
//返回  值：无
void gimbalAngleLimit()
{ 
	//yaw轴没有限位主要限制期望值不要超过实际值范围即可，主要是pitch限位
	gimbal_motor_mode_e yawMode = gimbalTaskStructure.yawMotor.mode;
	
	if(yawMode == GIMBAL_MOTOR_ENCONDE || yawMode == GIMBAL_MOTOR_GYRO)
		gimbalTaskStructure.yawMotor.angle_set[yawMode] = loop_fp32_constrain(gimbalTaskStructure.yawMotor.angle_set[yawMode],gimbalTaskStructure.yawMotor.minRelativeAngle, gimbalTaskStructure.yawMotor.maxRelativeAngle) - 30;
	//Pitch轴电机限位
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
		//如果是陀螺仪控制方法，需要根据当前陀螺仪反馈和编码器反馈的实际值，将原有的编码器上下限转化为陀螺仪实际值的上下限
		nowAngleSet = gimbalTaskStructure.pitchMotor.angle_set + GIMBAL_MOTOR_GYRO;
		maxRange = ABS_PITCH_MAX_RANGE;
		minRange = ABS_PITCH_MIN_RANGE;
		//计算当前位置到位置上限和位置下限的角度差
		float relativeMax, relativeMin;
		relativeMax = loop_fp32_constrain(gimbalTaskStructure.pitchMotor.maxRelativeAngle - gimbalTaskStructure.pitchMotor.angle[GIMBAL_MOTOR_ENCONDE],RELATIVE_PITCH_MIN_RANGE, RELATIVE_PITCH_MAX_RANGE);
		relativeMin = loop_fp32_constrain(gimbalTaskStructure.pitchMotor.minRelativeAngle - gimbalTaskStructure.pitchMotor.angle[GIMBAL_MOTOR_ENCONDE],RELATIVE_PITCH_MIN_RANGE, RELATIVE_PITCH_MAX_RANGE);
		//将角度差由编码器的量程调整为陀螺仪的量程
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
	{ //最大限位和最小限位之间有一个量程临界的跳变，需要特殊处理
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

//函数名称：GIMBAL_CanbusCtrlMotors(s16 pitchCurrent, s16 yawCurrent,s16 bomb_Curreent, s16 frCurrent, s16 plCurrent)
//函数作用：通过CAN给电机发送电流值
//入口参数：pitch yaw bomb，plate电流值
//返回  值：无
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

