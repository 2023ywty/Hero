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

/*结构体和函数声明*/
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
					
	float view_shoot_left, view_shoot_right, view_shoot_plate; //云台观测
	float *flExp,*frExp,*plExp;

//发射任务主函数
void shootTask(void *pvParameters)
{
	//发射相关初始化
	shootInit();
	vTaskDelay(1000);
	TickType_t shootDelayTick = xTaskGetTickCount();
	while(1)
	{
		//云台观测
		view_shoot_left = shootTaskStructure.leftFirMotor.speed;
		view_shoot_right = shootTaskStructure.rightFirMotor.speed;
		view_shoot_plate = shootTaskStructure.plateMotor.base_inf.real_speed_rpm;
		
		//选择当前发射的模式
		shootBehaviourSelect();
		//发生控制方式切换进行清0
		shootCtrlChangeHandle();
		//实际数据更新
		shootFeedbackUpdata();

			plExp = &(shootTaskStructure.shootHzSet);
			flExp = &(shootTaskStructure.leftFirMotor.speedSet);
			frExp = &(shootTaskStructure.rightFirMotor.speedSet);

		//给处理函数赋值
		shootTaskStructure.nowBeh->behaviorHandleFun(flExp, frExp, plExp);
		//更新遥控器参数
		rcDataCopy(&(shootTaskStructure.rc.last));
		//热量限制
		shootHeatLimt();
		//PID计算
		shootPidCalc();	
		
		//加入队列进行发送
		GIMBAL_CanbusCtrlMotors(gimbalTaskStructure.pitchMotor.base_inf.given_current, 
														-gimbalTaskStructure.yawMotor.  base_inf.given_current,
														shootTaskStructure. plateMotor.base_inf.given_current,
		                        shootTaskStructure. leftFirMotor.base_inf.given_current, 
		                        shootTaskStructure. rightFirMotor.base_inf.given_current);
		
		vTaskDelayUntil(&shootDelayTick, SHOOT_TASK_MS);
	}
}

//发射任务初始化
void shootInit()
{
	//拨盘相关参数初始化
	shootPlateReset();
	//获取遥控器的值
	shootTaskStructure.rc.now = get_remote_control_point();
	//电机减速比设置
	motorInit(&(shootTaskStructure.leftFirMotor.base_inf),  1);
	motorInit(&(shootTaskStructure.rightFirMotor.base_inf), 1);
	motorInit(&(shootTaskStructure.plateMotor.base_inf),   19);
	KalmanCreate(&(shootTaskStructure.leftFirMotor.klmFiller), 1, 5);
	KalmanCreate(&(shootTaskStructure.rightFirMotor.klmFiller), 1, 5);
	
	//拨盘发射相关参数
	shootTaskStructure.plateMotor.teethNum = 6;
	shootTaskStructure.plateMotor.shootcontinue = 1;
	shootTaskStructure.plateMotor.ifSensorOnLine=1;
	//左右摩擦轮半径设置
	shootTaskStructure.rightFirMotor.radius = 30;
	shootTaskStructure.leftFirMotor.radius  = 30;
	//PID初始化
	shootPidInit();
	HEAD_CLOSE();
	Electricpush2_off();
	Electricpush_on();
	//行为初始化
	shootBehInit(shootTaskStructure.behList + SHOOT_DEBUG, DEBUG_SHOOT_LF_TYPE, DEBUG_SHOOT_RF_TYPE, DEBUG_SHOOT_PL_TYPE, SHOOT_DEBUG, shootbehDebugHandleFun, shootbehDebugEnterCondition, shootbehDebugOutCondition, NULL, NULL);
	shootBehInit(shootTaskStructure.behList + SHOOT_ZERO_FORCE, SHOOT_MOTOR_RAW, SHOOT_MOTOR_RAW, SHOOT_MOTOR_RAW, SHOOT_ZERO_FORCE, shootbehZeroForceHandleFun, shootbehZeroForceEnterCondition, shootbehZeroForceOutCondition, NULL, NULL);
	//默认无力模式
	shootBehChange(shootTaskStructure.behList + SHOOT_ZERO_FORCE);
}



//卡弹反转函数，没太看懂
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


//模式切换函数
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



//模式配备函数
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


//拨盘相关参数初始化函数
void shootPlateReset()
{
	shootTaskStructure.shootHzSet = 0;
	shootTaskStructure.plateMotor.speed_set = 0;
	shootTaskStructure.plateMotor.position_set = shootTaskStructure.plateMotor.base_inf.real_ecd;
	shootTaskStructure.plateMotor.rawCmdCurrent = 0;
	shootTaskStructure.shootNumSet = 0;
}


//实际值更新函数
void shootFeedbackUpdata()
{
	//将摩擦轮转速rpm转化为摩擦轮边线速度（卡尔曼滤波）
	shootTaskStructure.leftFirMotor.filterSpeed = KalmanFilter(&(shootTaskStructure.leftFirMotor.klmFiller), shootTaskStructure.leftFirMotor.base_inf.real_speed_rpm);
	shootTaskStructure.rightFirMotor.filterSpeed = KalmanFilter(&(shootTaskStructure.rightFirMotor.klmFiller), shootTaskStructure.rightFirMotor.base_inf.real_speed_rpm);
	shootTaskStructure.leftFirMotor.speed  = PI/1000.0*shootTaskStructure.leftFirMotor.filterSpeed;
	shootTaskStructure.rightFirMotor.speed = PI/1000.0*shootTaskStructure.rightFirMotor.filterSpeed;
}


//PID初始化函数
void shootPidInit()
{
	//拨盘速度环&位置环
	PIDInit(&(shootTaskStructure.plateMotor.PIDParameter[INNER]), 130, 1.2, 0, 0, 10000, 50/36.0, 3100/36.0, -1, 5000, SPEED);
	PIDInit(&(shootTaskStructure.plateMotor.PIDParameter[OUTER]),   5, 0, 0, 0, -1, 0, -1, 360 , -1, POSITION_360);
	//左右摩擦轮电机速度环
	PIDInit(&(shootTaskStructure.leftFirMotor.PIDParameter)	, 1500	, 5.00f, 4000	, 0, 16000, 0, 15, 29, 2000, SPEED);
	PIDInit(&(shootTaskStructure.rightFirMotor.PIDParameter), 1800	, 5.00f, 4000	, 0, 16000, 0, 15, 29, 2000, SPEED);
}


//编码器差值 优弧转反向劣弧
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


//PID的计算函数
void shootPidCalc()
{
	//不懂1800是什么意思
	float realPos = motor_ecd_to_angle_change(shootTaskStructure.plateMotor.base_inf.real_ecd,1800);
	float expPos  = motor_ecd_to_angle_change(shootTaskStructure.plateMotor.position_set,     1800);
	//拨盘电机位置环控制
	if(shootTaskStructure.plateMotor.mode == SHOOT_MOTOR_POSITION)
	{
		//位置环计算
		PID_Calc(shootTaskStructure.plateMotor.PIDParameter + OUTER, realPos, expPos);
		//位置环输出赋值
		shootTaskStructure.plateMotor.speed_set = shootTaskStructure.plateMotor.PIDParameter[OUTER].output;
		//卡单反转
		plateBackSpeed(&(shootTaskStructure.plateMotor.speed_set), shootTaskStructure.plateMotor.base_inf.real_speed_rpm, SHOOT_TASK_MS);
		//速度环控制
		PID_Calc(shootTaskStructure.plateMotor.PIDParameter + INNER, shootTaskStructure.plateMotor.base_inf.real_speed_rpm, shootTaskStructure.plateMotor.speed_set);
		//速度环输出赋值
		shootTaskStructure.plateMotor.currentSet = shootTaskStructure.plateMotor.PIDParameter[INNER].output;
	}
	//拨盘电机速度环控制
	else if(shootTaskStructure.plateMotor.mode == SHOOT_MOTOR_SPEED)
	{
		//射频转化为拨盘转速
		shootTaskStructure.plateMotor.speed_set = shootTaskStructure.shootHzSet*60/shootTaskStructure.plateMotor.teethNum;
		//卡单反转
		plateBackSpeed(&(shootTaskStructure.plateMotor.speed_set), shootTaskStructure.plateMotor.base_inf.real_speed_rpm, SHOOT_TASK_MS);
		//速度环控制
		PID_Calc(shootTaskStructure.plateMotor.PIDParameter + INNER, shootTaskStructure.plateMotor.base_inf.real_speed_rpm, shootTaskStructure.plateMotor.speed_set);
		//速度环输出赋值
		shootTaskStructure.plateMotor.currentSet = shootTaskStructure.plateMotor.PIDParameter[INNER].output;		
	}
	//电流模式
	else if(shootTaskStructure.plateMotor.mode == SHOOT_MOTOR_RAW)
	{
		shootTaskStructure.plateMotor.currentSet = shootTaskStructure.plateMotor.rawCmdCurrent;
	}
	//限制拨盘电机电流输出
	abs_limit(&(shootTaskStructure.plateMotor.currentSet), 10000);
	shootTaskStructure.plateMotor.base_inf.given_current = (s16)shootTaskStructure.plateMotor.currentSet;
	//摩擦轮线速度PID计算
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
	//限制电流值
	abs_limit(&(shootTaskStructure.rightFirMotor.currentSet), 16384);
	abs_limit(&(shootTaskStructure.leftFirMotor.currentSet),  16384);
  //电流值赋值
	shootTaskStructure.rightFirMotor.base_inf.given_current = (s16)shootTaskStructure.rightFirMotor.currentSet;	
	shootTaskStructure.leftFirMotor.base_inf.given_current  = (s16)shootTaskStructure.leftFirMotor.currentSet;
}


//模式切换函数
void shootBehaviourSelect()
{
	for(shoot_behaviour_t *iterator = shootTaskStructure.behList; iterator < shootTaskStructure.nowBeh; iterator++)
	{ //查看优先级比当前行为高的行为的进入条件是否满足，满足则进入
		if(iterator->enterBehaviorCondition != NULL && iterator->enterBehaviorCondition())
		{
			shootBehChange(iterator);
			return;
		}
	}
	if(shootTaskStructure.nowBeh->outBehaviorCondition != NULL && shootTaskStructure.nowBeh->outBehaviorCondition())
	{ //如果当前行为达到退出条件，就寻找满足进入条件的行为，并进入
		for(shoot_behaviour_t *iterator = shootTaskStructure.nowBeh + 1; iterator < shootTaskStructure.behList + SHOOT_MODE_LENGTH; iterator++)
		{
			if(iterator->enterBehaviorCondition != NULL && iterator->enterBehaviorCondition())
			{
				shootBehChange(iterator);
				return;
			}
		}
		//如果所有behaviour的进入条件都没有达到就进入无力模式
		shootBehChange(shootTaskStructure.behList + SHOOT_ZERO_FORCE);
	}
}


//模式切换的处理函数
void shootCtrlChangeHandle()
{
	static shoot_motor_mode_e lastPlateMode;
	
	if(lastPlateMode != shootTaskStructure.plateMotor.mode)  //拨盘上次的模式和这次的模式不同
	{
		if(shootTaskStructure.plateMotor.mode == SHOOT_MOTOR_POSITION)
		{
			shootTaskStructure.plateMotor.position_set = shootTaskStructure.plateMotor.base_inf.real_ecd;  //将现在的编码器值变成期望值
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
  

//热量限制函数
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
	shootTaskStructure.heatLimit.heatNow=BoardBLink.RefreeSyetem.shoot.s16_Shoot_heat;//当前热量
	if(shootTaskStructure.heatLimit.maxHeat-shootTaskStructure.heatLimit.heatNow<100)
  {			 
				shootTaskStructure.plateMotor.position_set= shootTaskStructure.plateMotor.base_inf.real_ecd;//position下期望值为当前值	
	}
}

