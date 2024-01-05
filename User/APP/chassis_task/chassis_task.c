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


/*功率限制比例系数数组*/
//未装裁判系统的时候第一个数设置成1(上裁判系统之后把1改成0)
const float power_data[61]=
{
	   1  ,  0.0030  ,  0.0060  ,  0.0090  ,  0.0120  ,  0.0150  ,  0.0180  ,  0.0210  ,  0.0240  ,  0.0270  ,  0.0300   ,  0.0390  ,  
0.0480  ,  0.0570  ,  0.0660  ,  0.0750  ,  0.0840  ,  0.0930  ,  0.1020  ,  0.1110  ,  0.1200  ,  0.1460  ,  0.1720   ,  0.1980  ,
0.2240  ,  0.2500  ,  0.3000  ,  0.3500  ,  0.4000  ,  0.4500  ,  0.5000  ,  0.5500  ,  0.6000  ,  0.6500  ,  0.7000   ,  0.7500  ,
0.7760  ,  0.8020  ,  0.8280  ,  0.8540  ,  0.8800  ,  0.8890  ,  0.8980  ,  0.9070  ,  0.9160  ,  0.9250  ,  0.9340   ,  0.9430  ,
0.9520  ,  0.9610  ,  0.9700  ,  0.9730  ,  0.9760  ,  0.9790  ,  0.9820  ,  0.9850  ,  0.9880  ,  0.9910  ,  0.9940   ,  0.9970  ,  
1.0000 
}; 


/*结构体初始化和函数声明*/
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

//函数名称：chassisTask()
//函数作用：底盘任务主函数
//入口参数：无
//返回  值：无
void chassisTask(void *pvParameters)
{
	portTickType currentTime;
	//底盘初始化
	chassisTaskInit();
	//FreeRTOS操作系统相对延时函数，为其他任务初始化提供时间
	vTaskDelay(4);
	//获取当前时间，提供给绝对延时函数上
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
		
		//选择底盘控制模式
		chassisBehaviorSelect();
		//电机模式过渡
		chassisModeChangeHandle();
		//更新实际值，并对实际值进行滤波
		chassisFeedBackUpdate();
		/*****赋值vx,vy,vz三个速度******/
		//直接发电流模式
		if(chassisTaskStructure.behaviorNow->mode == CHASSIS_RAW) 
			chassisTaskStructure.behaviorNow->behaviorHandleFun(&(chassisTaskStructure.motor[CHASSIS_CM1].rawCmdCurrent),
																                          &(chassisTaskStructure.motor[CHASSIS_CM2].rawCmdCurrent),
															                          	&(chassisTaskStructure.motor[CHASSIS_CM3].rawCmdCurrent),
																                          &(chassisTaskStructure.motor[CHASSIS_CM4].rawCmdCurrent));
		
		//云台夹角模式或者vx，vy，vz模式
		else if(chassisTaskStructure.behaviorNow->mode == CHASSIS_ANGLE || chassisTaskStructure.behaviorNow->mode == CHASSIS_SPEED)
		{
			chassisTaskStructure.behaviorNow->behaviorHandleFun(chassisTaskStructure.relativeSpeedSet + VX, chassisTaskStructure.relativeSpeedSet + VY, chassisTaskStructure.relativeSpeedSet + VZ, NULL);
		}
		//对三个速度限幅
		chassisSpeedLimit();
		//更新遥控器(新值和旧值)
		rcDataCopy(&(chassisTaskStructure.rc.last));
		//vx vy相对位置转化，并进行pid计算
		chassisPidCalc();
		//功率信息更新
		chassisTaskStructure.powerLimit.kMaxOutput = power_data[*chassisTaskStructure.powerLimit.buffer];
		//功率限制
 	  chassisPowerLimt();
	 	//队列发送
		chassisCanbuscCtrlMotors(chassisTaskStructure.motor[CHASSIS_CM1].baseInf.given_current *chassisTaskStructure.powerLimit.kMaxOutput, 
		                         chassisTaskStructure.motor[CHASSIS_CM2].baseInf.given_current *chassisTaskStructure.powerLimit.kMaxOutput,
								             chassisTaskStructure.motor[CHASSIS_CM3].baseInf.given_current *chassisTaskStructure.powerLimit.kMaxOutput,
								             chassisTaskStructure.motor[CHASSIS_CM4].baseInf.given_current *chassisTaskStructure.powerLimit.kMaxOutput);
		chassisCanbuscCtrlMotors(chassisTaskStructure.motor[CHASSIS_CM1].baseInf.given_current, 
		                         chassisTaskStructure.motor[CHASSIS_CM2].baseInf.given_current,
								             chassisTaskStructure.motor[CHASSIS_CM3].baseInf.given_current,
								             chassisTaskStructure.motor[CHASSIS_CM4].baseInf.given_current);

		//freeRTOS操作系统绝对延时函数
		vTaskDelayUntil(&currentTime, CHASSIS_TASK_MS);
	}
}


//函数名称：chassisTaskInit()
//函数作用：底盘初始化
//入口参数：无
//返回  值：无
void chassisTaskInit()
{
	//底盘功率和缓冲能量填充
	chassisTaskStructure.powerLimit.power  = &BoardBLink.RefreeSyetem.power.chassis_power;     //底盘功率
	chassisTaskStructure.powerLimit.buffer = &BoardBLink.RefreeSyetem.power.chassis_power_buff;//缓冲能量剩
	//获取遥控器数据
	chassisTaskStructure.rc.now =get_remote_control_point();
	//初始化速度限制变量  400(最大为473)
	chassisTaskStructure.relativeSpeedMax[VX] = CHASSIS_NORMAL_MAX_VX;
	chassisTaskStructure.relativeSpeedMax[VY] = CHASSIS_NORMAL_MAX_VY;
	chassisTaskStructure.relativeSpeedMax[VZ] = CHASSIS_NORMAL_MAX_VZ;
  //初始化功率限制变量
	chassisTaskStructure.powerLimit.warnBuffer = CHASSIS_WARN_BUFF;
	chassisTaskStructure.powerLimit.maxOutput  = CHASSIS_NORMAL_MAX_OUTPUT;
	chassisTaskStructure.powerLimit.capState   = CLOSE;
	//初始化电机控制模式（默认直接发电流模式）
	chassisTaskStructure.mode     = CHASSIS_RAW;
	chassisTaskStructure.lastMode = CHASSIS_RAW;
	//初始化底盘四个电机的减速比 19  3508电机
	chassisTaskStructure.motor[CHASSIS_CM1].baseInf.reduction_ratio = CHASSIS_MOTOR_REDUCTION_RATIO;
	chassisTaskStructure.motor[CHASSIS_CM2].baseInf.reduction_ratio = CHASSIS_MOTOR_REDUCTION_RATIO;
	chassisTaskStructure.motor[CHASSIS_CM3].baseInf.reduction_ratio = CHASSIS_MOTOR_REDUCTION_RATIO;
	chassisTaskStructure.motor[CHASSIS_CM4].baseInf.reduction_ratio = CHASSIS_MOTOR_REDUCTION_RATIO;
  //底盘跟随云台模式的位置式PID
	PIDInit(&(chassisTaskStructure.anglePidParameter), 6, 0, 10, 0.8, CHASSIS_FOLLOW_MAX_VZ, 0, 4.4, chassisTaskStructure.maxRelativeAngle, -1, POSITION_180);
  //底盘四个电机的速度单环PID
	PIDInit(&(chassisTaskStructure.motor[CHASSIS_CM1].pidParameter), 60, 0.25, 100, 0, 16000, -1, 500/19, -1, -1, SPEED);
	PIDInit(&(chassisTaskStructure.motor[CHASSIS_CM2].pidParameter), 60, 0.25, 100, 0, 16000, -1, 500/19, -1, -1, SPEED);
	PIDInit(&(chassisTaskStructure.motor[CHASSIS_CM3].pidParameter), 60, 0.25, 100, 0, 16000, -1, 500/19, -1, -1, SPEED);
	PIDInit(&(chassisTaskStructure.motor[CHASSIS_CM4].pidParameter), 60, 0.25, 100, 0, 16000, -1, 500/19, -1, -1, SPEED);
	//创建对应的卡尔曼滤波
	KalmanCreate(&(chassisTaskStructure.motor[CHASSIS_CM1].klmFiller), 1, 40);
	KalmanCreate(&(chassisTaskStructure.motor[CHASSIS_CM2].klmFiller), 1, 40);
	KalmanCreate(&(chassisTaskStructure.motor[CHASSIS_CM3].klmFiller), 1, 40);
	KalmanCreate(&(chassisTaskStructure.motor[CHASSIS_CM4].klmFiller), 1, 40);
	//设置底盘跟随模式的相关参数
	chassisTaskStructure.relativeAngle    = gimbalTaskStructure.yawMotor.angle + ENCONDE;//编码器模式的角度
	chassisTaskStructure.maxRelativeAngle = RELATIVE_YAW_MAX_RANGE;
	chassisTaskStructure.minRelativeAngle = RELATIVE_YAW_MIN_RANGE;
	/*给每一个模式配备相应的处理函数*/
	//DEBUG模式
	chassisBehaviourInit(chassisTaskStructure.behaviorList + CHASSIS_DEBUG     ,CHASSIS_DEBUG     ,DEBUG_CHSSIS_TYPE,enterChassisDebugCondition ,outChassisDebugCondition ,NULL,NULL,chassisDebugHandleFun);
	//无力模式
	chassisBehaviourInit(chassisTaskStructure.behaviorList + CHASSIS_ZERO_FORCE,CHASSIS_ZERO_FORCE,CHASSIS_RAW      ,enterZeroForceCondition    ,outZeroForceCondition    ,NULL,NULL,zeroForceHandleFun);
	//初始化模式 
	chassisBehaviourInit(chassisTaskStructure.behaviorList + CHASSIS_INIT      ,CHASSIS_INIT      ,CHASSIS_RAW      ,enterChassisInitCondition  ,outChassisInitCondition  ,NULL,NULL,chassisInitHandleFun);
  //陀螺模式
	chassisBehaviourInit(chassisTaskStructure.behaviorList + CHASSIS_ROTATE    ,CHASSIS_ROTATE    ,CHASSIS_SPEED    ,enterChassisRotateCondition,outChassisRotateCondition,NULL,NULL,chassisRotateHandleFun);
	//独立不跟随模式
	chassisBehaviourInit(chassisTaskStructure.behaviorList + CHASSIS_ALONE     ,CHASSIS_ALONE     ,CHASSIS_SPEED    ,enterChassisAloneCondition ,outChassisAloneCondition ,NULL,NULL,chassisAloneHandleFun);
	//跟随模式
	chassisBehaviourInit(chassisTaskStructure.behaviorList + CHASSIS_FOLLOW    ,CHASSIS_FOLLOW    ,CHASSIS_ANGLE    ,enterChassisFollowCondition,outChassisFollowCondition,NULL,NULL,chassisFollowHandleFun);
	//默认为无力模式
	chassisTaskStructure.behaviorNow = chassisTaskStructure.behaviorList + CHASSIS_ZERO_FORCE;
}


//函数名称：chassisBehaviourInit()
//函数作用：填充initBehavior结构体
//入口参数：模式，条件，处理函数
//返回  值：无
//bool_t (*enterBehaviorCondition)(void)，为一个函数指针，指向一个函数，返回了一个bool_t的值
//函数不加地址指的是函数的地址
void chassisBehaviourInit(chassis_behaviour_t *initBehavior, chassis_behaviour_e num, chassis_mode_e mode, 	bool_t (*enterBehaviorCondition)(void),
								          bool_t (*outBehaviorCondition)(void), void (*enterBehaviorFun)(void), void (*outBehaviorFun)(void), void (*behaviorHandleFun)(float* vx, float* vy, float* vz, float* raw))
{
	if(initBehavior == NULL || num < 0 || num >= CHASSIS_BEHAVOUR_LENGTH || enterBehaviorCondition == NULL || outBehaviorCondition == NULL || behaviorHandleFun == NULL)
	return;
	//数据填充
	initBehavior->num = num;
	initBehavior->behaviorHandleFun = behaviorHandleFun;
	initBehavior->enterBehaviorCondition = enterBehaviorCondition;
	initBehavior->outBehaviorCondition = outBehaviorCondition;
	initBehavior->enterBehaviorFun = enterBehaviorFun;
	initBehavior->outBehaviorFun = outBehaviorFun;
	initBehavior->mode = mode;
}


//函数名称：chassisBehaviorSelect()
//函数作用：底盘控制切换模式函数
//入口参数：无
//返回  值：无
void chassisBehaviorSelect()
{
	//查看有没有优先级比当行为高的行为的进入条件得到满足
	for(chassis_behaviour_t *iterator = chassisTaskStructure.behaviorList;iterator<chassisTaskStructure.behaviorNow;iterator++)
	{ 
		//如果有优先级高的进入行为得到满足，则进入那个行为的模式
		if(iterator->enterBehaviorCondition())
		{
			chassisBehaviorChange(iterator);
			break;
		}
	}
	//查看当前行为是否已经达到了退出的条件，如果达到，则寻找下一个可以进入的行为
	if(chassisTaskStructure.behaviorNow->outBehaviorCondition())
	{ 
		for(chassis_behaviour_t *iterator=chassisTaskStructure.behaviorNow;iterator<chassisTaskStructure.behaviorList+CHASSIS_BEHAVOUR_LENGTH;iterator++)
		{
			//按照优先级，如果有满足的行为，那么进入这个行为
			if(iterator->enterBehaviorCondition())
			{
				chassisBehaviorChange(iterator);
				return;
			}
		}
	}
	else
		return;
	//如果没有满足所有行为的进入条件，那么底盘进入无力模式
	chassisBehaviorChange(chassisTaskStructure.behaviorList + CHASSIS_ZERO_FORCE);
}


//函数名称；chassisBehaviorChange()
//函数作用：更换底盘模式
//入口参数：下一个电机模式
//返回  值：无
void chassisBehaviorChange(chassis_behaviour_t *next)
{
	//如果有退出该行为的处理函数，则执行
	if(chassisTaskStructure.behaviorNow->outBehaviorFun != NULL)
	   chassisTaskStructure.behaviorNow->outBehaviorFun();
	//将当前行为切换成函数传入的行为
	chassisTaskStructure.behaviorNow   = next;
	chassisTaskStructure.behaviourStep = 0;
	chassisTaskStructure.mode = chassisTaskStructure.behaviorNow->mode;
	//如果有进入该行为的处理函数，则执行
	if(chassisTaskStructure.behaviorNow->enterBehaviorFun != NULL)
		 chassisTaskStructure.behaviorNow->enterBehaviorFun();
}


//函数名称：chassisModeChangeHandle()
//函数作用：底盘电机模式过渡
//入口参数：无
//返回  值：无
void chassisModeChangeHandle()
{
	//判断电机模式是否发生了改变
	if(chassisTaskStructure.mode != chassisTaskStructure.lastMode)
	{ 
		//将上次模式赋值给lastMode
		chassisTaskStructure.lastMode = chassisTaskStructure.mode;
		//切换到跟随模式的时候，将当前的角度偏差作为期望，防止切换到这个模式后底盘甩头
		if(chassisTaskStructure.mode == CHASSIS_ANGLE)
			chassisTaskStructure.relativeAngleSet = *(chassisTaskStructure.relativeAngle);
	}
}


//函数名称：chassisSpeedLimit()
//函数作用：底盘电机速度限幅
//入口参数：无
//返回  值：无
void chassisSpeedLimit()
{
	//电容管理
	if((IF_KEY_PRESSED_SHIFT)||(IF_KEY_PRESSED_CTRL)||( IF_RC_SW1_UP && IF_RC_SW2_DOWN ))
	  chassisTaskStructure.if_want_to_open_cap = CAP_CONFIG; 
	/*****输出限幅*****/
	//如果电容开启，输出幅度升高
	if(chassisTaskStructure.if_want_to_open_cap == CAP_CONFIG)
	{
		chassisTaskStructure.powerLimit.maxOutput = CHASSIS_CAP_MAX_OUTPUT;	
		chassisTaskStructure.relativeSpeedMax[VX] = CHASSIS_CAP_MAX_VX;
		chassisTaskStructure.relativeSpeedMax[VY] = CHASSIS_CAP_MAX_VY;	
	}
	//如果电容不开启，输出限幅不升高
	else
	{
		chassisTaskStructure.powerLimit.maxOutput = CHASSIS_NORMAL_MAX_OUTPUT;	
		chassisTaskStructure.relativeSpeedMax[VX] = CHASSIS_NORMAL_MAX_VX;
		chassisTaskStructure.relativeSpeedMax[VY] = CHASSIS_NORMAL_MAX_VY;
	}
	//限制旋转方向的输出
	chassisTaskStructure.relativeSpeedSet[VZ] = fp32_constrain(chassisTaskStructure.relativeSpeedSet[VZ], -chassisTaskStructure.relativeSpeedMax[VZ], chassisTaskStructure.relativeSpeedMax[VZ]);
	//如果旋转方向的输出很大，限制平移速度防止转弯半径过大
//	if(f_abs(chassisTaskStructure.relativeSpeedSet[VZ]) > WARN_SPEED_VZ)
	if(0)
	{
		float k = (f_abs(chassisTaskStructure.chassisSpeedSet[VZ]) - WARN_SPEED_VZ)*(f_abs(chassisTaskStructure.chassisSpeedSet[VZ]) - WARN_SPEED_VZ)/(chassisTaskStructure.relativeSpeedMax[VZ]*chassisTaskStructure.relativeSpeedMax[VZ]);
		k = fp32_constrain(k, 0, 1);
		chassisTaskStructure.relativeSpeedMax[VY] *= k;
		chassisTaskStructure.relativeSpeedMax[VX] *= k;
	}
	//限制平移方向的输出
	chassisTaskStructure.relativeSpeedSet[VX] = fp32_constrain(chassisTaskStructure.relativeSpeedSet[VX], -chassisTaskStructure.relativeSpeedMax[VX], chassisTaskStructure.relativeSpeedMax[VX]);
	chassisTaskStructure.relativeSpeedSet[VY] = fp32_constrain(chassisTaskStructure.relativeSpeedSet[VY], -chassisTaskStructure.relativeSpeedMax[VY], chassisTaskStructure.relativeSpeedMax[VY]);
}


//函数名称：chassisFeedBackUpdate()
//函数作用：更新实际值，并对实际值进行滤波
//入口参数：无
//返回  值：无
void chassisFeedBackUpdate()
{
	chassisTaskStructure.motor[CHASSIS_CM1].filterSpeed = KalmanFilter(&(chassisTaskStructure.motor[CHASSIS_CM1].klmFiller), chassisTaskStructure.motor[CHASSIS_CM1].baseInf.real_speed_rpm);
	chassisTaskStructure.motor[CHASSIS_CM2].filterSpeed = KalmanFilter(&(chassisTaskStructure.motor[CHASSIS_CM2].klmFiller), chassisTaskStructure.motor[CHASSIS_CM2].baseInf.real_speed_rpm);
	chassisTaskStructure.motor[CHASSIS_CM3].filterSpeed = KalmanFilter(&(chassisTaskStructure.motor[CHASSIS_CM3].klmFiller), chassisTaskStructure.motor[CHASSIS_CM3].baseInf.real_speed_rpm);
	chassisTaskStructure.motor[CHASSIS_CM4].filterSpeed = KalmanFilter(&(chassisTaskStructure.motor[CHASSIS_CM4].klmFiller), chassisTaskStructure.motor[CHASSIS_CM4].baseInf.real_speed_rpm);
}



//函数名称：chassisPidCalc()
//函数作用：vx vy相对位置转化，并进行pid计算
//入口参数：无
//返回  值：无
void chassisPidCalc()
{	
	//如果是直接给电机发送电流，不需要进行PID计算，直接发送即可
	if(chassisTaskStructure.mode == CHASSIS_RAW)
	{ 
		for(u8 i = CHASSIS_CM1; i <= CHASSIS_CM4; i++)
		{
			chassisTaskStructure.motor[i].currentSet = chassisTaskStructure.motor[i].rawCmdCurrent;
		}
	}
	//如果是云台夹角模式和vx，vy，vz模式，需要进行PID计算
	else if(chassisTaskStructure.mode == CHASSIS_ANGLE || chassisTaskStructure.mode == CHASSIS_SPEED)
	{
		//获取云台底盘相对角度(弧度制)
		float relationAngle = (*(chassisTaskStructure.relativeAngle)+125)/180.0*PI;
		//底盘速度分解
		chassisTaskStructure.chassisSpeedSet[VX] = cos(relationAngle)*chassisTaskStructure.relativeSpeedSet[VX] - sin(relationAngle)*chassisTaskStructure.relativeSpeedSet[VY];
		chassisTaskStructure.chassisSpeedSet[VY] = sin(relationAngle)*chassisTaskStructure.relativeSpeedSet[VX] + cos(relationAngle)*chassisTaskStructure.relativeSpeedSet[VY];
		chassisTaskStructure.chassisSpeedSet[VZ] = chassisTaskStructure.relativeSpeedSet[VZ];
		//将三个方向的速度分解到麦轮上
		chassisTaskStructure.motor[CHASSIS_CM1].speedSet = +chassisTaskStructure.chassisSpeedSet[VX] - chassisTaskStructure.chassisSpeedSet[VY] - chassisTaskStructure.chassisSpeedSet[VZ];
		chassisTaskStructure.motor[CHASSIS_CM2].speedSet = +chassisTaskStructure.chassisSpeedSet[VX] + chassisTaskStructure.chassisSpeedSet[VY] - chassisTaskStructure.chassisSpeedSet[VZ];
		chassisTaskStructure.motor[CHASSIS_CM3].speedSet = -chassisTaskStructure.chassisSpeedSet[VX] + chassisTaskStructure.chassisSpeedSet[VY] - chassisTaskStructure.chassisSpeedSet[VZ];
		chassisTaskStructure.motor[CHASSIS_CM4].speedSet = -chassisTaskStructure.chassisSpeedSet[VX] - chassisTaskStructure.chassisSpeedSet[VY] - chassisTaskStructure.chassisSpeedSet[VZ];
		//斜坡使实际值缓慢靠近速度期望值
		RAMP_float(chassisTaskStructure.motor[CHASSIS_CM1].speedSet,chassisTaskStructure.motor[CHASSIS_CM1].filterSpeed,100);	
		RAMP_float(chassisTaskStructure.motor[CHASSIS_CM2].speedSet,chassisTaskStructure.motor[CHASSIS_CM2].filterSpeed,100);	
		RAMP_float(chassisTaskStructure.motor[CHASSIS_CM3].speedSet,chassisTaskStructure.motor[CHASSIS_CM3].filterSpeed,100);	
	  RAMP_float(chassisTaskStructure.motor[CHASSIS_CM4].speedSet,chassisTaskStructure.motor[CHASSIS_CM4].filterSpeed,100);	
		//速度环PID的计算部分
		PID_Calc(&(chassisTaskStructure.motor[CHASSIS_CM1].pidParameter), chassisTaskStructure.motor[CHASSIS_CM1].filterSpeed,chassisTaskStructure.motor[CHASSIS_CM1].speedSet);
		PID_Calc(&(chassisTaskStructure.motor[CHASSIS_CM2].pidParameter), chassisTaskStructure.motor[CHASSIS_CM2].filterSpeed,chassisTaskStructure.motor[CHASSIS_CM2].speedSet);
		PID_Calc(&(chassisTaskStructure.motor[CHASSIS_CM3].pidParameter), chassisTaskStructure.motor[CHASSIS_CM3].filterSpeed,chassisTaskStructure.motor[CHASSIS_CM3].speedSet);
		PID_Calc(&(chassisTaskStructure.motor[CHASSIS_CM4].pidParameter), chassisTaskStructure.motor[CHASSIS_CM4].filterSpeed,chassisTaskStructure.motor[CHASSIS_CM4].speedSet);
		//将PID的计算结果赋值给电机
		for(u8 i = CHASSIS_CM1;i <= CHASSIS_CM4; i++)
		{
			chassisTaskStructure.motor[i].currentSet = chassisTaskStructure.motor[i].pidParameter.output;
		}		
	}
  //如果无模式选择，将四个轮子的电流值设置为0
	else
	{
		for(u8 i = CHASSIS_CM1; i <= CHASSIS_CM4; i++)
		{
			chassisTaskStructure.motor[i].currentSet = 0;
		}			
	}
	//限制四个PID的output
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



//功率限制函数
void chassisPowerLimt()//功率限制2022赛季，待测试
{
	float WARNING_REMAIN_POWER=60;
	
	float Joule_Residue=0;//剩余功率缓冲量 
	
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

