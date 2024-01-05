#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "Remote_Control.h"
#include "IMU_onBoard.h"
#include "pid.h"
#include "robot.h"
#include "motor.h"
#include "kalman_filter.h"
#include "stm32f4xx.h"
#include "user_lib.h"

#define RELATIVE_PITCH_ANGLE_MAX  72
#define RELATIVE_PITCH_ANGLE_MIN -68

//#define GIMBAL_PITCH_MOTOR_OFFSET_ECD  2705  //4100.3463  //仰角最高
#define GIMBAL_PITCH_MOTOR_OFFSET_ECD  2628 //4100.3463 //仰角水平
#define GIMBAL_YAW_MOTOR_OFFSET_ECD    2600

#define INNER 	0	
#define OUTER 	1
#define GYRO 		0
#define ENCONDE 1


#define GIMBAL_CANBUS_SEND_HEADER 0x1FF
//电机编码值转化成角度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

#ifndef MOTOR_ECD_TO_360
#define MOTOR_ECD_TO_360 0.04395068f //      2*  PI  /8192
#endif

#ifndef RAD_TO_360
#define RAD_TO_360 57.3
#endif

#ifndef _360_TO_RAD
#define _360_TO_RAD 0.01745329
#endif


#ifndef PI
#define PI 3.1415926
#endif

#ifndef RPM_TO_360_s
#define RPM_TO_360_s 6.0
#endif

//此处为传感器的量程
#define ABS_YAW_MAX_RANGE 180
#define ABS_YAW_MIN_RANGE -180
#define ABS_PITCH_MAX_RANGE 180
#define ABS_PITCH_MIN_RANGE -180
#define RELATIVE_PITCH_MAX_RANGE 180
#define RELATIVE_PITCH_MIN_RANGE -180
#define RELATIVE_YAW_MAX_RANGE 180
#define RELATIVE_YAW_MIN_RANGE -180

//电机码盘值最大以及中值
#define HALF_ECD_RANGE 4096
#define MAX_ECD_RANGE 8191

//下降沿触发宏定义 其他遥控器相关的宏定义在Remote_Control.h
#define GIMBAL_FIRST_PRESS_W		!IF_LAST_KEY_PRESSED_W(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_W
#define GIMBAL_FIRST_PRESS_S		!IF_LAST_KEY_PRESSED_S(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_S
#define GIMBAL_FIRST_PRESS_A		!IF_LAST_KEY_PRESSED_A(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_A
#define GIMBAL_FIRST_PRESS_D		!IF_LAST_KEY_PRESSED_D(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_D
#define GIMBAL_FIRST_PRESS_Q		!IF_LAST_KEY_PRESSED_Q(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_Q     //空闲
#define GIMBAL_FIRST_PRESS_E		!IF_LAST_KEY_PRESSED_E(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_E
#define GIMBAL_FIRST_PRESS_G		!IF_LAST_KEY_PRESSED_G(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_G     //空闲
#define GIMBAL_FIRST_PRESS_X		!IF_LAST_KEY_PRESSED_X(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_X     //空闲
#define GIMBAL_FIRST_PRESS_Z		!IF_LAST_KEY_PRESSED_Z(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_Z     //空闲
#define GIMBAL_FIRST_PRESS_C		!IF_LAST_KEY_PRESSED_C(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_C     //空闲
#define GIMBAL_FIRST_PRESS_B		!IF_LAST_KEY_PRESSED_B(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_B     //空闲
#define GIMBAL_FIRST_PRESS_V		!IF_LAST_KEY_PRESSED_V(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_V     //空闲
#define GIMBAL_FIRST_PRESS_F		!IF_LAST_KEY_PRESSED_F(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_F     //空闲
#define GIMBAL_FIRST_PRESS_R		!IF_LAST_KEY_PRESSED_R(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_R     
#define GIMBAL_FIRST_PRESS_CTRL		!IF_LAST_KEY_PRESSED_CTRL(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_CTRL
#define GIMBAL_FIRST_PRESS_SHIFT	!IF_LAST_KEY_PRESSED_SHIFT(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_SHIFT

#define GIMBAL_FIRST_S1_UP			IF_RC_SW1_UP && (!(gimbalTaskStructure.rc.last.rc.s[0] == 1))
#define GIMBAL_FIRST_S1_DOWN		IF_RC_SW1_DOWN && (!(gimbalTaskStructure.rc.last.rc.s[0] == 2))
#define GIMBAL_FIRST_S1_MID			IF_RC_SW1_MID && (!(gimbalTaskStructure.rc.last.rc.s[0] == 3))

#define GIMBAL_FIRST_S2_UP			(IF_RC_SW2_UP && (!(gimbalTaskStructure.rc.last.rc.s[1] == 1)))
#define GIMBAL_FIRST_S2_DOWN		(IF_RC_SW2_DOWN && (!(gimbalTaskStructure.rc.last.rc.s[1] == 2)))
#define GIMBAL_FIRST_S2_MID			(IF_RC_SW2_MID && (!(gimbalTaskStructure.rc.last.rc.s[1] == 3)))

#define GIMBAL_FIRST_MOUSE_X_STOP	MOUSE_X_MOVE_SPEED == 0 && gimbalTaskStructure.rc.last.mouse.x != 0
#define GIMBAL_FIRST_MOUSE_Y_STOP	MOUSE_Y_MOVE_SPEED == 0 && gimbalTaskStructure.rc.last.mouse.y != 0

#define GIMBAL_FIRST_CH0_MID		RC_CH0_RLR_OFFSET == 0 && gimbalTaskStructure.rc.last.rc.ch[0] != 0
#define GIMBAL_FIRST_CH1_MID		RC_CH1_RUD_OFFSET == 0 && gimbalTaskStructure.rc.last.rc.ch[1] != 0
#define GIMBAL_FIRST_CH2_MID		RC_CH2_LLR_OFFSET == 0 && gimbalTaskStructure.rc.last.rc.ch[2] != 0
#define GIMBAL_FIRST_CH3_MID		RC_CH3_LUD_OFFSET == 0 && gimbalTaskStructure.rc.last.rc.ch[3] != 0



/*二阶卡尔曼*/
#define KF_ANGLE	0
#define KF_SPEED	1
#define KF_ACCEL	2


typedef enum
{
  GIMBAL_MOTOR_GYRO = 0,   	  //电机陀螺仪角度控制
  GIMBAL_MOTOR_ENCONDE, 		  //电机编码值角度控制
	GIMBAL_MOTOR_GYRO_SPEED,	  //陀螺仪速度控制
	GIMBAL_MOTOR_ENCONDE_SPEED,	//编码器速度控制
	GIMBAL_MOTOR_RAW,	  		    //电机原始值控制
} gimbal_motor_mode_e;


typedef enum
{
	GIMBAL_DEBUG=0,		    //debug模式，用于pid调参
	GIMBAL_ZERO_FORCE,	 	//云台无力
	GIMBAL_INIT,        	//云台初始化
	AUTO, 					      //自瞄模式
	GIMBAL_Dropshot,      //吊射
	GIMBAL_NORMAL, 			  //普通跟随模式
	GIMBAL_MODE_LENGTH,	  //长度
} gimbal_behaviour_e;


typedef struct
{ 
	gimbal_motor_mode_e motorPitCtrlType;
	gimbal_motor_mode_e motorYawCtrlType;
	void (*behaviorHandleFun)(float *yawExp, float *pitExp); //此模式的主要处理函数，最终要修改pid计算的期望值
	bool_t (*enterBehaviorCondition)(void); //进入该模式的条件
	bool_t (*outBehaviorCondition)(void);	//退出该模式的条件
	void (*enterBehaviorFun)(void); //进入该模式的处理
	void (*outBehaviorFun)(void);	//退出该模式的处理
} gimbal_behaviour_t;

typedef struct
{ 
	motor_measure_t base_inf;
	gimbal_motor_mode_e mode;
	gimbal_motor_mode_e lastMode;
	PidTypeDef PIDParameterList[2][2];
	uint16_t offsetEcd;	 	//校准的时候设置好的初始状态下的编码器的值，对应云台yaw中值，pitch水平值
  fp32 maxRelativeAngle;  //度，用于限位
  fp32 minRelativeAngle;  //度，用于限位 
	fp32 angle[2];     		//度 相对于中值（offest—_ecd）编码器反馈值的误差
    fp32 angle_set[2];		//度 
    fp32 speed[2];         	//度/秒
    fp32 speed_set[2];
    fp32 rawCmdCurrent; 		//raw模式下要发送的电流值
    fp32 currentSet;			//要发送的电流值
} gimbal_motor_t;

typedef struct{
	struct{
		const RC_ctrl_t *now; //遥控器当前值
		RC_ctrl_t last; //遥控器上一次的值
	} rc;
	const IMUTypedef *imu; //陀螺仪数据
	gimbal_behaviour_t behaviorList[GIMBAL_MODE_LENGTH]; //云台行为列表
	gimbal_behaviour_t *behaviorNow; //云台当前行为
	gimbal_behaviour_e nowBehaviorName; //当前模式的名字
	u8 behaviourStep;
	robot_mode_e *robotMode;
	robot_init_step *robotModeStep; 
	gimbal_motor_t pitchMotor; //pitch电机
	gimbal_motor_t left_pitchMotor;
	gimbal_motor_t yawMotor; //yaw电机
	struct {
		float yaw_exp[2];
		float pitch_exp[2];
	} visionDate;
	struct{
		struct{
			float err;										 //视觉传输过来误差
			float raw_exp;								 //未经平滑yaw期望
			float target_speed;            //视觉识别到目标的速度
			float position_real;					 //自瞄模式下yaw位置实际值
			float speed_real;							 //自瞄模式下yaw速度实际值			
			float *kf_result;						 //二阶卡尔曼滤波结果,0角度 1速度			
			kalman_filter_t kalman_filter;
			speed_calc_data_t speed_struct;//对目标速度解算结构体
		}yaw;
		struct{
			float err;										 //视觉传输过来误差
			float raw_exp;								 //未经平滑yaw期望
			float target_speed;            //视觉识别到目标的速度
			float position_real;					 //自瞄模式下yaw位置实际值
			float speed_real;							 //自瞄模式下yaw速度实际值			
			float *kf_result;						 //二阶卡尔曼滤波结果,0角度 1速度			
			kalman_filter_t kalman_filter;
			speed_calc_data_t speed_struct;//对目标速度解算结构体
		}pitch;
		struct{
			u8 if_rotate;//标记目标是否为陀螺
		}flag;
	}Auto_Mode;
} GimbalCtrl_t;

extern GimbalCtrl_t gimbalTaskStructure;
extern motor_speed_calc_t gimbalPitMotorSpeedCalc;
extern motor_speed_calc_t gimbalLeftPitMotorSpeedCalc;
extern motor_speed_calc_t gimbalYawMotorSpeedCalc;

void gimbalTask(void *pvParameters);
void gimbalBehaviourChange(gimbal_behaviour_t* next);
void GIMBAL_CanbusCtrlMotors(s16 pitchCurrent, s16 yawCurrent, s16 plCurrent,s16 flCurrent,s16 frCurrent);

#endif