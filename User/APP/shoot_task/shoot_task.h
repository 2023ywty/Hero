#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H
#include "motor.h"
#include "PID.h"
#include "Remote_Control.h"
#include "kalman.h"

#ifndef INNER
#define INNER 0
#endif

#ifndef OUTER
#define OUTER 1
#endif

#define SHOOT_TASK_MS 2
#define SHOOT_FIRST_PRESS_W		!IF_LAST_KEY_PRESSED_W(shootTaskStructure.rc.last) && IF_KEY_PRESSED_W
#define SHOOT_FIRST_PRESS_S		!IF_LAST_KEY_PRESSED_S(shootTaskStructure.rc.last) && IF_KEY_PRESSED_S
#define SHOOT_FIRST_PRESS_A		!IF_LAST_KEY_PRESSED_A(shootTaskStructure.rc.last) && IF_KEY_PRESSED_A
#define SHOOT_FIRST_PRESS_D		!IF_LAST_KEY_PRESSED_D(shootTaskStructure.rc.last) && IF_KEY_PRESSED_D
#define SHOOT_FIRST_PRESS_Q		!IF_LAST_KEY_PRESSED_Q(shootTaskStructure.rc.last) && IF_KEY_PRESSED_Q
#define SHOOT_FIRST_PRESS_E		!IF_LAST_KEY_PRESSED_E(shootTaskStructure.rc.last) && IF_KEY_PRESSED_E
#define SHOOT_FIRST_PRESS_G		!IF_LAST_KEY_PRESSED_G(shootTaskStructure.rc.last) && IF_KEY_PRESSED_G
#define SHOOT_FIRST_PRESS_X		!IF_LAST_KEY_PRESSED_X(shootTaskStructure.rc.last) && IF_KEY_PRESSED_X
#define SHOOT_FIRST_PRESS_Z		!IF_LAST_KEY_PRESSED_Z(shootTaskStructure.rc.last) && IF_KEY_PRESSED_Z
#define SHOOT_FIRST_PRESS_C		!IF_LAST_KEY_PRESSED_C(shootTaskStructure.rc.last) && IF_KEY_PRESSED_C
#define SHOOT_FIRST_PRESS_B		!IF_LAST_KEY_PRESSED_B(shootTaskStructure.rc.last) && IF_KEY_PRESSED_B
#define SHOOT_FIRST_PRESS_V		!IF_LAST_KEY_PRESSED_V(shootTaskStructure.rc.last) && IF_KEY_PRESSED_V
#define SHOOT_FIRST_PRESS_F		!IF_LAST_KEY_PRESSED_F(shootTaskStructure.rc.last) && IF_KEY_PRESSED_F
#define SHOOT_FIRST_PRESS_R		!IF_LAST_KEY_PRESSED_R(shootTaskStructure.rc.last) && IF_KEY_PRESSED_R
#define SHOOT_FIRST_PRESS_CTRL	!IF_LAST_KEY_PRESSED_CTRL(shootTaskStructure.rc.last) && IF_KEY_PRESSED_CTRL
#define SHOOT_FIRST_PRESS_SHIFT	!IF_LAST_KEY_PRESSED_SHIFT(shootTaskStructure.rc.last) && IF_KEY_PRESSED_SHIFT

#define SHOOT_FIRST_S1_UP			(IF_RC_SW1_UP && (shootTaskStructure.rc.last.rc.s[0] != 1))
#define SHOOT_FIRST_S1_DOWN			(IF_RC_SW1_DOWN && (shootTaskStructure.rc.last.rc.s[0] != 2))
#define SHOOT_FIRST_S1_MID			(IF_RC_SW1_MID && (shootTaskStructure.rc.last.rc.s[0] != 3))

#define SHOOT_FIRST_S2_UP			(IF_RC_SW2_UP && (shootTaskStructure.rc.last.rc.s[1] != 1))
#define SHOOT_FIRST_S2_DOWN			(IF_RC_SW2_DOWN && (shootTaskStructure.rc.last.rc.s[1] != 2))
#define SHOOT_FIRST_S2_MID			(IF_RC_SW2_MID && (shootTaskStructure.rc.last.rc.s[1] != 3))

#define SHOOT_FIRST_MOUSE_X_STOP	MOUSE_X_MOVE_SPEED == 0 && shootTaskStructure.rc.last.mouse.x != 0
#define SHOOT_FIRST_MOUSE_Y_STOP	MOUSE_Y_MOVE_SPEED == 0 && shootTaskStructure.rc.last.mouse.y != 0
#define SHOOT_FIRST_CH0_MID		RC_CH0_RLR_OFFSET == 0 && shootTaskStructure.rc.last.rc.ch[0] != 0
#define SHOOT_FIRST_CH1_MID		RC_CH1_RUD_OFFSET == 0 && shootTaskStructure.rc.last.rc.ch[1] != 0
#define SHOOT_FIRST_CH2_MID		RC_CH2_LLR_OFFSET == 0 && shootTaskStructure.rc.last.rc.ch[2] != 0
#define SHOOT_FIRST_CH3_MID		RC_CH3_LUD_OFFSET == 0 && shootTaskStructure.rc.last.rc.ch[3] != 0

#define SHOOT_CANBUS_SEND_HEADER 0x200

#define REVOL_CAN_OPEN    350  //摩擦轮实际速度超过这个值才允许拨盘转动,根据摩擦轮最小目标速度来改变

#define READING_SENSOR()	GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6)			//读取微动开关电平

#define HEAD_CLOSE()	TIM_SetCompare4(TIM8, 125);		//不使用倍镜
#define HEAD_High_OPEN()		TIM_SetCompare4(TIM8, 177);		//使用高倍镜
#define HEAD_Low_OPEN()		TIM_SetCompare4(TIM8, 83);		//使用低倍镜

#define SHOOT_SPEED_HIGH   19//对应rpm7002 线速度数值为22   
#define SHOOT_SPEED_MID 	 16//对应rpm4456 线速度数值为14
#define SHOOT_SPEED_LOW    14//对应rpm7002 线速度数值为22 


#define SHOOT_SPEED_QUIET  0//摩擦轮不转   
#define MAX_HEAT_LEVEL1	100		//1级热量上限
#define MAX_HEAT_LEVEL2	200		//2级热量上限
#define MAX_HEAT_LEVEL3	300		//3级热量上限	


typedef enum
{
    SHOOT_MOTOR_POSITION = 0,  //位置环控制电机
    SHOOT_MOTOR_SPEED, 		//速度环控制电机
    SHOOT_MOTOR_RAW,		//直接发送电流控制电机
} shoot_motor_mode_e;

typedef enum
{
    SHOOT_FIRCTION_MOTOR_LEFT = 0,
    SHOOT_FIRCTION_MOTOR_RIGHT,
    SHOOT_FIRCTION_MOTOR_PLATE,
} shoot_motor_type_e;

typedef struct
{
    motor_measure_t base_inf;
    PidTypeDef PIDParameter[2];	//pid参数
    fp32 speed_set;		//rpm
    fp32 position_set;	//0-8191
    fp32 currentSet;	//要发送的电流值
    fp32 rawCmdCurrent;	//raw模式下发送的电流
    u8 teethNum;		//播齿的数量
	  u8 shootcontinue;  //连发
    u8 ifSensorOnLine;	//是否含有对射开关
    shoot_motor_mode_e mode;	//拨盘模式
} plate_motor_t;


typedef struct
{
    motor_measure_t base_inf;
    float speed;				  //摩擦轮线速度m/s
    float speedSet;				//摩擦轮线速度期望
    fp32 rawCmdCurrent; 		//raw模式下要发送的电流值
    fp32 currentSet;			//要发送的电流值
    shoot_motor_mode_e mode;	//摩擦轮模式
    fp32 radius;				//摩擦轮半径 mm
    PidTypeDef PIDParameter;
		fp32 filterSpeed;         //卡尔曼滤波后实际速度值
		extKalman_t klmFiller;    //卡尔曼结构体
} firction_motor_t;


typedef enum
{
	BULLET_ON=0,		//炮管有子弹
	BULLET_OFF,			//炮管无子弹
}sensor_bullet_status;


typedef enum
{
    SHOOT_DEBUG = 0,		//debug模式，用于pid调参
    SHOOT_ZERO_FORCE,	 	//发射无力
    SHOOT_SENSOR,			  //有微动开关模式
    SHOOT_POSITION,			//无微动开关拨盘位置双环控制,主要控制函数传入拨盘期望值为编码器原值0-8191
    SHOOT_FIR_OFF,			//摩擦轮电机关闭模式
    SHOOT_MODE_LENGTH,	    //长度
} shoot_behaviour_e;


typedef struct
{
    shoot_motor_mode_e firLeftMode;
    shoot_motor_mode_e firRightMode;
    shoot_motor_mode_e plateMode;
    shoot_behaviour_e num;
    void (*behaviorHandleFun)(float *leftFirExp, float *rightFirExp, float *plateExp); //此模式的主要处理函数，最终要修改pid计算的期望值
    bool_t (*enterBehaviorCondition)(void); //进入该模式的条件
    bool_t (*outBehaviorCondition)(void);	//退出该模式的条件
    void (*enterBehaviorFun)(void); //进入该模式的处理
    void (*outBehaviorFun)(void);	//退出该模式的处理
} shoot_behaviour_t;


typedef struct
{
	firction_motor_t leftFirMotor;
	firction_motor_t rightFirMotor;
	plate_motor_t    plateMotor;

	struct 
	{
			const RC_ctrl_t *now; //遥控器当前值
			RC_ctrl_t last; //遥控器上一次的值
	} rc;

	shoot_behaviour_t behList[SHOOT_MODE_LENGTH];
	shoot_behaviour_t *nowBeh;
	u8 behaviourStep;	//行为的步骤

	float shootHzSet;	//射频期望
	float shootHz;    
	s16 shootNumSet;	//发射子弹数量期望
	u8 ifSensorExit;
	u8 shootspeed_set;
	u8 shootfirturn_flag;
	struct
	{
			s16 maxHeat;
			s16 heatNow;
			float lastSpeed;
	} heatLimit;
	struct 
	{
		sensor_bullet_status lastSensorStatus;//上回炮管有无子弹状态
		sensor_bullet_status nowSensorStatus;//当前炮管有无子弹状态
		u8 if_plate_stop_immedially;//是否拨盘立即停止
		u16 leftUnpressTime;//鼠标左键未按下时间
		u16 shootPlateflag;
		u8  Projectile_if_Prepare;
	} shoot_sensor;	
	
} ShootCtrl_t;


void shootInit();
void shootReset();
void shootTask(void *pvParameters);
void shootBehChange(shoot_behaviour_t *next);


extern ShootCtrl_t shootTaskStructure;


#endif