#ifndef CHASSIS_TASK
#define CHASSIS_TASK

#include "robot.h"
#include "remote_control.h"
#include "pid.h"
#include "motor.h"
#include "kalman.h"
#include "user_lib.h"

//底盘发送报文标识符
#define CHASSIS_CANBUS_SEND_HEADER 0x200


//键盘按键宏定义
#define CHASSIS_FIRST_PRESS_W		!IF_LAST_KEY_PRESSED_W(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_W
#define CHASSIS_FIRST_PRESS_S		!IF_LAST_KEY_PRESSED_S(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_S
#define CHASSIS_FIRST_PRESS_A		!IF_LAST_KEY_PRESSED_A(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_A
#define CHASSIS_FIRST_PRESS_D		!IF_LAST_KEY_PRESSED_D(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_D
#define CHASSIS_FIRST_PRESS_Q		!IF_LAST_KEY_PRESSED_Q(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_Q
#define CHASSIS_FIRST_PRESS_E		!IF_LAST_KEY_PRESSED_E(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_E
#define CHASSIS_FIRST_PRESS_G		!IF_LAST_KEY_PRESSED_G(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_G
#define CHASSIS_FIRST_PRESS_X		!IF_LAST_KEY_PRESSED_X(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_X
#define CHASSIS_FIRST_PRESS_Z		!IF_LAST_KEY_PRESSED_Z(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_Z
#define CHASSIS_FIRST_PRESS_C		!IF_LAST_KEY_PRESSED_C(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_C
#define CHASSIS_FIRST_PRESS_B		!IF_LAST_KEY_PRESSED_B(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_B
#define CHASSIS_FIRST_PRESS_V		!IF_LAST_KEY_PRESSED_V(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_V
#define CHASSIS_FIRST_PRESS_F		!IF_LAST_KEY_PRESSED_F(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_F
#define CHASSIS_FIRST_PRESS_R		!IF_LAST_KEY_PRESSED_R(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_R
#define CHASSIS_FIRST_PRESS_CTRL	!IF_LAST_KEY_PRESSED_CTRL(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_CTRL
#define CHASSIS_FIRST_PRESS_SHIFT	!IF_LAST_KEY_PRESSED_SHIFT(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_SHIFT


//遥控器开关宏定义
#define CHASSIS_FIRST_S1_UP			IF_RC_SW1_UP && (!(chassisTaskStructure.rc.last.rc.s[0] == 1))
#define CHASSIS_FIRST_S1_DOWN		IF_RC_SW1_DOWN && (!(chassisTaskStructure.rc.last.rc.s[0] == 2))
#define CHASSIS_FIRST_S1_MID		IF_RC_SW1_MID && (!(chassisTaskStructure.rc.last.rc.s[0] == 3))
#define CHASSIS_FIRST_S2_UP			IF_RC_SW2_UP && (!(chassisTaskStructure.rc.last.rc.s[1] == 1))
#define CHASSIS_FIRST_S2_DOWN		IF_RC_SW2_DOWN && (!(chassisTaskStructure.rc.last.rc.s[1] == 2))
#define CHASSIS_FIRST_S2_MID		IF_RC_SW2_MID && (!(chassisTaskStructure.rc.last.rc.s[1] == 3))

//鼠标和遥控器遥杆宏定义
#define CHASSIS_FIRST_MOUSE_X_STOP	MOUSE_X_MOVE_SPEED == 0 && chassisTaskStructure.rc.last.mouse.x != 0
#define CHASSIS_FIRST_MOUSE_Y_STOP	MOUSE_Y_MOVE_SPEED == 0 && chassisTaskStructure.rc.last.mouse.y != 0
#define CHASSIS_FIRST_CH0_MID		RC_CH0_RLR_OFFSET == 0 && chassisTaskStructure.rc.last.rc.ch[0] != 0
#define CHASSIS_FIRST_CH1_MID		RC_CH1_RUD_OFFSET == 0 && chassisTaskStructure.rc.last.rc.ch[1] != 0
#define CHASSIS_FIRST_CH2_MID		RC_CH2_LLR_OFFSET == 0 && chassisTaskStructure.rc.last.rc.ch[2] != 0
#define CHASSIS_FIRST_CH3_MID		RC_CH3_LUD_OFFSET == 0 && chassisTaskStructure.rc.last.rc.ch[3] != 0

//3508电机减速比
#define CHASSIS_MOTOR_REDUCTION_RATIO 19


#ifndef CLOSE
#define CLOSE 0
#endif

#ifndef OPEN
#define OPEN 1
#endif

//底盘四个电机宏定义
#define CHASSIS_CM1	0
#define CHASSIS_CM2	1
#define CHASSIS_CM3	2
#define CHASSIS_CM4	3

//底盘功率和速度限制宏定义
#define CHASSIS_WARN_BUFF         60
#define CHASSIS_CAP_MAX_OUTPUT    40000	 //电容开启的情况下，最大输出
#define CHASSIS_CAP_MAX_VX        500    //电容开启下平移的最大速度     
#define CHASSIS_CAP_MAX_VY        500
#define CHASSIS_NORMAL_MAX_OUTPUT 25000  //普通模式下，电机的最大输出 
#define CHASSIS_NORMAL_MAX_VX     400    //360 //普通模式下，最大的平移速度 473
#define CHASSIS_NORMAL_MAX_VY     400    //360 473
#define CHASSIS_NORMAL_MAX_VZ     400
#define CHASSIS_FOLLOW_MAX_VZ			250		// Lemonade@230413
#define WARN_SPEED_VZ             75        
#define CAP_CONFIG                1
#define CAP_UNCONFIG              0



typedef enum
{
	VX = 0,
	VY,
	VZ
} chassis_speed_e;

//电机控制方式列表
typedef enum
{
	CHASSIS_ANGLE = 0,	//采用云台夹角方式控制旋转
	CHASSIS_SPEED,		  //采用vx vy vz的方式控制底盘
	CHASSIS_RAW,		    //直接给四个电机发电流
} chassis_mode_e;

//电机相关参数宏定义
typedef struct
{
	PidTypeDef pidParameter;  //PID参数结构体      
	fp32 rawCmdCurrent;       
	fp32 currentSet;           
	motor_measure_t baseInf;  //电机返回值结构体
	fp32 filterSpeed;         //卡尔曼滤波后实际速度值
	extKalman_t klmFiller;    //卡尔曼结构体
	float speedSet;           //速度期望值  
} chassis_motor_t;

//底盘模式列表
typedef enum
{
	CHASSIS_DEBUG = 0,
	CHASSIS_ZERO_FORCE,   //无力
	CHASSIS_INIT, 	      //初始化模式,
	CHASSIS_ROTATE,       //陀螺模式,平移直接受遥控器控制，根据条件进行匀速或者变速陀螺
	CHASSIS_ALONE,	      //独立模式,平移旋转都直接受遥控器控制
	CHASSIS_FOLLOW,	      //跟随模式
	CHASSIS_BEHAVOUR_LENGTH,
} chassis_behaviour_e;


//每一个模式所需条件结构体
typedef struct
{
	chassis_behaviour_e num;                //底盘模式
	chassis_mode_e      mode;               //底盘控制模式
	bool_t (*enterBehaviorCondition)(void); //进入模式条件
	bool_t (*outBehaviorCondition)(void);   //退出模式条件
	void (*enterBehaviorFun)(void);         //进入模式函数
	void (*outBehaviorFun)(void);           //退出模式函数
	void (*behaviorHandleFun)(float* vx, float* vy, float* vz, float* raw);
	                                        //模式处理函数
} chassis_behaviour_t;


typedef struct
{
	fp32 relativeSpeedSet[3];        //相对速度期望值(VX,VY,VZ)
	fp32 chassisSpeedSet[3];         //底盘速度期望值(VX,VY,VZ)
	fp32 relativeSpeedMax[3];        //相对速度最大值(VX,VY,VZ)
	//遥控器数据结构体
	struct{
		const RC_ctrl_t *now;          //遥控器当前值
		RC_ctrl_t last;                //遥控器上一次的值
	} rc;
	//功率限制结构体
	struct {
		uint16_t *buffer;	             //当前缓冲能量
		fp32 *power;		               //当前功率
		uint16_t warnBuffer;	         //减速缓冲能量阈值
		fp32 maxOutput;		             //最大输出总合
		fp32 kMaxOutput;	             //最大输出比例系数
		u8 capState;
	} powerLimit;
	chassis_mode_e mode;             //本次底盘控制模式
	chassis_mode_e lastMode;         //上次底盘控制模式
	chassis_motor_t motor[4];        //四个电机参数结构体
	fp32* relativeAngle;             //真实的角度值
	u8 if_want_to_open_cap;          //是否开启电容
	fp32 relativeAngleSet;           //角度的期望值        
	fp32 maxRelativeAngle;           //最大的角度值
	fp32 minRelativeAngle;           //最小的角度值 
	PidTypeDef anglePidParameter;    //角度环的PID结构体
	chassis_behaviour_t  behaviorList[CHASSIS_BEHAVOUR_LENGTH];
	                                 //底盘行为列表
	chassis_behaviour_t* behaviorNow;//底盘当前行为
	u8 behaviourStep;                //底盘初始化步骤
}ChassisCtrl_t;



void chassisTaskInit();
void chassisTask(void *pvParameters);

extern ChassisCtrl_t chassisTaskStructure;

#endif

