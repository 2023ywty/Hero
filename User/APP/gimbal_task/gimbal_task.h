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

//#define GIMBAL_PITCH_MOTOR_OFFSET_ECD  2705  //4100.3463  //�������
#define GIMBAL_PITCH_MOTOR_OFFSET_ECD  2628 //4100.3463 //����ˮƽ
#define GIMBAL_YAW_MOTOR_OFFSET_ECD    2600

#define INNER 	0	
#define OUTER 	1
#define GYRO 		0
#define ENCONDE 1


#define GIMBAL_CANBUS_SEND_HEADER 0x1FF
//�������ֵת���ɽǶ�ֵ
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

//�˴�Ϊ������������
#define ABS_YAW_MAX_RANGE 180
#define ABS_YAW_MIN_RANGE -180
#define ABS_PITCH_MAX_RANGE 180
#define ABS_PITCH_MIN_RANGE -180
#define RELATIVE_PITCH_MAX_RANGE 180
#define RELATIVE_PITCH_MIN_RANGE -180
#define RELATIVE_YAW_MAX_RANGE 180
#define RELATIVE_YAW_MIN_RANGE -180

//�������ֵ����Լ���ֵ
#define HALF_ECD_RANGE 4096
#define MAX_ECD_RANGE 8191

//�½��ش����궨�� ����ң������صĺ궨����Remote_Control.h
#define GIMBAL_FIRST_PRESS_W		!IF_LAST_KEY_PRESSED_W(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_W
#define GIMBAL_FIRST_PRESS_S		!IF_LAST_KEY_PRESSED_S(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_S
#define GIMBAL_FIRST_PRESS_A		!IF_LAST_KEY_PRESSED_A(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_A
#define GIMBAL_FIRST_PRESS_D		!IF_LAST_KEY_PRESSED_D(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_D
#define GIMBAL_FIRST_PRESS_Q		!IF_LAST_KEY_PRESSED_Q(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_Q     //����
#define GIMBAL_FIRST_PRESS_E		!IF_LAST_KEY_PRESSED_E(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_E
#define GIMBAL_FIRST_PRESS_G		!IF_LAST_KEY_PRESSED_G(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_G     //����
#define GIMBAL_FIRST_PRESS_X		!IF_LAST_KEY_PRESSED_X(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_X     //����
#define GIMBAL_FIRST_PRESS_Z		!IF_LAST_KEY_PRESSED_Z(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_Z     //����
#define GIMBAL_FIRST_PRESS_C		!IF_LAST_KEY_PRESSED_C(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_C     //����
#define GIMBAL_FIRST_PRESS_B		!IF_LAST_KEY_PRESSED_B(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_B     //����
#define GIMBAL_FIRST_PRESS_V		!IF_LAST_KEY_PRESSED_V(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_V     //����
#define GIMBAL_FIRST_PRESS_F		!IF_LAST_KEY_PRESSED_F(gimbalTaskStructure.rc.last) && IF_KEY_PRESSED_F     //����
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



/*���׿�����*/
#define KF_ANGLE	0
#define KF_SPEED	1
#define KF_ACCEL	2


typedef enum
{
  GIMBAL_MOTOR_GYRO = 0,   	  //��������ǽǶȿ���
  GIMBAL_MOTOR_ENCONDE, 		  //�������ֵ�Ƕȿ���
	GIMBAL_MOTOR_GYRO_SPEED,	  //�������ٶȿ���
	GIMBAL_MOTOR_ENCONDE_SPEED,	//�������ٶȿ���
	GIMBAL_MOTOR_RAW,	  		    //���ԭʼֵ����
} gimbal_motor_mode_e;


typedef enum
{
	GIMBAL_DEBUG=0,		    //debugģʽ������pid����
	GIMBAL_ZERO_FORCE,	 	//��̨����
	GIMBAL_INIT,        	//��̨��ʼ��
	AUTO, 					      //����ģʽ
	GIMBAL_Dropshot,      //����
	GIMBAL_NORMAL, 			  //��ͨ����ģʽ
	GIMBAL_MODE_LENGTH,	  //����
} gimbal_behaviour_e;


typedef struct
{ 
	gimbal_motor_mode_e motorPitCtrlType;
	gimbal_motor_mode_e motorYawCtrlType;
	void (*behaviorHandleFun)(float *yawExp, float *pitExp); //��ģʽ����Ҫ������������Ҫ�޸�pid���������ֵ
	bool_t (*enterBehaviorCondition)(void); //�����ģʽ������
	bool_t (*outBehaviorCondition)(void);	//�˳���ģʽ������
	void (*enterBehaviorFun)(void); //�����ģʽ�Ĵ���
	void (*outBehaviorFun)(void);	//�˳���ģʽ�Ĵ���
} gimbal_behaviour_t;

typedef struct
{ 
	motor_measure_t base_inf;
	gimbal_motor_mode_e mode;
	gimbal_motor_mode_e lastMode;
	PidTypeDef PIDParameterList[2][2];
	uint16_t offsetEcd;	 	//У׼��ʱ�����úõĳ�ʼ״̬�µı�������ֵ����Ӧ��̨yaw��ֵ��pitchˮƽֵ
  fp32 maxRelativeAngle;  //�ȣ�������λ
  fp32 minRelativeAngle;  //�ȣ�������λ 
	fp32 angle[2];     		//�� �������ֵ��offest��_ecd������������ֵ�����
    fp32 angle_set[2];		//�� 
    fp32 speed[2];         	//��/��
    fp32 speed_set[2];
    fp32 rawCmdCurrent; 		//rawģʽ��Ҫ���͵ĵ���ֵ
    fp32 currentSet;			//Ҫ���͵ĵ���ֵ
} gimbal_motor_t;

typedef struct{
	struct{
		const RC_ctrl_t *now; //ң������ǰֵ
		RC_ctrl_t last; //ң������һ�ε�ֵ
	} rc;
	const IMUTypedef *imu; //����������
	gimbal_behaviour_t behaviorList[GIMBAL_MODE_LENGTH]; //��̨��Ϊ�б�
	gimbal_behaviour_t *behaviorNow; //��̨��ǰ��Ϊ
	gimbal_behaviour_e nowBehaviorName; //��ǰģʽ������
	u8 behaviourStep;
	robot_mode_e *robotMode;
	robot_init_step *robotModeStep; 
	gimbal_motor_t pitchMotor; //pitch���
	gimbal_motor_t left_pitchMotor;
	gimbal_motor_t yawMotor; //yaw���
	struct {
		float yaw_exp[2];
		float pitch_exp[2];
	} visionDate;
	struct{
		struct{
			float err;										 //�Ӿ�����������
			float raw_exp;								 //δ��ƽ��yaw����
			float target_speed;            //�Ӿ�ʶ��Ŀ����ٶ�
			float position_real;					 //����ģʽ��yawλ��ʵ��ֵ
			float speed_real;							 //����ģʽ��yaw�ٶ�ʵ��ֵ			
			float *kf_result;						 //���׿������˲����,0�Ƕ� 1�ٶ�			
			kalman_filter_t kalman_filter;
			speed_calc_data_t speed_struct;//��Ŀ���ٶȽ���ṹ��
		}yaw;
		struct{
			float err;										 //�Ӿ�����������
			float raw_exp;								 //δ��ƽ��yaw����
			float target_speed;            //�Ӿ�ʶ��Ŀ����ٶ�
			float position_real;					 //����ģʽ��yawλ��ʵ��ֵ
			float speed_real;							 //����ģʽ��yaw�ٶ�ʵ��ֵ			
			float *kf_result;						 //���׿������˲����,0�Ƕ� 1�ٶ�			
			kalman_filter_t kalman_filter;
			speed_calc_data_t speed_struct;//��Ŀ���ٶȽ���ṹ��
		}pitch;
		struct{
			u8 if_rotate;//���Ŀ���Ƿ�Ϊ����
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