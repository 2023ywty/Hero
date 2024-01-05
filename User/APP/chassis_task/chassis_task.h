#ifndef CHASSIS_TASK
#define CHASSIS_TASK

#include "robot.h"
#include "remote_control.h"
#include "pid.h"
#include "motor.h"
#include "kalman.h"
#include "user_lib.h"

//���̷��ͱ��ı�ʶ��
#define CHASSIS_CANBUS_SEND_HEADER 0x200


//���̰����궨��
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


//ң�������غ궨��
#define CHASSIS_FIRST_S1_UP			IF_RC_SW1_UP && (!(chassisTaskStructure.rc.last.rc.s[0] == 1))
#define CHASSIS_FIRST_S1_DOWN		IF_RC_SW1_DOWN && (!(chassisTaskStructure.rc.last.rc.s[0] == 2))
#define CHASSIS_FIRST_S1_MID		IF_RC_SW1_MID && (!(chassisTaskStructure.rc.last.rc.s[0] == 3))
#define CHASSIS_FIRST_S2_UP			IF_RC_SW2_UP && (!(chassisTaskStructure.rc.last.rc.s[1] == 1))
#define CHASSIS_FIRST_S2_DOWN		IF_RC_SW2_DOWN && (!(chassisTaskStructure.rc.last.rc.s[1] == 2))
#define CHASSIS_FIRST_S2_MID		IF_RC_SW2_MID && (!(chassisTaskStructure.rc.last.rc.s[1] == 3))

//����ң����ң�˺궨��
#define CHASSIS_FIRST_MOUSE_X_STOP	MOUSE_X_MOVE_SPEED == 0 && chassisTaskStructure.rc.last.mouse.x != 0
#define CHASSIS_FIRST_MOUSE_Y_STOP	MOUSE_Y_MOVE_SPEED == 0 && chassisTaskStructure.rc.last.mouse.y != 0
#define CHASSIS_FIRST_CH0_MID		RC_CH0_RLR_OFFSET == 0 && chassisTaskStructure.rc.last.rc.ch[0] != 0
#define CHASSIS_FIRST_CH1_MID		RC_CH1_RUD_OFFSET == 0 && chassisTaskStructure.rc.last.rc.ch[1] != 0
#define CHASSIS_FIRST_CH2_MID		RC_CH2_LLR_OFFSET == 0 && chassisTaskStructure.rc.last.rc.ch[2] != 0
#define CHASSIS_FIRST_CH3_MID		RC_CH3_LUD_OFFSET == 0 && chassisTaskStructure.rc.last.rc.ch[3] != 0

//3508������ٱ�
#define CHASSIS_MOTOR_REDUCTION_RATIO 19


#ifndef CLOSE
#define CLOSE 0
#endif

#ifndef OPEN
#define OPEN 1
#endif

//�����ĸ�����궨��
#define CHASSIS_CM1	0
#define CHASSIS_CM2	1
#define CHASSIS_CM3	2
#define CHASSIS_CM4	3

//���̹��ʺ��ٶ����ƺ궨��
#define CHASSIS_WARN_BUFF         60
#define CHASSIS_CAP_MAX_OUTPUT    40000	 //���ݿ���������£�������
#define CHASSIS_CAP_MAX_VX        500    //���ݿ�����ƽ�Ƶ�����ٶ�     
#define CHASSIS_CAP_MAX_VY        500
#define CHASSIS_NORMAL_MAX_OUTPUT 25000  //��ͨģʽ�£������������ 
#define CHASSIS_NORMAL_MAX_VX     400    //360 //��ͨģʽ�£�����ƽ���ٶ� 473
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

//������Ʒ�ʽ�б�
typedef enum
{
	CHASSIS_ANGLE = 0,	//������̨�нǷ�ʽ������ת
	CHASSIS_SPEED,		  //����vx vy vz�ķ�ʽ���Ƶ���
	CHASSIS_RAW,		    //ֱ�Ӹ��ĸ����������
} chassis_mode_e;

//�����ز����궨��
typedef struct
{
	PidTypeDef pidParameter;  //PID�����ṹ��      
	fp32 rawCmdCurrent;       
	fp32 currentSet;           
	motor_measure_t baseInf;  //�������ֵ�ṹ��
	fp32 filterSpeed;         //�������˲���ʵ���ٶ�ֵ
	extKalman_t klmFiller;    //�������ṹ��
	float speedSet;           //�ٶ�����ֵ  
} chassis_motor_t;

//����ģʽ�б�
typedef enum
{
	CHASSIS_DEBUG = 0,
	CHASSIS_ZERO_FORCE,   //����
	CHASSIS_INIT, 	      //��ʼ��ģʽ,
	CHASSIS_ROTATE,       //����ģʽ,ƽ��ֱ����ң�������ƣ����������������ٻ��߱�������
	CHASSIS_ALONE,	      //����ģʽ,ƽ����ת��ֱ����ң��������
	CHASSIS_FOLLOW,	      //����ģʽ
	CHASSIS_BEHAVOUR_LENGTH,
} chassis_behaviour_e;


//ÿһ��ģʽ���������ṹ��
typedef struct
{
	chassis_behaviour_e num;                //����ģʽ
	chassis_mode_e      mode;               //���̿���ģʽ
	bool_t (*enterBehaviorCondition)(void); //����ģʽ����
	bool_t (*outBehaviorCondition)(void);   //�˳�ģʽ����
	void (*enterBehaviorFun)(void);         //����ģʽ����
	void (*outBehaviorFun)(void);           //�˳�ģʽ����
	void (*behaviorHandleFun)(float* vx, float* vy, float* vz, float* raw);
	                                        //ģʽ������
} chassis_behaviour_t;


typedef struct
{
	fp32 relativeSpeedSet[3];        //����ٶ�����ֵ(VX,VY,VZ)
	fp32 chassisSpeedSet[3];         //�����ٶ�����ֵ(VX,VY,VZ)
	fp32 relativeSpeedMax[3];        //����ٶ����ֵ(VX,VY,VZ)
	//ң�������ݽṹ��
	struct{
		const RC_ctrl_t *now;          //ң������ǰֵ
		RC_ctrl_t last;                //ң������һ�ε�ֵ
	} rc;
	//�������ƽṹ��
	struct {
		uint16_t *buffer;	             //��ǰ��������
		fp32 *power;		               //��ǰ����
		uint16_t warnBuffer;	         //���ٻ���������ֵ
		fp32 maxOutput;		             //�������ܺ�
		fp32 kMaxOutput;	             //����������ϵ��
		u8 capState;
	} powerLimit;
	chassis_mode_e mode;             //���ε��̿���ģʽ
	chassis_mode_e lastMode;         //�ϴε��̿���ģʽ
	chassis_motor_t motor[4];        //�ĸ���������ṹ��
	fp32* relativeAngle;             //��ʵ�ĽǶ�ֵ
	u8 if_want_to_open_cap;          //�Ƿ�������
	fp32 relativeAngleSet;           //�Ƕȵ�����ֵ        
	fp32 maxRelativeAngle;           //���ĽǶ�ֵ
	fp32 minRelativeAngle;           //��С�ĽǶ�ֵ 
	PidTypeDef anglePidParameter;    //�ǶȻ���PID�ṹ��
	chassis_behaviour_t  behaviorList[CHASSIS_BEHAVOUR_LENGTH];
	                                 //������Ϊ�б�
	chassis_behaviour_t* behaviorNow;//���̵�ǰ��Ϊ
	u8 behaviourStep;                //���̳�ʼ������
}ChassisCtrl_t;



void chassisTaskInit();
void chassisTask(void *pvParameters);

extern ChassisCtrl_t chassisTaskStructure;

#endif

