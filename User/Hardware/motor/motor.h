#ifndef MOTOR_H
#define MOTOR_H
#include "main.h"
#include "stm32f4xx.h"

#define MOTOR_ECD_FEEDBACL_RANGE 8191


typedef struct
{
    int8_t  position_cnt;
    uint16_t rotor_ecd;		 //ת�ӽǶ�
    int16_t last_ecd;		   //�ϴ�ת�ӵĽǶ�
    int16_t real_ecd;		   //�����ĽǶ�
    uint8_t reduction_ratio; //������ٱ�
    int16_t speed_rpm;		 //��ǰת��ת��
    float real_speed_rpm;	   //������ת��
    int16_t given_current;	 //���͵ĵ���ֵ
} motor_measure_t;

typedef struct
{
	fp32 *speed_rpm;
	uint32_t lastTime;
	uint16_t minCalcTime;
	int16_t lastEcd;
	int16_t ecdMaxVal;
	int16_t ecdMinVal;
	uint16_t ecdRange;
} motor_speed_calc_t;

void motorSpeedCalcInit(motor_speed_calc_t* calcS, motor_measure_t *motorS, int16_t ecdMinVal, int16_t ecdMaxVal, uint32_t time, uint16_t minCalcTime);
void reductMotorDataRecieve(CanRxMsg *canMsg, motor_measure_t *motor);
void gimbalMotorDataRecieve(CanRxMsg *canMsg, motor_measure_t *motor);
void chassisMotorDataRecieve(CanRxMsg *canMsg, motor_measure_t *motor);
float getMotorSpeedByPosition(motor_speed_calc_t* calcS, int16_t nowEcd, uint32_t nowTime);
void motorInit(motor_measure_t *motor, u16 radio);
#endif
