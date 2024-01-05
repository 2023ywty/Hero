#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "robot.h"

//��������
void zeroForceHandleFun(float *raw_cm1, float *raw_cm2, float *raw_cm3, float *raw_cm4);
bool_t enterZeroForceCondition();
bool_t outZeroForceCondition();

//���̳�ʼ��
void chassisInitHandleFun(float *raw_cm1, float *raw_cm2, float *raw_cm3, float *raw_cm4);
bool_t outChassisInitCondition();
bool_t enterChassisInitCondition();

//���̲���
void chassisSleepHandleFun(float* vx, float* vy, float* vz, float *none);
bool_t outChassisSleepCondition();
bool_t enterChassisSleepCondition();

//Ť��ģʽ
bool_t  enterTwistCondition();
bool_t  outTwistCondition();
void chassisTwistHandleFun(float* vx, float *vy, float* vz, float* none);

//����ģʽ
#define CHASSIS_ROTATE_VZ 200
bool_t enterChassisRotateCondition();
bool_t outChassisRotateCondition();
void chassisRotateHandleFun(float* vx, float *vy, float* vz, float* none);

//����������ģʽ
bool_t enterChassisAloneCondition();
bool_t outChassisAloneCondition();
void chassisAloneHandleFun(float* vx, float *vy, float* vz, float* none);

void chassisFollowHandleFun(float *vx, float *vy, float *vz, float *none);
bool_t enterChassisFollowCondition();
bool_t outChassisFollowCondition();

//debug
bool_t enterChassisDebugCondition();
bool_t outChassisDebugCondition();
void chassisDebugHandleFun(float *vx, float *vy, float *vz, float *none);
#endif