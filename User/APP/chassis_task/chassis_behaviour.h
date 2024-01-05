#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "robot.h"

//底盘无力
void zeroForceHandleFun(float *raw_cm1, float *raw_cm2, float *raw_cm3, float *raw_cm4);
bool_t enterZeroForceCondition();
bool_t outZeroForceCondition();

//底盘初始化
void chassisInitHandleFun(float *raw_cm1, float *raw_cm2, float *raw_cm3, float *raw_cm4);
bool_t outChassisInitCondition();
bool_t enterChassisInitCondition();

//底盘不动
void chassisSleepHandleFun(float* vx, float* vy, float* vz, float *none);
bool_t outChassisSleepCondition();
bool_t enterChassisSleepCondition();

//扭腰模式
bool_t  enterTwistCondition();
bool_t  outTwistCondition();
void chassisTwistHandleFun(float* vx, float *vy, float* vz, float* none);

//陀螺模式
#define CHASSIS_ROTATE_VZ 200
bool_t enterChassisRotateCondition();
bool_t outChassisRotateCondition();
void chassisRotateHandleFun(float* vx, float *vy, float* vz, float* none);

//独立不跟随模式
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