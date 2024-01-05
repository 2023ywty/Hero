#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H
#include "robot.h"
//每个behavior对应函数声明
bool_t gimbalDebugBehaviourEnterCondition();
bool_t gimbalDebugBehaviourOutCondition();
void gimbalDebugBehaviourHandleFun(float *yawExp, float *pitExp);

bool_t gimbalMidBehaviourEnterCondition();
bool_t gimbalMidBehaviourOutCondition();
void gimbalMidBehaviourHandleFun(float *yawExp, float *pitExp);

void gimbalZeroForceBehaviourHandleFun(float *yawExp, float *pitExp);
bool_t gimbalZeroForceBehaviourEnterCondition();
bool_t gimbalZeroForceBehaviourOutCondition();

void gimbalInitBehaviourHandleFun(float *yawExp, float *pitExp);
bool_t gimbalInitBehaviourEnterCondition();
bool_t gimbalInitBehaviourOutCondition();


void gimbalNormalBehaviourHandleFun(float *yawExp, float *pitExp);
bool_t gimbalNormalBehaviourEnterCondition();
bool_t gimbalNormalBehaviourOutCondition();

void gimbalTurnBehaviourHandleFun(float *yawExp, float *pitExp);
bool_t gimbalTurnBehaviourEnterCondition();
bool_t gimbalTurnBehaviourOutCondition();

void gimbalAutoBehaviourHandleFun(float *yawExp, float *pitExp);
bool_t gimbalAutoBehaviourEnterCondition();
bool_t gimbalAutoBehaviourOutCondition();


void gimbalSmallBuffBehaviourHandleFun(float *yawExp, float *pitExp);
bool_t gimbalSmallBuffBehaviourEnterCondition();
bool_t gimbalSmallBuffBehaviourOutCondition();


void gimbalBigBuffBehaviourHandleFun(float *yawExp, float *pitExp);
bool_t gimbalBigBuffBehaviourEnterCondition();
bool_t gimbalBigBuffBehaviourOutCondition();


void gimbalDropshotBehaviourHandleFun(float *yawExp, float *pitExp);
bool_t gimbalDropshotBehaviourEnterCondition(void);
bool_t gimbalADropshotBehaviourOutCondition(void);


#endif