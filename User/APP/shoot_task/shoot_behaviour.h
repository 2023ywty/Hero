#ifndef SHOOT_BEHAVIOUR_H
#define SHOOT_BEHAVIOUR_H
#include "robot.h"


//debug
bool_t shootbehDebugEnterCondition();
bool_t shootbehDebugOutCondition();
void shootbehDebugHandleFun(float *leftFirExp, float *rightFirExp, float *plateExp);

//zeroFroce
bool_t shootbehZeroForceEnterCondition();
bool_t shootbehZeroForceOutCondition();
void shootbehZeroForceHandleFun(float *leftFirExp, float *rightFirExp, float *plateExp);

//sensor
bool_t shootbehSensorEnterCondition();
bool_t shootbehSensorOutCondition();
void shootbehSensorHandleFun(float *leftFirExp, float *rightFirExp, float *plateExp);


//RATATE
bool_t shootbehbeat_rotateEnterCondition();
bool_t shootbehbeat_rotateOutCondition();
void shootbehbeat_rotateHandleFun(float *leftFirExp, float *rightFirExp, float *plateExp);

//GUARD
bool_t shootbehbeat_guardEnterCondition();
bool_t shootbehbeat_guardOutCondition();
void   shootbehbeat_guardHandleFun(float *leftFirExp, float *rightFirExp, float *plateExp);
 
 
//position
bool_t shootbehPositionEnterCondition();
bool_t shootbehPositionOutCondition();
void shootbehPositionHandleFun(float *leftFirExp, float *rightFirExp, float *plateExp);

//speed
bool_t shootbehSpeedEnterCondition();
bool_t shootbehSpeedOutCondition();
void shootbehSpeedHandleFun(float *leftFirExp, float *rightFirExp, float *plateExp);

//firoff
bool_t shootbehFirOffEnterCondition();
bool_t shootbehFirOffOutCondition();
void shootbehFirOffHandleFun(float *leftFirExp, float *rightFirExp, float *plateExp);

#endif
