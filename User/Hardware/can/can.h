#ifndef CAN_H
#define CAN_H

#include "stm32f4xx.h"
void CAN1_Init();
void CAN2_Init();
void djiMotorCurrentSendQueue(CAN_TypeDef* CANx, uint32_t stdId, s16 *current, u8 len);
void CAN1_SendBoardB(float yawAngle, float yawSpeed, float pitchAngle, float pitchSpeed);
void CAN1_SendRequestMsg(u8 requestMsg);
void CAN1_SendBoardB_AboutCapacitior(u8 status);
#endif