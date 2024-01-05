/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       detect_task.c/h
  * @brief      
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef DETECT_Task_H
#define DETECT_Task_H
#include "main.h"
//�������Լ���Ӧ�豸˳��
enum errorList
{
  DBUSTOE = 0,
	YAW_MOTOR,
	PITCH_MOTOR,
	DETECT_CHASSIS_CM1_MOTOR,
	DETECT_CHASSIS_CM2_MOTOR,
	DETECT_CHASSIS_CM3_MOTOR,
	DETECT_CHASSIS_CM4_MOTOR,
	DETECT_VISION,
	DETECT_SHOOT_LF_MOTOR,
	DETECT_SHOOT_RF_MOTOR,
	DETECT_SHOOT_PLATE_MOTOR,
	DETECT_BOARDB_LINK,
	DETECT_JUDGE_SYSTEM,
  errorListLength,
};

//���ݶ�ʧ���ڼ�⣬���ݴ����߼����
typedef __packed struct
{ 
    uint32_t newTime;		//�ϴθ��µ�ʱ��
    uint32_t lastTime;
    uint32_t Losttime;
    uint32_t worktime;		//�ϵ�ʱ��
    uint16_t setOfflineTime : 12;
    uint16_t setOnlineTime : 12;
    uint8_t enable : 1;
    uint8_t Priority : 4;
    uint8_t errorExist : 1;
    uint8_t isLost : 1;
    uint8_t dataIsError : 1;

    fp32 frequency;
    bool_t (*dataIsErrorFun)(void);		//���ݴ����麯��
    void (*solveLostFun)(void);			//���ݶ�ʧ������
    void (*solveDataErrorFun)(void);	//���ݴ�������
} error_t;

extern bool_t toe_is_error(uint8_t err);
extern void DetectTask(void *pvParameters);
void DetectHook(uint8_t toe);
extern const error_t *getErrorListPoint(void);
extern void DetectHook(uint8_t toe);
#endif
