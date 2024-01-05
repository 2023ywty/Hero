/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       detect_task.c/h
  * @brief      
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
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
//错误码以及对应设备顺序
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

//数据丢失周期检测，数据错误逻辑检测
typedef __packed struct
{ 
    uint32_t newTime;		//上次更新的时间
    uint32_t lastTime;
    uint32_t Losttime;
    uint32_t worktime;		//上电时间
    uint16_t setOfflineTime : 12;
    uint16_t setOnlineTime : 12;
    uint8_t enable : 1;
    uint8_t Priority : 4;
    uint8_t errorExist : 1;
    uint8_t isLost : 1;
    uint8_t dataIsError : 1;

    fp32 frequency;
    bool_t (*dataIsErrorFun)(void);		//数据错误检查函数
    void (*solveLostFun)(void);			//数据丢失处理函数
    void (*solveDataErrorFun)(void);	//数据错误处理函数
} error_t;

extern bool_t toe_is_error(uint8_t err);
extern void DetectTask(void *pvParameters);
void DetectHook(uint8_t toe);
extern const error_t *getErrorListPoint(void);
extern void DetectHook(uint8_t toe);
#endif
