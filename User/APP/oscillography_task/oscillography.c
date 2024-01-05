/**************************************************************************
 * @file     	oscillography.h/.c
 * @brief    	通过串口2连接上位机显示波形调参使用
 **************************************************************************/
 #include "oscillography.h"
 #include "stm32f4xx.h"
 #include "usart2.h"
 #include "FreeRTOS.h"
 #include "task.h"
 #include "usart2.h"
 #include "gimbal_task.h"
 #include "chassis_task.h"
 #include "shoot_task.h"
 #include "Detect_Task.h"
 #include "usart6.h"
 s16 oscillographyData[10];
 int gimal_yaw_enconde;

VisionRecvData_t VisionRevData_test;
 //上位机发送数据给单片机时的回调函数
void upComputer_dataRecieve(s16 roll_p, s16 roll_i, s16 roll_d, s16 pitch_p, s16 pitch_i, s16 pitch_d, s16 yaw_p, s16 yaw_i, s16 yaw_d){

}
//向上位机发送一组数据，用于显示
void oscillography_sendData(s16 *dataSend, u8 len){
	u8 i = 0;
	u8 data[20];
	if(len > 10)
		len=10;
	for(i=0;i < len; i++){
		data[2*i]=dataSend[i];
		data[2*i+1]=(u8)(dataSend[i]>>8);
//		data[2*i] = (u8)(((s16)(dataSend[i]))>>8);
//		data[2*i + 1] = (u8)((s16)(dataSend[i]));
	}
	Send_data8(data, len*2, 1);
}

void oscillography_task(void *pvParameters)
{

	USART2_Init();
	vTaskDelay(100);
	while(1)
	{
		if(1)
		{
			oscillographyData[0] =(s16)chassisTaskStructure.motor[CHASSIS_CM1].filterSpeed;;
			oscillographyData[1] =(s16)chassisTaskStructure.motor[CHASSIS_CM2].filterSpeed;
			oscillographyData[2] =(s16)chassisTaskStructure.motor[CHASSIS_CM3].filterSpeed;
			oscillographyData[3] =(s16)chassisTaskStructure.motor[CHASSIS_CM4].filterSpeed;//自瞄模式下的YAW轴期望
			oscillographyData[4] =(s16)chassisTaskStructure.motor[CHASSIS_CM1].speedSet;//当前云台所处的模式
			oscillographyData[5] =(s16)chassisTaskStructure.motor[CHASSIS_CM2].speedSet;//是否为自瞄模式
			oscillographyData[6] =(s16)chassisTaskStructure.motor[CHASSIS_CM3].speedSet;//YAW轴电机期望值
			oscillographyData[7] =(s16)chassisTaskStructure.motor[CHASSIS_CM4].speedSet;//YAW轴电机实际值
			oscillographyData[8] =(s16)gimbalTaskStructure.yawMotor.angle_set[GYRO];//P轴电机期望值
			oscillographyData[9] =(s16)gimbalTaskStructure.yawMotor.angle[GYRO];//P轴电机实际值
		}
		oscillography_sendData(oscillographyData, 10);
		vTaskDelay(5);
	}
}
//1591.54
//15.9154
//31.83
