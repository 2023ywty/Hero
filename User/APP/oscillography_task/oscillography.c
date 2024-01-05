/**************************************************************************
 * @file     	oscillography.h/.c
 * @brief    	ͨ������2������λ����ʾ���ε���ʹ��
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
 //��λ���������ݸ���Ƭ��ʱ�Ļص�����
void upComputer_dataRecieve(s16 roll_p, s16 roll_i, s16 roll_d, s16 pitch_p, s16 pitch_i, s16 pitch_d, s16 yaw_p, s16 yaw_i, s16 yaw_d){

}
//����λ������һ�����ݣ�������ʾ
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
			oscillographyData[3] =(s16)chassisTaskStructure.motor[CHASSIS_CM4].filterSpeed;//����ģʽ�µ�YAW������
			oscillographyData[4] =(s16)chassisTaskStructure.motor[CHASSIS_CM1].speedSet;//��ǰ��̨������ģʽ
			oscillographyData[5] =(s16)chassisTaskStructure.motor[CHASSIS_CM2].speedSet;//�Ƿ�Ϊ����ģʽ
			oscillographyData[6] =(s16)chassisTaskStructure.motor[CHASSIS_CM3].speedSet;//YAW��������ֵ
			oscillographyData[7] =(s16)chassisTaskStructure.motor[CHASSIS_CM4].speedSet;//YAW����ʵ��ֵ
			oscillographyData[8] =(s16)gimbalTaskStructure.yawMotor.angle_set[GYRO];//P��������ֵ
			oscillographyData[9] =(s16)gimbalTaskStructure.yawMotor.angle[GYRO];//P����ʵ��ֵ
		}
		oscillography_sendData(oscillographyData, 10);
		vTaskDelay(5);
	}
}
//1591.54
//15.9154
//31.83
