/**************************************************************************
 * @file     	imu_ex.c
 * @brief    	陀螺仪数据处理，陀螺仪型号：维特智能WT931
 **************************************************************************/
#include "imu_ex.h"
#include "USART3_DMA.h"
#include "USART6.h"
#include "delay.h"
/**********************************接收用到的宏定义声明******************************/
#define NO_DATE 0
#define UNKNOWN_DATE 1
#define SPEED_DATE 2
#define ANGLE_DATE 3
#define ROLL_L 0
#define ROLL_H 1
#define PITCH_L 2
#define PITCH_H 3
#define YAW_L 4
#define YAW_H 5
#define T_L 6
#define T_H 7
#define SUM 8
/***********************************************************************************/

/*************************************外部接口声明***********************************/
MPU6050_t MPUEX;	//保存陀螺仪数据信息
/***********************************************************************************/

/***********************************内部变量使用声明*********************************/
u8 now_recieve = NO_DATE; 	//接收状态标志变量
u8 read_position = 0; 		//已经读取到有效信息的位置

u8 IMU_Reset_date[7][5] = {
	{0xFF, 0xAA, 0x69, 0x88, 0xB5},
	{0xFF, 0xAA, 0x02, 0x0C, 0x00},
	{0xFF, 0xAA, 0x03, 0x0B, 0x00},
	{0xFF, 0xAA, 0x24, 0x01, 0x00},
	{0xFF, 0xAA, 0x23, 0x00, 0x00},
	{0xFF, 0xAA, 0x1F, 0x00, 0x00},
	{0xFF, 0xAA, 0x04, 0x06, 0x00}
};
/***********************************************************************************/
/**
  * @name 	IMUEX_Init()
  * @brief 	陀螺仪初始化
  * @param	None
  * @return None
  */
void IMUEX_Init()
{
	USART3_Init();
	USART6_Init();
	MPUEX.filler_k = 0;
}

/**
  * @name 	IMU_dataRecieve()
  * @brief 	外部陀螺仪接收函数，在串口对应的DMA中断中调用
  * @param	gyro_rx_buffer:DMA传送的内存地址
  * @return None
  */
void IMU_dataRecieve(unsigned char *gyro_rx_buffer)
{
 	u8 recieve_num = 0; //当前读取的数在DMA数组中的位置
	if(DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1))
	{	
		DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1);
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
		
		while(recieve_num < 20)
		{
			if(now_recieve == NO_DATE)
			{ //说明没有接收到帧头，继续寻找帧头
				
				for(; recieve_num < 20; recieve_num++)
				{
					if(gyro_rx_buffer[recieve_num] == 0x55)
					{ //找到帧头，更换状态标志位，记录读取位置，结束循环，进入下 一个状态
						now_recieve = UNKNOWN_DATE;
						recieve_num++;
						recieve_num %= 20;	//下一个判断起始位
						if(recieve_num == 0)
							//如果标志位已经是最后一个数则结束读取
							return;
						
						break;
					}
				}
				
			}
			
			if(now_recieve == UNKNOWN_DATE)
			{ //确定结束数据信息类型阶段
				switch(gyro_rx_buffer[recieve_num++])
				{
					case 0x52: now_recieve = SPEED_DATE; break;
					case 0x53: now_recieve = ANGLE_DATE; break;
					default: now_recieve = NO_DATE; break;
				}
				
				if(recieve_num == 20)
					return;
			}
			
			if(now_recieve == ANGLE_DATE)
			{
				for(int i = read_position; i <= SUM; i++)
				{ 
					read_position++;
					MPUEX.angle.angle_union.angle_8b[i] = gyro_rx_buffer[recieve_num++];
					if(recieve_num == 20)
					{ //说明这组数据已经读完，结束读取
						break;
					}
				}
			}
			
			if(now_recieve == SPEED_DATE)
			{
				for(int i = read_position; i <= SUM; i++)
				{ 
					read_position++;
					MPUEX.v.v_union.v_8b[i] = gyro_rx_buffer[recieve_num++];
					if(recieve_num == 20)
					{ //说明这组数据已经读完，结束读取
						break;
					}
				}
			}
			
			//判断是否读完
			if(read_position > SUM)
			{ //说明已经读取完完整的一组数，修改标志位，更新数据
				if(now_recieve == SPEED_DATE)
				{	
					u8 the_sum = 0;
					for(int i = 0; i < 8; i++)
					{
						the_sum += MPUEX.v.v_union.v_8b[i];
					}
					the_sum += 0x55;
					the_sum += 0x52;
					if(the_sum == MPUEX.v.v_union.v_8b[8])
					{
						MPUEX.v.pitch = (MPUEX.v.v_union.v_16b[PITCH_L/2])/32768.0*2000;
						MPUEX.v.roll = (MPUEX.v.v_union.v_16b[ROLL_L/2])/32768.0*2000;
						MPUEX.pre_yaw = (MPUEX.v.v_union.v_16b[YAW_L/2])/32768.0*2000;
						MPUEX.v.yaw = MPUEX.pre_yaw * (1 - MPUEX.filler_k) + MPUEX.filler_k*MPUEX.v.yaw;
					}
				}
				else
				{
					u8 the_sum = 0;
					for(int i = 0; i < 8; i++)
					{
						the_sum += MPUEX.angle.angle_union.angle_8b[i];
					}
					the_sum += 0x55;
					the_sum += 0x53;

					if(the_sum == MPUEX.angle.angle_union.angle_8b[8])
					{
						MPUEX.angle.pitch = (MPUEX.angle.angle_union.angle_16b[PITCH_L/2])/32768.0*180;
						MPUEX.angle.roll = (MPUEX.angle.angle_union.angle_16b[ROLL_L/2])/32768.0*180;
						MPUEX.angle.yaw = (MPUEX.angle.angle_union.angle_16b[YAW_L/2])/32768.0*180;
					}
				}
				read_position = ROLL_L;
				now_recieve = NO_DATE;
			}
		}
	}
}

/**
  * @name 	IMU_ResetStep()
  * @brief 	陀螺仪复位
  * @param	step:复位的步骤，一共分为7步 发送0-6，相邻两部之间间隔100ms 必须写在主函数执行!!!!!!!
  * @return None
  */
void IMU_ResetStep(u8 step){
	USART3_Send_Buf(IMU_Reset_date[step], 5);
}

/**
  * @name 	IMU_ResetStep()
  * @brief 	陀螺仪复位 必须写在主函数执行!!!!!!!
  * @param	None
  * @return None
  */
void IMU_Reset(){
	IMU_ResetStep(0);
	delay_ms(100);
	IMU_ResetStep(1);
	delay_ms(100);
	IMU_ResetStep(2);
	delay_ms(100);
	IMU_ResetStep(3);
	delay_ms(100);
	IMU_ResetStep(4);
	delay_ms(100);
	IMU_ResetStep(5);
	delay_ms(100);
	IMU_ResetStep(6);
	delay_ms(3000);
}