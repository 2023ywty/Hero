/**************************************************************************
 * @file     	usart6.h
 * @brief    	串口六，用于视觉通信实验
 **************************************************************************/
#ifndef _USART6_H_
#define _USART6_H_
#include "stm32f4xx.h"
#define VisionBufferLength       255


/*--------------------------------2020视觉电控通信协议-------------------------------------*/

//起始字节,协议固定为0xA1
#define    VISION_FRAME_HEADER         (0xA1)

//长度根据协议定义,数据段长度为n需要根据帧头第二字节来获取
#define    VISION_LEN_HEADER    		5         //帧头长
#define    VISION_LEN_CMDID    		  2        //命令码长度  帧类型
#define    VISION_SEND_LEN_DATA 		6        //数据段长度,可自定义
#define    VISION_LEN_TAIL      		2	      	//帧尾CRC16
#define    VISION_SEND_LEN_PACKED   15        //数据包长度,可自定义

//#define    VISION_LEN_HEADER    		5         //帧头长
//#define    VISION_LEN_CMDID    		  2        //命令码长度
//#define    VISION_SEND_LEN_DATA 		17        //数据段长度,可自定义
//#define    VISION_LEN_TAIL      		2	      	//帧尾CRC16
//#define    VISION_SEND_LEN_PACKED   26        //数据包长度,可自定义


#define    VISION_OFF         		(0)//关闭视觉
#define    VISION_AUTO           	(1)//自瞄请求
#define    VISION_BUFF_SMALL    	(2)//小幅请求
#define    VISION_BUFF_BIG   		 	(3)//大幅请求
#define    VISION_ROTATE   				(4)//打旋转小陀螺请求


typedef union
{//枚举
	u8 data[4];
	float f_data;
} float_u8;

typedef __packed struct
{
	/* 头 */
	uint8_t   SOF;			//帧头起始位,固定为0xA1
	uint16_t  DataLen;  //数据段长度 n 
	uint8_t   PackedId;	//多包传输序号，暂时没用，随便发 
	uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
}extVisionSendHeader_t;



/*ID 0X001    Byte:  17   单片机->PC数据结构体 */
typedef __packed struct {
	float fire_speed;  
	u8 enter_mode;
	u8 rotate_flag;
}controlSysMsg_t; 

//typedef __packed struct {
//	float yawAngle;  
//	float yawSpeed;
//	float pitchAngle;


//	uint8_t requestMsg; 
//}controlSysMsg_t; 


/*ID 0X002    Byte:  2   PC->单片机数据结构体 */
//信息反馈
typedef __packed struct{
		float yawerr;
	  float piterr;
	  float distance;
	  u8  fire_flag;
	  u8  fire_mode;
	  double fire_delaytime;
}pcResponseRev_t;


//识别目标
//typedef __packed struct{
//	uint8_t catchFlag:1; 
//	uint8_t upGimbalCatchFlag:1; 
//}pcResponseCatch_t;


//typedef __packed struct {
//	pcResponseRev_t pcResponseRev;
//	pcResponseCatch_t pcResponseCatch;
//} pcResponseMsg_t; 


typedef __packed struct {
	  float yawerr;
	  float piterr;
	  float distance;
	  u8  fire_flag;
	  u8  fire_mode;
	  double fire_delaytime;
} pcResponseMsg_t; 


/*ID 0X003    Byte:  9   打大幅 PC->单片机数据结构体*/
typedef __packed struct { 
  float yawErr; 
  float pitchErr; 
	uint8_t shootFlag; 
}buffAttackMsg_t;


/*ID 0X004、0x005  Byte:9	自瞄、打陀螺数据  PC->单片机数据结构体*/
typedef __packed struct {  
	float yawErr; 
	float pitchErr; 
	uint8_t shootFlag; 
}amorAttackMsg_t;


//PC->单片机接受信息结构体
typedef struct
{
	pcResponseMsg_t pcResponseMsg;//PC回应相关数据
	buffAttackMsg_t buffAttackMsg;//打大幅相关数据
	amorAttackMsg_t amorAttackMsg;//  自瞄相关数据
	struct{
		uint8_t pcRes_if_updata;										//视觉数据是否更新
		uint8_t buff_if_updata;					            //是否识别到目标
		uint8_t amor_if_updata;           				  //是否发射
	}flag;
}VisionRecvData_t;




//单片机->PC发射信息结构体
typedef struct
{ 
	uint16_t pstart;
	uint16_t FrameType;//帧类型(2-byte) 
	uint16_t DataLen;//数据长度	
	uint8_t  DataSegment[6];//单片机->PC 发送信息结构体 数据段(n-byte) 
}VisionSendData_t;  //21

typedef struct
{ 
	uint16_t pstart;
	uint16_t FrameType;//帧类型(2-byte) 
	uint16_t DataLen;//数据长度	
	uint8_t  DataSegment[17];//单片机->PC 发送信息结构体 数据段(n-byte) 
}VisionSendData_t_old;

extern VisionRecvData_t VisionRevData_Struct;
void USART6_Init();		//串口6初始化
void Vision_Send_Queue(uint16_t DataLen,uint16_t FrameType,uint8_t  *DataSegment);
void Vision_Info_Send(VisionSendData_t *VisionSendData_Struct);
void Vision_Send_Queue(uint16_t DataLen,uint16_t FrameType,uint8_t  *DataSegment);
void Vision_Rm2020_Cm0x0001_Send_Queue(float yawAngle, float yawSpeed, float pitchAngle, float pitchSpeed, u8 requestMsg);
void Vision_Rm2021_Cm0x0001_Send_Queue(float fire_speed, u8 enter_mode, u8 rotate_flag);
void Vision_Info_Rev(uint8_t *VisionRevBuffer);
#endif