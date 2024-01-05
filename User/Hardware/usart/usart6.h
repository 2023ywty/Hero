/**************************************************************************
 * @file     	usart6.h
 * @brief    	�������������Ӿ�ͨ��ʵ��
 **************************************************************************/
#ifndef _USART6_H_
#define _USART6_H_
#include "stm32f4xx.h"
#define VisionBufferLength       255


/*--------------------------------2020�Ӿ����ͨ��Э��-------------------------------------*/

//��ʼ�ֽ�,Э��̶�Ϊ0xA1
#define    VISION_FRAME_HEADER         (0xA1)

//���ȸ���Э�鶨��,���ݶγ���Ϊn��Ҫ����֡ͷ�ڶ��ֽ�����ȡ
#define    VISION_LEN_HEADER    		5         //֡ͷ��
#define    VISION_LEN_CMDID    		  2        //�����볤��  ֡����
#define    VISION_SEND_LEN_DATA 		6        //���ݶγ���,���Զ���
#define    VISION_LEN_TAIL      		2	      	//֡βCRC16
#define    VISION_SEND_LEN_PACKED   15        //���ݰ�����,���Զ���

//#define    VISION_LEN_HEADER    		5         //֡ͷ��
//#define    VISION_LEN_CMDID    		  2        //�����볤��
//#define    VISION_SEND_LEN_DATA 		17        //���ݶγ���,���Զ���
//#define    VISION_LEN_TAIL      		2	      	//֡βCRC16
//#define    VISION_SEND_LEN_PACKED   26        //���ݰ�����,���Զ���


#define    VISION_OFF         		(0)//�ر��Ӿ�
#define    VISION_AUTO           	(1)//��������
#define    VISION_BUFF_SMALL    	(2)//С������
#define    VISION_BUFF_BIG   		 	(3)//�������
#define    VISION_ROTATE   				(4)//����תС��������


typedef union
{//ö��
	u8 data[4];
	float f_data;
} float_u8;

typedef __packed struct
{
	/* ͷ */
	uint8_t   SOF;			//֡ͷ��ʼλ,�̶�Ϊ0xA1
	uint16_t  DataLen;  //���ݶγ��� n 
	uint8_t   PackedId;	//���������ţ���ʱû�ã���㷢 
	uint8_t   CRC8;			//֡ͷCRCУ��,��֤���͵�ָ������ȷ��
}extVisionSendHeader_t;



/*ID 0X001    Byte:  17   ��Ƭ��->PC���ݽṹ�� */
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


/*ID 0X002    Byte:  2   PC->��Ƭ�����ݽṹ�� */
//��Ϣ����
typedef __packed struct{
		float yawerr;
	  float piterr;
	  float distance;
	  u8  fire_flag;
	  u8  fire_mode;
	  double fire_delaytime;
}pcResponseRev_t;


//ʶ��Ŀ��
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


/*ID 0X003    Byte:  9   ���� PC->��Ƭ�����ݽṹ��*/
typedef __packed struct { 
  float yawErr; 
  float pitchErr; 
	uint8_t shootFlag; 
}buffAttackMsg_t;


/*ID 0X004��0x005  Byte:9	���顢����������  PC->��Ƭ�����ݽṹ��*/
typedef __packed struct {  
	float yawErr; 
	float pitchErr; 
	uint8_t shootFlag; 
}amorAttackMsg_t;


//PC->��Ƭ��������Ϣ�ṹ��
typedef struct
{
	pcResponseMsg_t pcResponseMsg;//PC��Ӧ�������
	buffAttackMsg_t buffAttackMsg;//�����������
	amorAttackMsg_t amorAttackMsg;//  �����������
	struct{
		uint8_t pcRes_if_updata;										//�Ӿ������Ƿ����
		uint8_t buff_if_updata;					            //�Ƿ�ʶ��Ŀ��
		uint8_t amor_if_updata;           				  //�Ƿ���
	}flag;
}VisionRecvData_t;




//��Ƭ��->PC������Ϣ�ṹ��
typedef struct
{ 
	uint16_t pstart;
	uint16_t FrameType;//֡����(2-byte) 
	uint16_t DataLen;//���ݳ���	
	uint8_t  DataSegment[6];//��Ƭ��->PC ������Ϣ�ṹ�� ���ݶ�(n-byte) 
}VisionSendData_t;  //21

typedef struct
{ 
	uint16_t pstart;
	uint16_t FrameType;//֡����(2-byte) 
	uint16_t DataLen;//���ݳ���	
	uint8_t  DataSegment[17];//��Ƭ��->PC ������Ϣ�ṹ�� ���ݶ�(n-byte) 
}VisionSendData_t_old;

extern VisionRecvData_t VisionRevData_Struct;
void USART6_Init();		//����6��ʼ��
void Vision_Send_Queue(uint16_t DataLen,uint16_t FrameType,uint8_t  *DataSegment);
void Vision_Info_Send(VisionSendData_t *VisionSendData_Struct);
void Vision_Send_Queue(uint16_t DataLen,uint16_t FrameType,uint8_t  *DataSegment);
void Vision_Rm2020_Cm0x0001_Send_Queue(float yawAngle, float yawSpeed, float pitchAngle, float pitchSpeed, u8 requestMsg);
void Vision_Rm2021_Cm0x0001_Send_Queue(float fire_speed, u8 enter_mode, u8 rotate_flag);
void Vision_Info_Rev(uint8_t *VisionRevBuffer);
#endif