#include "main.h"
#include "stm32f4xx.h"
#include "string.h"
#include "usart6.h"
#include "gimbal_task.h"
#include "timer_send_task.h"


/********************************全局变量*************************************/
VisionRecvData_t VisionRevData_Struct;//PC->单片机 接受信息结构体

/*******************************读取阶段中间数据定义******************************************/
u8 VisionDataBuffer[VisionBufferLength]={0}; 				//DMA容器 读取裁判系统原始数据
/********************************************************************************************/
u8 a=0;


/****************************************读取阶段中间函数声明******************************************/
//CRC校验相关函数
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);//
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);	//生成8位CRC校验码
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);	//生成16位CRC校验

//串口发送函数
void USART6_Send_Data(u8 data);

//信息接受处理函数
void Vision_Info_Rev(uint8_t *VisionRevBuffer);

//信息发送，发送方法为调用发送队列函数，之后队列自动会调用视觉信息发送发函数
void Vision_Rm2020_Cm0x0001_Send_Queue(float yawAngle, float yawSpeed, float pitchAngle, float pitchSpeed, u8 requestMsg);//针对2020赛季0x0001函数发送
void Vision_Send_Queue(uint16_t DataLen,uint16_t FrameType,uint8_t  *DataSegment);//发送视觉数据到队列
void Vision_Info_Send(VisionSendData_t *VisionSendData_Struct);//视觉队列数据发送函数
/****************************************************************************************************/


/**
  * @name		USART6_Init()
  * @brief 	串口6初始化
  * @param  None
  * @return None
  */

void USART6_Init(void)
{
	//时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG | RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9, GPIO_AF_USART6);	
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource14, GPIO_AF_USART6);
	
	//结构体定义
	GPIO_InitTypeDef GPIO_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART6_InitStruct;
	
	//
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOG, &GPIO_InitStruct);
	
	USART_DeInit(USART6);
	USART6_InitStruct.USART_BaudRate = 115200;
	USART6_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART6_InitStruct.USART_StopBits = USART_StopBits_1;
	USART6_InitStruct.USART_Parity = USART_Parity_No; //偶校验
	USART6_InitStruct.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	USART6_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART6,&USART6_InitStruct);
	USART_Cmd(USART6,ENABLE);
	
	//NVIC配置
	NVIC_InitStructure.NVIC_IRQChannel						=	USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	10;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	0;
	NVIC_Init(&NVIC_InitStructure);
	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);

	DMA_InitTypeDef DMA_InitStruct;
	DMA_DeInit(DMA2_Stream1);
	DMA_InitStruct.DMA_Channel = DMA_Channel_5;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(VisionDataBuffer);
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStruct.DMA_BufferSize = VisionBufferLength;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//8位字节传输
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream1,&DMA_InitStruct);
	DMA_Cmd(DMA2_Stream1,ENABLE);
}


void USART6_IRQHandler(void)
{
	if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
	{ 
		uint8_t UARTtemp;
		UARTtemp = USART6->SR;
		UARTtemp = USART6->DR;
		DMA_Cmd(DMA2_Stream1, DISABLE);
		Vision_Info_Rev(VisionDataBuffer);//数据接受函数
		//重启DMA
		USART_ClearITPendingBit(USART6, USART_IT_IDLE); 
		DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
		while(DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);
		DMA_SetCurrDataCounter(DMA2_Stream1, VisionBufferLength);
		DMA_Cmd(DMA2_Stream1, ENABLE);
	}
}


/**
  * @brief  发送视觉指令到队列、主要调用这个函数发送数据。
  * @param  
  * @retval void
  */
void Vision_Send_Queue(uint16_t DataLen,uint16_t FrameType,uint8_t  *DataSegment)
{
	VisionSendData_t VisionSendData_Struct;
	VisionSendData_Struct.DataLen=DataLen;
	memcpy(&VisionSendData_Struct.DataSegment,DataSegment,DataLen);
	VisionSendData_Struct.FrameType=FrameType;
	xQueueSend(VISION_Send_Queue,&VisionSendData_Struct,1);//向队列中填充内容
}






/**
  * @brief  针对RM20200 x0001 指令  发送数据到函数
  * @param  
	* @retval 单个数据先发送低位再发送高位
		u8 requestMsg
		#define    VISION_OFF         		(0)//关闭视觉
		#define    VISION_AUTO           	(1)//自瞄请求
		#define    VISION_BUFF_SMALL    	(2)//小幅请求
		#define    VISION_BUFF_BIG   		 	(3)//大幅请求
		#define    VISION_ROTATE   				(4)//打旋转小陀螺请求
		#enter_mode  进入模式 
		#rotate_flag 陀螺标志   0：否   1：是
  */
void Vision_Rm2021_Cm0x0001_Send_Queue(float fire_speed, u8 enter_mode, u8 rotate_flag)
{
	float_u8  vision_data21;
	static VisionSendData_t VisionSendData_Struct_Rm2021;//注意为静态全局变量，因为下面函数传递的是指针
	
	VisionSendData_Struct_Rm2021.FrameType=0x0001;
	VisionSendData_Struct_Rm2021.DataLen=0x0006;//对应十进制6位,
	
	//数据段
	vision_data21.f_data=fire_speed;
	VisionSendData_Struct_Rm2021.DataSegment[0]=vision_data21.data[3];
	VisionSendData_Struct_Rm2021.DataSegment[1]=vision_data21.data[2];
	VisionSendData_Struct_Rm2021.DataSegment[2]=vision_data21.data[1];
	VisionSendData_Struct_Rm2021.DataSegment[3]=vision_data21.data[0];
	
	VisionSendData_Struct_Rm2021.DataSegment[4]=enter_mode;  //请求命令
	
	VisionSendData_Struct_Rm2021.DataSegment[5]=rotate_flag;  //是否陀螺模式
		
	 //发送信息
  Vision_Info_Send(&VisionSendData_Struct_Rm2021);   //不经过队列直接发送
}



/**
  * @brief  单片机发送机器人数据给视觉
  * @param  NULL
  * @retval void
  * @attention NULL 
  */

void Vision_Info_Send(VisionSendData_t *VisionSendData_Struct){
  u8 i;//循环发送次数
	extVisionSendHeader_t usart6VisionSendHeader_Struct;//头
	u8 vision_send_pack[255] = {0};  //发送数组
	if(	VisionSendData_Struct==NULL)
			return ;
	
	//帧头
	usart6VisionSendHeader_Struct.SOF=VISION_FRAME_HEADER;//固定头 
	usart6VisionSendHeader_Struct.DataLen=VisionSendData_Struct->DataLen;//数据段长度
	usart6VisionSendHeader_Struct.PackedId=0;//多包传输序号，暂时没用，随便发 
	memcpy( vision_send_pack, &usart6VisionSendHeader_Struct, VISION_LEN_HEADER);//写入帧头

	//因为CRC校验与之前数据有关，所以与上面的位置不能交换
	Append_CRC8_Check_Sum(vision_send_pack,VISION_LEN_HEADER);//帧头CRC8校验协议
	
	//帧类型
	vision_send_pack[5]=(uint8_t)((VisionSendData_Struct->FrameType&0xFf));//二进低八位
	vision_send_pack[6]=(uint8_t)((VisionSendData_Struct->FrameType)>>8);//二进制高八位
	
	//数据段
	memcpy(vision_send_pack + VISION_LEN_HEADER+VISION_LEN_CMDID, VisionSendData_Struct->DataSegment, ((vision_send_pack[2] << 8)|vision_send_pack[1]) );//写入数据
		
	//帧尾CRC16校验协议
	Append_CRC16_Check_Sum(vision_send_pack, VISION_LEN_HEADER+VISION_LEN_CMDID+VISION_LEN_TAIL+((vision_send_pack[2] << 8)|vision_send_pack[1]));

	//将打包好的数据通过串口移位发送到裁判系统
	
	for (i = 0;  i <  (VISION_LEN_HEADER+VISION_LEN_CMDID+VISION_LEN_TAIL+((vision_send_pack[2] << 8)|vision_send_pack[1])); i++)
	{		
		USART6_Send_Data ( vision_send_pack[i] );
	}

	memset(vision_send_pack, 0x00, 255);//数组置零


}



/**
  * @brief  PC发送单片机数据接受
  * @param  NULL
  * @retval void
  * @attention 
  */
void Vision_Info_Rev(uint8_t *VisionRevBuffer){
	u16 start_pos = 0;	
	u8 retval_tf = FALSE;  //数据正确与否标志,每次调用读取裁判系统数据函数都先默认为错误
	uint16_t PC_length;    //统计一帧数据长度 
	u16 DATA_LEN = 0;
	
	//无数据包，则不作任何处理
	if (VisionRevBuffer == NULL)
	{
		return ;
	}

	//判断帧头数据是否为0xA1
	if (VisionRevBuffer[start_pos] == VISION_FRAME_HEADER)
	{
		//帧头CRC8校验
		if(Verify_CRC8_Check_Sum(VisionRevBuffer, 5 ) == TRUE)
		{
			
//			DATA_LEN = VisionRevBuffer[DATA_LENGTH] | (VisionRevBuffer[DATA_LENGTH + 1] << 8);			
//      统计一帧数据长度,用于CR16校验
//			PC_length = DATA_LEN + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			
		 //接收就一种类型 直接令长度为常数31  帧头‘5’+帧类型‘2’+数据段’22‘+帧尾16CRC’2‘
			  PC_length =31 ;

			//帧尾CRC16校验
			if(Verify_CRC16_Check_Sum( VisionRevBuffer, 31 ) == TRUE)
			{
				retval_tf = TRUE;//都校验过了则说明数据可用 
				memcpy(&VisionRevData_Struct.pcResponseMsg,  (VisionRevBuffer+ 7 ), 22);											
				VisionRevData_Struct.flag.pcRes_if_updata=1;	
				VisionRevData_Struct.flag.amor_if_updata=1;	

				//首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA1,用来判断一个数据包是否有多帧数据
				if (VisionRevBuffer[PC_length] == 0xA1)
				{
					//如果一个数据包出现了多帧数据,则再次读取
					Vision_Info_Rev(VisionRevBuffer + PC_length);
				}
			}
		}
	}
	
}




///**
//  * @brief  PC发送单片机数据接受
//  * @param  NULL
//  * @retval void
//  * @attention 
//  */
//void Vision_Info_Rev(uint8_t *VisionRevBuffer){
//		u16 start_pos = 0;		//当前帧起始位置
//		u16 next_start_pos=0;		//下一帧起始位置，由于裁判系统有时候会连续发送，因此一次串口空闲中断可能会接收多个帧	
//	
//		if(VisionRevBuffer==NULL)
//				return ;
//		/*开始读取,多个贞可能在一起发送*/
//		while(start_pos<VisionBufferLength && VisionRevBuffer[start_pos] == VISION_FRAME_HEADER)
//		{
//				//修改帧头，防止下次再次接收该帧
//        //VisionDataBuffer[start_pos]++;	//注意不能修改，因为CRC校验原理为前面数据生成校验码
//				//有时候会重复接收帧头多次，去掉重复的帧头
//			while(VisionRevBuffer[start_pos + 1] == VISION_FRAME_HEADER)
//			
//			//帧头CRC8校验
//			if(1)//Verify_CRC8_Check_Sum( (VisionRevBuffer+start_pos), VISION_LEN_HEADER ))
//			{
//				u16 sum_length =((VisionRevBuffer[start_pos + 2] << 8)|VisionRevBuffer[start_pos + 1]) +VISION_LEN_HEADER+VISION_LEN_CMDID+VISION_LEN_TAIL;	//本帧总长度 数据段+帧头+帧
//				next_start_pos = sum_length + start_pos;
//				if(next_start_pos>= VisionBufferLength)
//				{ //越界终止，几乎不会越界
//					start_pos = 0;
//					break;
//				}
//						//帧尾CRC16校验
//				if(1)//Verify_CRC16_Check_Sum((VisionRevBuffer+start_pos),sum_length) == 1)
//				{
//					u16 data_startPos = start_pos + VISION_LEN_HEADER+VISION_LEN_CMDID;	                	//获取数据位起始地址
//					u16 cmd_id = ((VisionRevBuffer[start_pos + 6] << 8)|VisionRevBuffer[start_pos + 5]);	//获取帧类型
//					switch(cmd_id)
//					{
//						case 0x0002:
//							memcpy(&VisionRevData_Struct.pcResponseMsg, (VisionRevBuffer + data_startPos), 2);
//							VisionRevData_Struct.flag.pcRes_if_updata=1;
//						 
//							break;
//						case 0x0003:
//							memcpy(&VisionRevData_Struct.buffAttackMsg, (VisionRevBuffer + data_startPos), 9);
//							VisionRevData_Struct.flag.buff_if_updata=1;
//							a=VisionRevData_Struct.buffAttackMsg.yawErr;					
//						break;
//						
//						case 0x0004:
//						case 0x0005:
//							memcpy(&VisionRevData_Struct.amorAttackMsg, (VisionRevBuffer + data_startPos), 9);
//  							VisionRevData_Struct.flag.amor_if_updata=1;
//							break;
//					}
//				}
//				start_pos = next_start_pos;		//下次假定帧头位置赋值
//			}//帧头CRC8校验
//		}//while	
//	memset(VisionRevBuffer, 0, VisionBufferLength);
//}





/**
  * @name 	USART6_Send_Data()
  * @brief 	串口6发送一个字节
  * @param	发送的字节内容
  * @return None
  */  
void USART6_Send_Data(u8 data)
{
	while(USART_GetFlagStatus(USART6,USART_FLAG_TC)==RESET){}; 
		USART6->DR=(data & (uint16_t)0x01FF);
}


//     ####################################################################################################


const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] =
{
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
	
};



unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8)
{
    unsigned char ucIndex;
    while (dwLength--)
    {
        ucIndex = ucCRC8^(*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return(ucCRC8);
}


/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2)) 
        return 0;
    ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT);
    return ( ucExpected == pchMessage[dwLength-1] );
}


/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2)) 
        return;
    ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT);
    pchMessage[dwLength-1] = ucCRC;
}

uint16_t CRC_INIT = 0xffff;

const uint16_t wCRC_Table[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};


/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == 0)
    {
        return 0xFFFF;
    }
    while(dwLength--)
    {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^
        (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}


/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}


/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength)
{
    uint16_t wCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return;
    }
    wCRC = Get_CRC16_Check_Sum ( (u8 *)pchMessage, dwLength-2, CRC_INIT );
    pchMessage[dwLength-2] = (u8)(wCRC & 0x00ff);
    pchMessage[dwLength-1] = (u8)((wCRC >> 8)& 0x00ff);
}
/***********************************    ↑    DJI提供的CRC校检函数   ↑  ***********************************/   