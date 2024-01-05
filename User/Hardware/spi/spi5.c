#include "spi5.h"

void Spi5_Init() {
    GPIO_InitTypeDef  GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI5, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //6500CS引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //磁力计RST引脚
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_SetBits(GPIOE, GPIO_Pin_2); //置低为复位有效电平

    GPIO_PinAFConfig(GPIOF,GPIO_PinSource7,GPIO_AF_SPI5);
    GPIO_PinAFConfig(GPIOF,GPIO_PinSource8,GPIO_AF_SPI5);
    GPIO_PinAFConfig(GPIOF,GPIO_PinSource9,GPIO_AF_SPI5);

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI5,ENABLE);//复位SPI5
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI5,DISABLE);//停止复位SPI5

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//串行同步时钟的空闲状态为高电平
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//定义波特率预分频的值:波特率预分频值为256
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
    SPI_InitStructure.SPI_CRCPolynomial = 7;	//stm32f4xx.h 7881 7882
    SPI_Init(SPI5, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

    SPI_Cmd(SPI5, ENABLE); //使能SPI外设
}

uint8_t SPI5_ReadWriteByte(uint8_t TxData){ //发送接收通用函数
    uint8_t retryTimer = 0;
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET){ //等待发送空闲
        retryTimer++;
        if (retryTimer > 200) return 0;
    }

    SPI_I2S_SendData(SPI5, TxData);
    retryTimer = 0;

    while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_RXNE) == RESET){ //等待接收空闲
        retryTimer++;
        if (retryTimer > 200) return 0;
    }

    return SPI_I2S_ReceiveData(SPI5);
}

//MPU6500中间件函数

uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data){
    MPU6500_NSS_Low(); //片选
    SPI5_ReadWriteByte(reg&0x7f); //最高位置0
    SPI5_ReadWriteByte(data);
    MPU6500_NSS_High(); //片选取消
    return 0;
}

uint8_t MPU6500_Read_Reg(uint8_t const reg){
    static uint8_t MPU_Rx = 0x00;
    MPU6500_NSS_Low(); //片选
    SPI5_ReadWriteByte(reg|0x80); //最高位置1
    MPU_Rx = SPI5_ReadWriteByte(0xFF);
    MPU6500_NSS_High(); //片选取消
    return MPU_Rx;
}

void MPU6500_Write_Regs(u8 reg, u8 *buf, u8 len){
	MPU6500_NSS_Low(); //片选
	SPI5_ReadWriteByte(reg&0x7f); //最高位置0
	while(len){
		SPI5_ReadWriteByte(*buf);
		len--; //长度减1
		buf++; //指针向后
	}
	MPU6500_NSS_High(); //片选取消
}

void MPU6500_Read_Regs(u8 reg, u8 *buf, u8 len){
	MPU6500_NSS_Low(); //片选
	SPI5_ReadWriteByte(reg|0x80); //最高位为1
	while(len){
		*buf=SPI5_ReadWriteByte(0xFF);
		len--; //长度减1
		buf++; //指针向后
	}
	MPU6500_NSS_High(); //片选取消
}

//IST8310中间件函数

void IST8310_Auto_Update(void){ //配置磁力计自动读取
    uint8_t readBuf[3] = {IST8310_ADDRESS|0x80, IST8310_R_MODE, 0x08|0x80}; //IIC地址（读） 起始地址 读取个数
    MPU6500_Write_Regs(MPU6500_I2C_SLV0_ADDR, readBuf, 3);
}

uint8_t IST8310_Reg_Write_By_MPU(uint8_t reg, uint8_t data){ //使用MPU6500写IST8310寄存器（SLV4寄存器功能)
    uint8_t writeBuf[4] = {IST8310_ADDRESS, reg, data, 0x80}; //IIC地址（写） 起始地址 读取个数 使能传送
    MPU6500_Write_Regs(MPU6500_I2C_SLV4_ADDR, writeBuf, 4);
    return 0;
}

uint8_t IST8310_Reg_Read_By_MPU(uint8_t reg){ //使用MPU6500读IST8310寄存器
    uint8_t readBuf[3] = {IST8310_ADDRESS|0x80, reg, 0x01|0x80}; //IIC地址（读） 起始地址 读取个数
    MPU6500_Write_Regs(MPU6500_I2C_SLV0_ADDR, readBuf, 3);
    smartDelayMs(2);
    return MPU6500_Read_Reg(MPU6500_EXT_SENS_DATA_00);
}

void IST8310_Regs_Write_By_MPU(uint8_t reg, uint8_t *data, uint8_t len){
    while (len){
        IST8310_Reg_Write_By_MPU(reg, (*data));
        reg++;
        data++;
        len--;
        smartDelayMs(2);
    }
}

void IST8310_Regs_Read_By_MPU(uint8_t reg, uint8_t *buf, uint8_t len){
    while (len){
        (*buf) = IST8310_Reg_Read_By_MPU(reg);
        reg++;
        buf++;
        len--;
    }
}
