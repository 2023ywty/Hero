#ifndef _SPI5_H
#define _SPI5_H

#include "main.h"
#include "stm32f4xx.h"
#include "delay.h"
#include "mpu6500_reg.h"
#include "IST8310_reg.h"

void Spi5_Init(void);	//SPI5初始化函数

uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data); //写单个寄存器
uint8_t MPU6500_Read_Reg(uint8_t const reg); //读单个寄存器
void MPU6500_Write_Regs(u8 reg, u8 *buf, u8 len); //写多个寄存器
void MPU6500_Read_Regs(u8 reg, u8 *buf, u8 len); //读多个寄存器

void IST8310_Auto_Update(void);
uint8_t IST8310_Reg_Write_By_MPU(uint8_t reg, uint8_t data); //写单个寄存器
uint8_t IST8310_Reg_Read_By_MPU(uint8_t reg); //读单个寄存器
void IST8310_Regs_Write_By_MPU(uint8_t reg, uint8_t *data, uint8_t len); //写多个寄存器
void IST8310_Regs_Read_By_MPU(uint8_t reg, uint8_t *buf, uint8_t len); //读多个寄存器

#define MPU6500_NSS_Low() GPIO_WriteBit(GPIOF, GPIO_Pin_6, Bit_RESET)
#define MPU6500_NSS_High() GPIO_WriteBit(GPIOF, GPIO_Pin_6, Bit_SET)

#define IST8310_NSS_Low() GPIO_WriteBit(GPIOE, GPIO_Pin_2, Bit_RESET)
#define IST8310_NSS_High() GPIO_WriteBit(GPIOE, GPIO_Pin_2, Bit_SET)

#endif
