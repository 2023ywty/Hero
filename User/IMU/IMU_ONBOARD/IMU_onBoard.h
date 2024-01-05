#ifndef _IMU_ONBOARD_
#define _IMU_ONBOARD_

#include <math.h>
#include "robot.h"
#include "stm32f4xx.h"
#include "spi5.h"
#include "sys.h"
#include "IST8310_reg.h"
#include "mpu6500_reg.h"
#include "FreeRTOS.h"
#include "task.h"
#include "AHRS.h"

#define REG_VALID_DATA_OFFSET 1
//修改以下宏定义以修改陀螺仪测量极限（精度）
#define MPU6500_GYRO_RANGE_1000
#define MPU6500_ACCEL_RANGE_4G

/*数据存储结构体声明*/
typedef struct{
    int16_t imuStatus; //IMU状态寄存器
    int16_t ax;
    int16_t ay;
    int16_t az; //加速度计三轴
    int16_t gx;
    int16_t gy;
    int16_t gz; //陀螺仪三轴
    int16_t mx;
    int16_t my;
    int16_t mz; //磁场计三轴
    int16_t temp; //温度
    
    float accel[3];
    float gyro[3];
    float mag[3];
    float accel_offset[3]; //加速度计三轴偏移量
    float gyro_offset[3]; //陀螺仪三轴偏移量
    float mag_offset[3]; //磁场计三轴偏移量
    float temp_offset; //温度偏移量
    
} IMUPreDataTypedef;

typedef struct{
    float gyro[3];
    float accel[3];
    float mag[3];
    
    float rol;
    float pit;
    float yaw; //欧拉角
} IMUTypedef;

extern IMUPreDataTypedef imu_data;
extern IMUTypedef imu_;

//  陀螺仪原始数据转换成rad/s 陀螺仪范围可以在h文件中修改
#ifdef MPU6500_GYRO_RANGE_2000
#define MPU_GYRO_RANGLE 0x18
#define GYRO_SEN 0.00106526443603169529841533860381f

#elif defined(MPU6500_GYRO_RANGE_1000)
#define MPU_GYRO_RANGLE 0x10
#define GYRO_SEN 0.0005326322180158476492076f

#elif defined(MPU6500_GYRO_RANGE_500)
#define MPU_GYRO_RANGLE 0x08
#define GYRO_SEN 0.0002663161090079238246038f

#elif defined(MPU6500_GYRO_RANGE_250)
#define MPU_GYRO_RANGLE 0x00
#define GYRO_SEN 0.000133158054503961923019f

#else
#error "Please set the right range of gyro (2000 , 1000, 500 or 250)"
#endif

//  加速度计原始数据转换成m/s2 加速度计范围可以在h文件中修改
#ifdef MPU6500_ACCEL_RANGE_2G
#define MPU_ACCEL_RANGLE 0x00
#define ACCEL_SEN 0.00059814453125f

#elif defined(MPU6500_ACCEL_RANGE_4G)
#define MPU_ACCEL_RANGLE 0x08
#define ACCEL_SEN 0.0011962890625f

#elif defined(MPU6500_ACCEL_RANGE_8G)
#define MPU_ACCEL_RANGLE 0x10
#define ACCEL_SEN 0.002392578125f

#elif defined(MPU6500_ACCEL_RANGE_16G)
#define MPU_ACCEL_RANGLE 0x18
#define ACCEL_SEN 0.00478515625f

#else
#error "Please set the right range of accel (16G , 8G, 4G or 2G)"
#define

#endif

#define IMU_BOARD_INSTALL_SPIN_MATRIX                            \
                                        { 0.0f, 1.0f, 0.0f},    \
                                        {-1.0f, 0.0f, 0.0f},    \
                                        { 0.0f, 0.0f, 1.0f}     
#define IST_BOARD_INSTALL_SPIN_MATRIX                            \
                                        { 1.0f, 0.0f, 0.0f},    \
                                        { 0.0f, 1.0f, 0.0f},    \
                                        { 0.0f, 0.0f, 1.0f}   

uint8_t MPU6500_Init(void);
uint8_t IST8310_Init(void);
void IMU_Update_Data(IMUPreDataTypedef* rawData);
void IMU_Handle_Data(IMUPreDataTypedef* rawData, IMUTypedef* targetData);
void imu_ahrsCalculate(fp32 quat[4], const fp32 timing_time, const fp32 gyro[3], const fp32 accel[3], const fp32 mag[3]);
                                        void imu_ahrsCalculate_IMUonly(fp32 quat[4], const fp32 timing_time, const fp32 gyro[3], const fp32 accel[3]);
void imu_EulerAngleUpdate(const fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);

#endif
