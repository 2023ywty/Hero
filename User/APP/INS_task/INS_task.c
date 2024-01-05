/**************************************外置陀螺仪********************************/
/*注：由于可能是通信延迟原因，暂时仅用外置陀螺仪的yaw轴角度*/

#include "INS_task.h"
#include  "IMU_ext.h"

//#define USING_DJIAHRS_ALG //DJI-AHRS 9轴融合，存在与实际转动角度有误差问题（推荐使用）
//#define USING_AHRS_ALGO //AHRS 9轴融合，存在与实际转动角度不成正比问题（不推荐使用）
#define USING_AHRS_ALGO_IMU_ONLY //AHRS 6轴融合，存在运动后零漂问题（兼容性好）

//加速度计低通滤波参数
static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
IMUPreDataTypedef imu_data; //原始数据和偏移量
IMUTypedef imu_; //输出数据
IMUTypedef imuu_;
extern raw_t rawa;
static fp32 quat[4]; //四元数


void INS_task(void *pvParameters){
    portTickType currentTime;
    currentTime = xTaskGetTickCount();
    vTaskDelay(100);
    while(1){

				datatrans(&rawa,&imu_);			 	
			 				
        IMU_Update_Data(&imu_data); //数据接收
        IMU_Handle_Data(&imu_data, &imu_); //数据处理
        
        /**********自动校准陀螺仪**********/
        static uint8_t calibrateState;
        static uint16_t caliStableTime;
        static float gyroCaliData[3];
        if((imu_data.imuStatus & 0x40) == 0x40 && !calibrateState) { //移动且没有校准
//            buzzerOn(10, 1, 40);
            caliStableTime = 0;
            for(uint8_t i=0; i<3; i++)
                gyroCaliData[i] = 0;
        }else if(!calibrateState){ //静止且没有校准
            caliStableTime += IMU_TASK_MS;
            if(caliStableTime >= 1000){ //改动处
                for(uint8_t i=0; i<3; i++)
                    gyroCaliData[i] += imu_data.gyro[i];
            }
        }
        if(!calibrateState && caliStableTime >= 3000){
            calibrateState = 1; //校准完成
            for(uint8_t i=0; i<3; i++)
                imu_data.gyro_offset[i] -= gyroCaliData[i] / (caliStableTime / (double)IMU_TASK_MS);
//            buzzerOn(40, 1, 30);
        }
            
        /**********角度计算**********/
        static uint8_t inited;
        if(!inited) { //如果没有初始化则先初始化
            inited = 1;
            AHRS_init(quat, imu_.accel, imu_.mag);
            accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = imu_.accel[0];
            accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = imu_.accel[1];
            accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = imu_.accel[2];
        }
        else {
            accel_fliter_1[0] = accel_fliter_2[0];
            accel_fliter_2[0] = accel_fliter_3[0]; //加速度计低通滤波
            accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + imu_.accel[0] * fliter_num[2];

            accel_fliter_1[1] = accel_fliter_2[1];
            accel_fliter_2[1] = accel_fliter_3[1]; //加速度计低通滤波
            accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + imu_.accel[1] * fliter_num[2];

            accel_fliter_1[2] = accel_fliter_2[2];
            accel_fliter_2[2] = accel_fliter_3[2]; //加速度计低通滤波
            accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + imu_.accel[2] * fliter_num[2];
            
            #ifdef USING_DJIAHRS_ALG
            AHRS_update(quat, (IMU_TASK_MS/1000.0f), imu_.gyro, accel_fliter_3, imu_.mag);
            #elif defined(USING_AHRS_ALGO)
            imu_ahrsCalculate(quat, (IMU_TASK_MS/1000.0f), imu_.gyro, accel_fliter_3, imu_.mag);
            #elif defined(USING_AHRS_ALGO_IMU_ONLY)
            imu_ahrsCalculate_IMUonly(quat, (IMU_TASK_MS/1000.0f), imu_.gyro, accel_fliter_3);
            #endif
        }
        fp32 tPitch, tRoll, tYaw;
        #ifdef USING_DJIAHRS_ALG
        get_angle(quat, &tPitch, &tRoll, &tYaw); //根据四元数计算欧拉角 输出弧度制
        #else
        imu_EulerAngleUpdate(quat, &tPitch, &tRoll, &tYaw); //根据四元数计算欧拉角 输出弧度制
        #endif  

//        imu_.pit = tYaw * 57.2957f; 
//        imu_.rol = tRoll * 57.2957f;
//				imu_.yaw = imuu_.yaw;

        
        /**********准备任务切换**********/
        if(robotInf.modeStep == (robot_init_step)(ROBOT_INIT_IMU)){
            static uint16_t initStableTime, targetInitStableTime = 0;
            if(1) initStableTime += IMU_TASK_MS; //等待数据稳定
            #ifdef USING_DJIAHRS_ALG 
            targetInitStableTime = 0;
            #elif defined(USING_AHRS_ALGO)
            targetInitStableTime = 3000;
            #elif defined(USING_AHRS_ALGO_IMU_ONLY)
            targetInitStableTime = 1000;
            #endif
            if(1) robotInf.modeStep = (robot_init_step)(ROBOT_INIT_IMU + 1); //IMU初始化完成 继续机器人的其它功能初始化
        }
        
        vTaskDelayUntil(&currentTime, IMU_TASK_MS);
    }
	}

	
	
	
/*********************************************************A板陀螺仪代码*********************************************************************/
	
//#include "INS_task.h"

////#define USING_DJIAHRS_ALG //DJI-AHRS 9轴融合，存在与实际转动角度有误差问题（推荐使用）
////#define USING_AHRS_ALGO //AHRS 9轴融合，存在与实际转动角度不成正比问题（不推荐使用）
//#define USING_AHRS_ALGO_IMU_ONLY //AHRS 6轴融合，存在运动后零漂问题（兼容性好）

////加速度计低通滤波参数
//static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
//static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
//static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
//static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
//IMUPreDataTypedef imu_data; //原始数据和偏移量
//IMUTypedef imu_; //输出数据
//static fp32 quat[4]; //四元数

//int16_t debugPitch, debugRoll, debugYaw;
//int16_t debugVx, debugVy, debugVz;

//void INS_task(void *pvParameters){
//    portTickType currentTime;
//    currentTime = xTaskGetTickCount();
//    vTaskDelay(100);
//    while(1){
//        IMU_Update_Data(&imu_data); //数据接收
//        IMU_Handle_Data(&imu_data, &imu_); //数据处理
//        
//        /**********自动校准陀螺仪**********/
//        static uint8_t calibrateState;
//        static uint16_t caliStableTime;
//        static float gyroCaliData[3];
//        if((imu_data.imuStatus & 0x40) == 0x40 && !calibrateState) { //移动且没有校准
////            buzzerOn(10, 1, 40);
//            caliStableTime = 0;
//            for(uint8_t i=0; i<3; i++)
//                gyroCaliData[i] = 0;
//        }else if(!calibrateState){ //静止且没有校准
//            caliStableTime += IMU_TASK_MS;
//            if(caliStableTime >= 1000){ //改动处
//                for(uint8_t i=0; i<3; i++)
//                    gyroCaliData[i] += imu_data.gyro[i];
//            }
//        }
//        if(!calibrateState && caliStableTime >= 3000){
//            calibrateState = 1; //校准完成
//            for(uint8_t i=0; i<3; i++)
//                imu_data.gyro_offset[i] -= gyroCaliData[i] / (caliStableTime / (double)IMU_TASK_MS);
////            buzzerOn(40, 1, 30);
//        }
//            
//        /**********角度计算**********/
//        static uint8_t inited;
//        if(!inited) { //如果没有初始化则先初始化
//            inited = 1;
//            AHRS_init(quat, imu_.accel, imu_.mag);
//            accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = imu_.accel[0];
//            accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = imu_.accel[1];
//            accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = imu_.accel[2];
//        }
//        else {
//            accel_fliter_1[0] = accel_fliter_2[0];
//            accel_fliter_2[0] = accel_fliter_3[0]; //加速度计低通滤波
//            accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + imu_.accel[0] * fliter_num[2];

//            accel_fliter_1[1] = accel_fliter_2[1];
//            accel_fliter_2[1] = accel_fliter_3[1]; //加速度计低通滤波
//            accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + imu_.accel[1] * fliter_num[2];

//            accel_fliter_1[2] = accel_fliter_2[2];
//            accel_fliter_2[2] = accel_fliter_3[2]; //加速度计低通滤波
//            accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + imu_.accel[2] * fliter_num[2];
//            
//            #ifdef USING_DJIAHRS_ALG
//            AHRS_update(quat, (IMU_TASK_MS/1000.0f), imu_.gyro, accel_fliter_3, imu_.mag);
//            #elif defined(USING_AHRS_ALGO)
//            imu_ahrsCalculate(quat, (IMU_TASK_MS/1000.0f), imu_.gyro, accel_fliter_3, imu_.mag);
//            #elif defined(USING_AHRS_ALGO_IMU_ONLY)
//            imu_ahrsCalculate_IMUonly(quat, (IMU_TASK_MS/1000.0f), imu_.gyro, accel_fliter_3);
//            #endif
//        }
//        fp32 tPitch, tRoll, tYaw;
//        #ifdef USING_DJIAHRS_ALG
//        get_angle(quat, &tPitch, &tRoll, &tYaw); //根据四元数计算欧拉角 输出弧度制
//        #else
//        imu_EulerAngleUpdate(quat, &tPitch, &tRoll, &tYaw); //根据四元数计算欧拉角 输出弧度制
//        #endif  
////        imu_.pit = tPitch * 57.2957f;
////        imu_.rol = tRoll * 57.2957f;
////        imu_.yaw = tYaw * 57.2957f; //注释与否与实际安装方向有关
//        imu_.pit = tYaw * 57.2957f; 
//        imu_.rol = tRoll * 57.2957f;
//        imu_.yaw = tPitch * 57.2957f; //注释与否与实际安装方向有关
//        debugPitch = imu_.pit;
//        debugRoll = imu_.rol;
//        debugYaw = imu_.yaw;
//        debugVx = imu_.gyro[0] * 100;
//        debugVy = imu_.gyro[1] * 100;
//        debugVz = imu_.gyro[2] * 100;
//        
//        /**********准备任务切换**********/
//        if(robotInf.modeStep == (robot_init_step)(ROBOT_INIT_IMU)){
//            static uint16_t initStableTime, targetInitStableTime = 0;
//            if(inited) initStableTime += IMU_TASK_MS; //等待数据稳定
//            #ifdef USING_DJIAHRS_ALG 
//            targetInitStableTime = 0;
//            #elif defined(USING_AHRS_ALGO)
//            targetInitStableTime = 3000;
//            #elif defined(USING_AHRS_ALGO_IMU_ONLY)
//            targetInitStableTime = 1000;
//            #endif
//            if(initStableTime >= targetInitStableTime) robotInf.modeStep = (robot_init_step)(ROBOT_INIT_IMU + 1); //IMU初始化完成 继续机器人的其它功能初始化
//        }
//        
//        vTaskDelayUntil(&currentTime, IMU_TASK_MS);
//    }
//}



