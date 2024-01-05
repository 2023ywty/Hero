/**************************************����������********************************/
/*ע�����ڿ�����ͨ���ӳ�ԭ����ʱ�������������ǵ�yaw��Ƕ�*/

#include "INS_task.h"
#include  "IMU_ext.h"

//#define USING_DJIAHRS_ALG //DJI-AHRS 9���ںϣ�������ʵ��ת���Ƕ���������⣨�Ƽ�ʹ�ã�
//#define USING_AHRS_ALGO //AHRS 9���ںϣ�������ʵ��ת���ǶȲ����������⣨���Ƽ�ʹ�ã�
#define USING_AHRS_ALGO_IMU_ONLY //AHRS 6���ںϣ������˶�����Ư���⣨�����Ժã�

//���ٶȼƵ�ͨ�˲�����
static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
IMUPreDataTypedef imu_data; //ԭʼ���ݺ�ƫ����
IMUTypedef imu_; //�������
IMUTypedef imuu_;
extern raw_t rawa;
static fp32 quat[4]; //��Ԫ��


void INS_task(void *pvParameters){
    portTickType currentTime;
    currentTime = xTaskGetTickCount();
    vTaskDelay(100);
    while(1){

				datatrans(&rawa,&imu_);			 	
			 				
        IMU_Update_Data(&imu_data); //���ݽ���
        IMU_Handle_Data(&imu_data, &imu_); //���ݴ���
        
        /**********�Զ�У׼������**********/
        static uint8_t calibrateState;
        static uint16_t caliStableTime;
        static float gyroCaliData[3];
        if((imu_data.imuStatus & 0x40) == 0x40 && !calibrateState) { //�ƶ���û��У׼
//            buzzerOn(10, 1, 40);
            caliStableTime = 0;
            for(uint8_t i=0; i<3; i++)
                gyroCaliData[i] = 0;
        }else if(!calibrateState){ //��ֹ��û��У׼
            caliStableTime += IMU_TASK_MS;
            if(caliStableTime >= 1000){ //�Ķ���
                for(uint8_t i=0; i<3; i++)
                    gyroCaliData[i] += imu_data.gyro[i];
            }
        }
        if(!calibrateState && caliStableTime >= 3000){
            calibrateState = 1; //У׼���
            for(uint8_t i=0; i<3; i++)
                imu_data.gyro_offset[i] -= gyroCaliData[i] / (caliStableTime / (double)IMU_TASK_MS);
//            buzzerOn(40, 1, 30);
        }
            
        /**********�Ƕȼ���**********/
        static uint8_t inited;
        if(!inited) { //���û�г�ʼ�����ȳ�ʼ��
            inited = 1;
            AHRS_init(quat, imu_.accel, imu_.mag);
            accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = imu_.accel[0];
            accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = imu_.accel[1];
            accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = imu_.accel[2];
        }
        else {
            accel_fliter_1[0] = accel_fliter_2[0];
            accel_fliter_2[0] = accel_fliter_3[0]; //���ٶȼƵ�ͨ�˲�
            accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + imu_.accel[0] * fliter_num[2];

            accel_fliter_1[1] = accel_fliter_2[1];
            accel_fliter_2[1] = accel_fliter_3[1]; //���ٶȼƵ�ͨ�˲�
            accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + imu_.accel[1] * fliter_num[2];

            accel_fliter_1[2] = accel_fliter_2[2];
            accel_fliter_2[2] = accel_fliter_3[2]; //���ٶȼƵ�ͨ�˲�
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
        get_angle(quat, &tPitch, &tRoll, &tYaw); //������Ԫ������ŷ���� ���������
        #else
        imu_EulerAngleUpdate(quat, &tPitch, &tRoll, &tYaw); //������Ԫ������ŷ���� ���������
        #endif  

//        imu_.pit = tYaw * 57.2957f; 
//        imu_.rol = tRoll * 57.2957f;
//				imu_.yaw = imuu_.yaw;

        
        /**********׼�������л�**********/
        if(robotInf.modeStep == (robot_init_step)(ROBOT_INIT_IMU)){
            static uint16_t initStableTime, targetInitStableTime = 0;
            if(1) initStableTime += IMU_TASK_MS; //�ȴ������ȶ�
            #ifdef USING_DJIAHRS_ALG 
            targetInitStableTime = 0;
            #elif defined(USING_AHRS_ALGO)
            targetInitStableTime = 3000;
            #elif defined(USING_AHRS_ALGO_IMU_ONLY)
            targetInitStableTime = 1000;
            #endif
            if(1) robotInf.modeStep = (robot_init_step)(ROBOT_INIT_IMU + 1); //IMU��ʼ����� ���������˵��������ܳ�ʼ��
        }
        
        vTaskDelayUntil(&currentTime, IMU_TASK_MS);
    }
	}

	
	
	
/*********************************************************A�������Ǵ���*********************************************************************/
	
//#include "INS_task.h"

////#define USING_DJIAHRS_ALG //DJI-AHRS 9���ںϣ�������ʵ��ת���Ƕ���������⣨�Ƽ�ʹ�ã�
////#define USING_AHRS_ALGO //AHRS 9���ںϣ�������ʵ��ת���ǶȲ����������⣨���Ƽ�ʹ�ã�
//#define USING_AHRS_ALGO_IMU_ONLY //AHRS 6���ںϣ������˶�����Ư���⣨�����Ժã�

////���ٶȼƵ�ͨ�˲�����
//static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
//static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
//static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
//static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
//IMUPreDataTypedef imu_data; //ԭʼ���ݺ�ƫ����
//IMUTypedef imu_; //�������
//static fp32 quat[4]; //��Ԫ��

//int16_t debugPitch, debugRoll, debugYaw;
//int16_t debugVx, debugVy, debugVz;

//void INS_task(void *pvParameters){
//    portTickType currentTime;
//    currentTime = xTaskGetTickCount();
//    vTaskDelay(100);
//    while(1){
//        IMU_Update_Data(&imu_data); //���ݽ���
//        IMU_Handle_Data(&imu_data, &imu_); //���ݴ���
//        
//        /**********�Զ�У׼������**********/
//        static uint8_t calibrateState;
//        static uint16_t caliStableTime;
//        static float gyroCaliData[3];
//        if((imu_data.imuStatus & 0x40) == 0x40 && !calibrateState) { //�ƶ���û��У׼
////            buzzerOn(10, 1, 40);
//            caliStableTime = 0;
//            for(uint8_t i=0; i<3; i++)
//                gyroCaliData[i] = 0;
//        }else if(!calibrateState){ //��ֹ��û��У׼
//            caliStableTime += IMU_TASK_MS;
//            if(caliStableTime >= 1000){ //�Ķ���
//                for(uint8_t i=0; i<3; i++)
//                    gyroCaliData[i] += imu_data.gyro[i];
//            }
//        }
//        if(!calibrateState && caliStableTime >= 3000){
//            calibrateState = 1; //У׼���
//            for(uint8_t i=0; i<3; i++)
//                imu_data.gyro_offset[i] -= gyroCaliData[i] / (caliStableTime / (double)IMU_TASK_MS);
////            buzzerOn(40, 1, 30);
//        }
//            
//        /**********�Ƕȼ���**********/
//        static uint8_t inited;
//        if(!inited) { //���û�г�ʼ�����ȳ�ʼ��
//            inited = 1;
//            AHRS_init(quat, imu_.accel, imu_.mag);
//            accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = imu_.accel[0];
//            accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = imu_.accel[1];
//            accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = imu_.accel[2];
//        }
//        else {
//            accel_fliter_1[0] = accel_fliter_2[0];
//            accel_fliter_2[0] = accel_fliter_3[0]; //���ٶȼƵ�ͨ�˲�
//            accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + imu_.accel[0] * fliter_num[2];

//            accel_fliter_1[1] = accel_fliter_2[1];
//            accel_fliter_2[1] = accel_fliter_3[1]; //���ٶȼƵ�ͨ�˲�
//            accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + imu_.accel[1] * fliter_num[2];

//            accel_fliter_1[2] = accel_fliter_2[2];
//            accel_fliter_2[2] = accel_fliter_3[2]; //���ٶȼƵ�ͨ�˲�
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
//        get_angle(quat, &tPitch, &tRoll, &tYaw); //������Ԫ������ŷ���� ���������
//        #else
//        imu_EulerAngleUpdate(quat, &tPitch, &tRoll, &tYaw); //������Ԫ������ŷ���� ���������
//        #endif  
////        imu_.pit = tPitch * 57.2957f;
////        imu_.rol = tRoll * 57.2957f;
////        imu_.yaw = tYaw * 57.2957f; //ע�������ʵ�ʰ�װ�����й�
//        imu_.pit = tYaw * 57.2957f; 
//        imu_.rol = tRoll * 57.2957f;
//        imu_.yaw = tPitch * 57.2957f; //ע�������ʵ�ʰ�װ�����й�
//        debugPitch = imu_.pit;
//        debugRoll = imu_.rol;
//        debugYaw = imu_.yaw;
//        debugVx = imu_.gyro[0] * 100;
//        debugVy = imu_.gyro[1] * 100;
//        debugVz = imu_.gyro[2] * 100;
//        
//        /**********׼�������л�**********/
//        if(robotInf.modeStep == (robot_init_step)(ROBOT_INIT_IMU)){
//            static uint16_t initStableTime, targetInitStableTime = 0;
//            if(inited) initStableTime += IMU_TASK_MS; //�ȴ������ȶ�
//            #ifdef USING_DJIAHRS_ALG 
//            targetInitStableTime = 0;
//            #elif defined(USING_AHRS_ALGO)
//            targetInitStableTime = 3000;
//            #elif defined(USING_AHRS_ALGO_IMU_ONLY)
//            targetInitStableTime = 1000;
//            #endif
//            if(initStableTime >= targetInitStableTime) robotInf.modeStep = (robot_init_step)(ROBOT_INIT_IMU + 1); //IMU��ʼ����� ���������˵��������ܳ�ʼ��
//        }
//        
//        vTaskDelayUntil(&currentTime, IMU_TASK_MS);
//    }
//}



