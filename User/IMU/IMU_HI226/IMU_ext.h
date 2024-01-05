#include "IMU_onBoard.h"
#include <stdint.h>
#include <string.h>



#define MAXRAWLEN       (512)       /* max raw frame long */
#define MAX_NODE_SIZE   (16)        /* max support node count */

/* data items */
#define kItemID                    (0x90)
#define kItemAccRaw                (0xA0)
#define kItemGyrRaw                (0xB0)
#define kItemMagRaw                (0xC0)
#define kItemRotationEul           (0xD0)
#define kItemRotationQuat          (0xD1)
#define kItemPressure              (0xF0)
#define KItemIMUSOL                (0x91)
#define KItemGWSOL                 (0x62)


typedef struct
{
    uint32_t id;            /* user defined ID       */
    float acc[3];           /* acceleration          */
    float gyr[3];           /* angular velocity      */  
    float mag[3];           /* magnetic field        */
    float eul[3];           /* attitude: eular angle */
    float quat[4];          /* attitude: quaternion  */
    float pressure;         /* air pressure          */
    uint32_t timestamp;
}ch_imu_data_t;

typedef struct
{
    int nbyte;                          /* number of bytes in message buffer */ 
    int len;                            /* message length (bytes) */
    uint8_t buf[MAXRAWLEN];             /* message raw buffer */
    uint8_t gwid;                       /* network ID(HI222) */
    uint8_t nimu;                       /* # of imu (HI222) */
    
    ch_imu_data_t imu[MAX_NODE_SIZE];   /* imu data list, if (HI226/HI229/CH100/CH110, use imu[0]) */
    uint8_t item_code[8];               /* item code recv in one frame */
    uint8_t nitem_code;                 /* # of item code */
}raw_t;




int ch_serial_input(raw_t *raw, uint8_t data);


void ch_dump_imu_data(raw_t *raw);

void datatrans(raw_t *raw,IMUTypedef *imuu_);
