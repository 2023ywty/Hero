#ifndef USER_LIB_H
#define USER_LIB_H
#include "main.h"

typedef __packed struct
{
    fp32 input;        //输入数据
    fp32 out;          //滤波输出的数据
    fp32 num[1];       //滤波参数
    fp32 frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;

typedef struct  
{
  int delay_cnt;//计算相邻两帧目标不变持续时间,用来判断速度是否为0
  int freq;
  int last_time;//上次受到目标角度的时间
  float last_position;//上个目标角度
  float speed;//速度
  float last_speed;//上次速度
  float processed_speed;//速度计算结果
}speed_calc_data_t;

//快速开方
extern fp32 invSqrt(fp32 num);
//根据角度解算目标速度
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position);
//斜坡函数,使目标输出值缓慢等于指针输入值
float RampInc_float( float *buffer, float now, float ramp);
//斜坡函数,使目标输出值缓慢等于期望值
float RAMP_float( float final, float now, float ramp);

//一阶滤波初始化
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num);
//一阶滤波计算
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
//绝对限制
extern void abs_limit(fp32 *num, fp32 Limit);
//判断符号位
extern fp32 sign(fp32 value);
//浮点死区
extern fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);
//int26死区
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//限幅函数
extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
//限幅函数
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//循环限幅函数
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
//角度 °限幅 180 ~ -180
extern fp32 theta_format(fp32 Ang);
float loop_ramp_float(float final, float now, float ramp, float minVal, float maxVal);
float f_abs(float num);
int16_t s16_abs(int16_t value);
//弧度格式化为-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

void AHRS_get_latitude(fp32 *latitude);
void AHRS_get_height(fp32 *high);
fp32 AHRS_invSqrt(fp32 num);
fp32 AHRS_sinf(fp32 angle);
fp32 AHRS_cosf(fp32 angle);
fp32 AHRS_tanf(fp32 angle);
fp32 AHRS_asinf(fp32 sin);
fp32 AHRS_acosf(fp32 cos);
fp32 AHRS_atan2f(fp32 y, fp32 x);

#endif
