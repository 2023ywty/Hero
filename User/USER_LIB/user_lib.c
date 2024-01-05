#include "user_lib.h"
#include "arm_math.h"
#include "main.h"

float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
    S->delay_cnt++;

    if (time != S->last_time)
    {
        S->speed = (position - S->last_position) / (time - S->last_time) * 2;//计算速度

        S->processed_speed = S->speed;

        S->last_time = time;
        S->last_position = position;
        S->last_speed = S->speed;
        S->delay_cnt = 0;
    }

    if(S->delay_cnt > 300/*100*/) // delay 200ms speed = 0
    {
        S->processed_speed = 0;//时间过长则认为速度不变
    }
    return S->processed_speed;//计算出的速度
}


/**
 
  * @attention * @brief  斜坡函数,使目标输出值缓慢等于期望值
  * @param  期望最终输出,当前输出,变化速度(越大越快)，不能应用在360度的电机斜坡中
  * @retval 当前输出
  */
float RAMP_float( float final, float now, float ramp)
{
    float buffer = 0;

    buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)
        {
            now += ramp;
        }
        else
        {
            now += buffer;
        }
    }
    else
    {
        if (buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now += buffer;
        }
    }

    return now;
}

//循环斜坡函数，用于云台期望斜坡，maxVal和minVal分别是云台位置实际值反馈的最大值和最小值
float loop_ramp_float(float final, float now, float ramp, float minVal, float maxVal)
{
    float err = final - now;
    if(err > (maxVal - minVal)/2)
    {
        err -= maxVal - minVal;
    }
    else if(err < -(maxVal - minVal)/2)
        err += maxVal - minVal;

    if(err > ramp)
        return now + ramp;
    else if(err < -ramp)
        return now - ramp;
    else
        return final;
}

float f_abs(float num) {
    if(num < 0)
        num = -num;
    return num;
}
/**
  * @brief  斜坡函数,使目标输出值缓慢等于指针输入值
  * @param  要在当前输出量上累加的值,目标输出量,递增快慢
  * @retval 目标输出量
  * @attention
  *
*/
float RampInc_float( float *buffer, float now, float ramp )
{

    if (*buffer > 0)
    {
        if (*buffer > ramp)
        {
            now     += ramp;
            *buffer -= ramp;
        }
        else
        {
            now     += *buffer;
            *buffer  = 0;
        }
    }
    else
    {
        if (*buffer < -ramp)
        {
            now += -ramp;
            *buffer -= -ramp;
        }
        else
        {
            now     += *buffer;
            *buffer  = 0;
        }
    }

    return now;
}

int16_t s16_abs(int16_t value)
{
    if(value > 0)
        return value;
    else
        return -value;
}

//快速开方
fp32 invSqrt(fp32 num)
{
    fp32 halfnum = 0.5f * num;
    fp32 y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(fp32 *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}


/**
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, fp32 num)
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num;
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{   //低通滤波 out = a*last_out + （1-a）*input
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out +
        first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

//绝对限制
void abs_limit(fp32 *num, fp32 Limit)
{
    if (*num > Limit)
    {
        *num = Limit;
    }
    else if (*num < -Limit)
    {
        *num = -Limit;
    }
}

//判断符号位
fp32 sign(fp32 value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

//浮点死区
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

//int26死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

//限幅函数
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//循环限幅函数
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        fp32 len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        fp32 len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//弧度格式化为-PI~PI

//角度格式化为-180~180
fp32 theta_format(fp32 Ang)
{
    return loop_fp32_constrain(Ang, -180.0f, 180.0f);
}

void AHRS_get_height(fp32 *high)
{
    if (high != NULL)
    {
        *high = 0.0f;
    }
}

/**
  * @brief          用于获取当前纬度
  * @author         RM
  * @param[in]      纬度的指针，fp32
  * @retval         返回空
  */

void AHRS_get_latitude(fp32 *latitude)
{
    if (latitude != NULL)
    {
        *latitude = Latitude_At_ShenZhen;
    }
}

/**
  * @brief          快速开方函数，
  * @author         RM
  * @param[in]      输入需要开方的浮点数，fp32
  * @retval         返回1/sqrt 开方后的倒数
  */

fp32 AHRS_invSqrt(fp32 num)
{
    fp32 halfnum = 0.5f * num;
    fp32 y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(fp32 *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

/**
  * @brief          sin函数
  * @author         RM
  * @param[in]      角度 单位 rad
  * @retval         返回对应角度的sin值
  */

fp32 AHRS_sinf(fp32 angle)
{
    return arm_sin_f32(angle);
}
/**
  * @brief          cos函数
  * @author         RM
  * @param[in]      角度 单位 rad
  * @retval         返回对应角度的cos值
  */

fp32 AHRS_cosf(fp32 angle)
{
    return arm_cos_f32(angle);
}

/**
  * @brief          tan函数
  * @author         RM
  * @param[in]      角度 单位 rad
  * @retval         返回对应角度的tan值
  */

fp32 AHRS_tanf(fp32 angle)
{
    return tanf(angle);
}
/**
  * @brief          用于32位浮点数的反三角函数 asin函数
  * @author         RM
  * @param[in]      输入sin值，最大1.0f，最小-1.0f
  * @retval         返回角度 单位弧度
  */

fp32 AHRS_asinf(fp32 sin)
{

    return asinf(sin);
}

/**
  * @brief          反三角函数acos函数
  * @author         RM
  * @param[in]      输入cos值，最大1.0f，最小-1.0f
  * @retval         返回对应的角度 单位弧度
  */

fp32 AHRS_acosf(fp32 cos)
{

    return acosf(cos);
}

/**
  * @brief          反三角函数atan函数
  * @author         RM
  * @param[in]      输入tan值中的y值 最大正无穷，最小负无穷
  * @param[in]      输入tan值中的x值 最大正无穷，最小负无穷
  * @retval         返回对应的角度 单位弧度
  */

fp32 AHRS_atan2f(fp32 y, fp32 x)
{
    return atan2f(y, x);
}
