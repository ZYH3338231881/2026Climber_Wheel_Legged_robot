#include "user_lib.h"

#include "arm_math.h"

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
  * @brief          斜波函数初始化
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      最大值
  * @param[in]      最小值
  * @retval         返回空
  */
void ramp_init(ramp_function_source_t * ramp_source_type, fp32 frame_period, fp32 max, fp32 min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
  * @brief          斜波函数计算，根据输入的值进行叠加， 输入单位为 /s 即一秒后增加输入的值
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      输入值
  * @param[in]      滤波参数
  * @retval         返回空
  */
void ramp_calc(ramp_function_source_t * ramp_source_type, fp32 input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value) {
        ramp_source_type->out = ramp_source_type->max_value;
    } else if (ramp_source_type->out < ramp_source_type->min_value) {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}
/**
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  */
void first_order_filter_init(
    first_order_filter_type_t * first_order_filter_type, fp32 frame_period, const fp32 num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
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
void first_order_filter_cali(first_order_filter_type_t * first_order_filter_type, fp32 input)
{
    // clang-format off
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
    // clang-format on
}

//绝对限制
void abs_limit(fp32 * num, fp32 Limit)
{
    if (*num > Limit) {
        *num = Limit;
    } else if (*num < -Limit) {
        *num = -Limit;
    }
}

//判断符号位
fp32 sign(fp32 value)
{
    if (value >= 0.0f) {
        return 1.0f;
    } else {
        return -1.0f;
    }
}

//浮点死区
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < maxValue && Value > minValue) {
        Value = 0.0f;
    }
    return Value;
}

//int16死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue) {
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
    if (maxValue < minValue) {
        return Input;
    }

    if (Input > maxValue) {
        fp32 len = maxValue - minValue;
        while (Input > maxValue) {
            Input -= len;
        }
    } else if (Input < minValue) {
        fp32 len = maxValue - minValue;
        while (Input < minValue) {
            Input += len;
        }
    }
    return Input;
}

//弧度格式化为-PI~PI
fp32 theta_format(fp32 Ang) { return loop_fp32_constrain(Ang, -PI, PI); }

/**
 * @brief     角度在极坐标系中的转换
 * @param[in] angle  (rad)当前角度
 * @param[in] dangle (rad)旋转角度
 * @param[in] direction 方向，1为正，-1为负
 * @param[in] duration 周期（1个周期-PI\\~PI, 2个周期-2*PI\\~2*PI, ...）
 * @return    转换后的角度
 */
fp32 theta_transform(fp32 angle, fp32 dangle, int8_t direction, uint8_t duration)
{
    return loop_fp32_constrain((angle + dangle) * direction, -3.1415925 * duration, 3.1415925 * duration);
}

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief 低通滤波器初始化
 * @param filter 滤波器结构体
 * @param alpha  平滑系数 [0,1] 0为不对输入数据滤波
 */
void LowPassFilterInit(LowPassFilter_t * filter, float alpha)
{
    filter->alpha = alpha;
    filter->out = 0.0f;
}

/**
 * @brief 低通滤波计算
 * @param filter 滤波器结构体
 * @param input  输入
 * @return 
 */
float LowPassFilterCalc(LowPassFilter_t * filter, float input)
{
    float output = (1.0f - filter->alpha) * input + filter->alpha * filter->out;
    filter->out = output;

    return output;
}

/**
 * @brief 角度范围限制，在-PI~PI之间对角度进行限制，如max < min能够过圈限幅
 * @param theta 角度
 * @param max 最大值
 * @param min 最小值
 * @param return_value 返回值 0-min 1-max ，用于判断在max<min时的返回值
 * @return 范围内的角度
 */
float ThetaRangeLimit(float theta, float max, float min, uint8_t return_value)
{
    if (max > min) {  // 正常范围，无需考虑过圈限幅
        if (theta > max)
            return max;
        else if (theta < min)
            return min;
    } else {  // 考虑过圈限幅
        float max_2pi = max + 2 * PI;
        float theta_2pi = theta + 2 * PI;

        if (theta < min && theta_2pi > max_2pi) {
            if (return_value)
                return max;
            else
                return min;
        }
    }
    return theta;
}

// 添加角度归一化函数，将角度限制在 [-π, π] 范围内
float normalize_angle(float angle)
{
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

// 计算两个角度之间的最短弧度差
float angle_difference(float target, float current)
{
    float diff = target - current;
    return normalize_angle(diff);
}


