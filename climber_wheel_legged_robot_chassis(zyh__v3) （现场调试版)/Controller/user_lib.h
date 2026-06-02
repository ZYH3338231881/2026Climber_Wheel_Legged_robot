#ifndef USER_LIB_H
#define USER_LIB_H
#include "attribute_typedef.h"
#include "struct_typedef.h"

typedef struct
{
    fp32 input;         //输入数据
    fp32 out;           //输出数据
    fp32 min_value;     //限幅最小值
    fp32 max_value;     //限幅最大值
    fp32 frame_period;  //时间间隔
} __packed__ ramp_function_source_t;

typedef struct
{
    fp32 input;         //输入数据
    fp32 out;           //滤波输出的数据
    fp32 num[1];        //滤波参数
    fp32 frame_period;  //滤波的时间间隔 单位 s
} __packed__ first_order_filter_type_t;

// 定义一阶低通滤波器结构体
typedef struct LowPassFilter
{
    float alpha;  // 平滑系数
    float out;    // 输出
} LowPassFilter_t;

//快速开方
extern fp32 invSqrt(fp32 num);

void ramp_init(ramp_function_source_t * ramp_source_type, fp32 frame_period, fp32 max, fp32 min);

void ramp_calc(ramp_function_source_t * ramp_source_type, fp32 input);

extern void first_order_filter_init(
    first_order_filter_type_t * first_order_filter_type, fp32 frame_period, const fp32 num[1]);

extern void first_order_filter_cali(
    first_order_filter_type_t * first_order_filter_type, fp32 input);

extern void abs_limit(fp32 * num, fp32 Limit);

extern fp32 sign(fp32 value);

extern fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);

extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);

extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);

extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);

extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);

extern fp32 theta_format(fp32 Ang);

extern fp32 theta_transform(fp32 angle, fp32 dangle, int8_t direction, uint8_t duration);

extern int float_to_uint(float x_float, float x_min, float x_max, int bits);

extern float uint_to_float(int x_int, float x_min, float x_max, int bits);

extern void LowPassFilterInit(LowPassFilter_t * filter, float alpha);

extern float LowPassFilterCalc(LowPassFilter_t * filter, float input);

extern float ThetaRangeLimit(float theta, float max, float min, uint8_t return_value);
extern float normalize_angle(float angle);
extern float angle_difference(float target, float current);
//弧度格式化为-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif

