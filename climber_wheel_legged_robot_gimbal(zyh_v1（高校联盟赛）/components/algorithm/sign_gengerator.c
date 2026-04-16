/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       signal_generator.c/h
  * @brief      信号发生器，用于生成测试的各种信号.
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-10-2024     Penguin         1. 添加
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "sign_gengerator.h"

#include "math.h"
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include <stdbool.h> // 引入布尔类型支持

/**
 * @brief          正弦信号发生器
 * @param[in]      amplitude 振幅
 * @param[in]      offset 输出偏移量
 * @param[in]      period (s)周期
 * @return         SinWave
 */
float GenerateSinWave(float amplitude, float offset, float period)
{
    float t = HAL_GetTick() / 1000.0f;
    return amplitude * sin((2 * M_PI / period) * t) + offset;
}

/**
 * @brief          阶跃信号发生器
 * @param[in]      a0 初始值
 * @param[in]      a1 最终值
 * @param[in]      t0 (s)起始时间
 * @return         StepWave
 */
float GenerateStepWave(float a0, float a1, float t0)
{
    float t = HAL_GetTick() / 1000.0f;
    return (t >= t0) ? a1 : a0;
}

// ... existing code ...
/**
 * @brief          生成斜坡波信号
 * @param[in]      a0 信号的初始幅度
 * @param[in]      a1 信号的最终幅度
 * @param[in]      duration 信号的变化周期 (单位：秒)
 * @return         当前时间下的信号幅度
 */
//float GenerateRampWave(float a0, float a1, float duration)
//{
//    static float start_time = 0;
//    static bool ramp_started = false;
//	

//    float t = HAL_GetTick() / 1000.0f;
//    
//    if (!ramp_started) {
//        start_time = t;
//        ramp_started = true;
//    }

//    float elapsed_time = t - start_time;
//    
//    if (elapsed_time >= duration || duration <= 0) {
//        return a1;  // 已经到达或超过持续时间，返回最终值
//    } else {
//        // 在时间段内线性插值
//        return a0 + (a1 - a0) * (elapsed_time / duration);
//    }
//}
// ... existing code ...

/**
 * @brief          生成脉冲波形信号
 * @param[in]      a1 脉冲波形的幅值1
 * @param[in]      a2 脉冲波形的幅值2
 * @param[in]      t1 (s)脉冲幅值1的持续时间
 * @param[in]      t2 (s)脉冲幅值2的持续时间
 * @return         当前时间在脉冲波形时间范围内时返回幅值1，否则返回幅值0
 */
float GeneratePulseWave(float a1, float a2, float t1, float t2)
{
    uint32_t t = HAL_GetTick();
    uint32_t t1_ = t1 * 1000;
    uint32_t t2_ = t2 * 1000;
    t = t % (t1_ + t2_);
    return (t < t1_) ? a1 : a2;
}

/**
 * @brief          生成锯齿波形
 * @param[in]      a0 波形起始值
 * @param[in]      a1 波形结束值
 * @param[in]      T (s)波形周期
 * @return         生成的锯齿波形值
 */
float GenerateSawtoothWave(float a0, float a1, float T)
{
    float t = HAL_GetTick() / 1000.0f;
    return a0 + (a1 - a0) * (t / T - floor(t / T));
}

/**
 * @brief          方波信号发生器（可调占空比）
 * @param[in]      amplitude 振幅
 * @param[in]      offset 输出偏移量
 * @param[in]      period (s)周期
 * @param[in]      duty_cycle 占空比 (0.0 ~ 1.0)
 * @return         方波信号值
 */
float GenerateSquareWaveEx(float amplitude, float offset, float period, float duty_cycle)
{
    float t = HAL_GetTick() / 1000.0f;
    float phase = fmod(t, period) / period;  // 归一化相位 [0,1)
    
    if (phase < duty_cycle) {
        return amplitude + offset;      // 高电平
    } else {
        return -amplitude + offset;     // 低电平
    }
}

/************************ END OF FILE ************************/
