#include "bsp_buzzer.h"
#include "cmsis_os.h"

// 全局静态变量（替代class）
static TIM_HandleTypeDef *buzzer_htim = NULL;
static uint32_t buzzer_channel = 0;
static float buzzer_freq = 4000.0f;
static float buzzer_loudness = 0.0f;



/**  
  * @brief  蜂鸣器初始化
  * @param  Frequency: 频率
  * @param  Loudness: 响度 0~1
  */
void BSP_Buzzer_Init(float Frequency, float Loudness)
{
    buzzer_htim = &htim12;
    buzzer_channel = TIM_CHANNEL_2;

    if(Frequency < 0) Frequency = 0;
    if(Frequency > 20000) Frequency = 20000;
    if(Loudness < 0) Loudness = 0;
    if(Loudness > 1) Loudness = 1;

    buzzer_freq = Frequency;
    buzzer_loudness = Loudness;

    __HAL_TIM_DISABLE(buzzer_htim);
    __HAL_TIM_SET_COUNTER(buzzer_htim, 0);

    float arr;
    if(buzzer_freq <= 0)
    {
        arr = 239;
        __HAL_TIM_SetAutoreload(buzzer_htim, (uint32_t)arr);
        __HAL_TIM_SetCompare(buzzer_htim, buzzer_channel, 0);
    }
    else
    {
        arr = (1000000.0f / buzzer_freq) - 1.0f;
        float ccr = (arr + 1.0f) * buzzer_loudness * 0.5f;
        __HAL_TIM_SetAutoreload(buzzer_htim, (uint32_t)arr);
        __HAL_TIM_SetCompare(buzzer_htim, buzzer_channel, (uint32_t)ccr);
    }

    HAL_TIM_GenerateEvent(buzzer_htim, TIM_EVENTSOURCE_UPDATE);
    HAL_TIM_PWM_Start(buzzer_htim, buzzer_channel);
    __HAL_TIM_ENABLE(buzzer_htim);
}

/**
  * @brief  设置声音
  */
void BSP_Buzzer_SetSound(float Frequency, float Loudness)
{
    if(Frequency < 0) Frequency = 0;
    if(Frequency > 20000) Frequency = 20000;
    if(Loudness < 0) Loudness = 0;
    if(Loudness > 1) Loudness = 1;

    buzzer_freq = Frequency;
    buzzer_loudness = Loudness;

    __HAL_TIM_DISABLE(buzzer_htim);
    __HAL_TIM_SET_COUNTER(buzzer_htim, 0);

    float arr;
    if(buzzer_freq <= 0 || buzzer_loudness <= 0)
    {
        arr = 239;
        __HAL_TIM_SetAutoreload(buzzer_htim, (uint32_t)arr);
        __HAL_TIM_SetCompare(buzzer_htim, buzzer_channel, 0);
    }
    else
    {
        arr = (1000000.0f / buzzer_freq) - 1.0f;
        float ccr = (arr + 1.0f) * buzzer_loudness * 0.5f;
        __HAL_TIM_SetAutoreload(buzzer_htim, (uint32_t)arr);
        __HAL_TIM_SetCompare(buzzer_htim, buzzer_channel, (uint32_t)ccr);
    }

    HAL_TIM_GenerateEvent(buzzer_htim, TIM_EVENTSOURCE_UPDATE);
    __HAL_TIM_ENABLE(buzzer_htim);
}

/**
  * @brief  播放一个音符
  */
void BSP_Buzzer_PlayNote(float Frequency, uint32_t Duration_ms, float Loudness, uint32_t Gap_ms)
{
    BSP_Buzzer_SetSound(Frequency, Loudness);
    osDelay(Duration_ms);

    BSP_Buzzer_SetSound(0, 0);
    if(Gap_ms > 0)
    {
        osDelay(Gap_ms);
    }
}