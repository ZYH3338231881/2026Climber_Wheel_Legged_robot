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
#ifndef __SIGNAL_GENERATOR_H
#define __SIGNAL_GENERATOR_H

extern float GenerateSinWave(float amplitude, float offset, float period);

extern float GenerateStepWave(float a0, float a1, float t0);

float GenerateRampWave(float a0, float a1, float duration);

extern float GeneratePulseWave(float a1, float a2, float t1, float t2);

extern float GenerateSawtoothWave(float a0, float a1, float T);
extern float GenerateSquareWaveEx(float amplitude, float offset, float period, float duty_cycle);

#endif  // __SIGNAL_GENERATOR_H
        /************************ END OF FILE ************************/