/**
 ******************************************************************************
 * @file    IMU_solve.c/h
 * @brief   6轴陀螺仪解算部分，通过EKF算法滤除噪声，得到姿态角。
 *          解算得到 姿态角、角速度、加速度
 * @note    仅在IMU解算任务中使用，不对其他模块外提供接口
 *          禁止在其他模块中导入该文件
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     2025-04-05      Penguin         1. done
 ******************************************************************************
 * @attention
 * 1st order LPF transfer function:
 *     1
 *  ———————
 *  as + 1
 ******************************************************************************
 @verbatim
 ==============================================================================
 详细原理参考王工的文章：
    四元数EKF姿态更新算法 https://zhuanlan.zhihu.com/p/454155643

 ==============================================================================
 @endverbatim
*/

#ifndef __IMU_SOLVE_H
#define __IMU_SOLVE_H

#include "kalman_filter.h"

typedef struct
{
    KalmanFilter_t IMU_QuaternionEKF; // 卡尔曼滤波器结构体
    uint8_t ConvergeFlag;
    float q[4];        // 四元数估计值
    float GyroBias[3]; // 陀螺仪零偏估计值
    float angle[3]; // 欧拉角估计值

    float Q1; // 四元数更新过程噪声
    float Q2; // 陀螺仪零偏过程噪声
    float R;  // 加速度计量测噪声

    float dt;                     // 姿态更新周期
    float ChiSquare;              // 卡方检验检测函数值
    float ChiSquareTestThreshold; // 卡方检验阈值
    float lambda;                 // 渐消因子
} INS_t;

extern float gVec[3];
extern INS_t INS;


void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda);
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void gEstimateKF_Init(float process_noise, float measure_noise);
void gEstimateKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

extern float GetEkfAngle(int i);

extern float GetEkfAccel(int i);

#endif /* __IMU_SOLVE_H */
/*------------------------------ End of File ------------------------------*/