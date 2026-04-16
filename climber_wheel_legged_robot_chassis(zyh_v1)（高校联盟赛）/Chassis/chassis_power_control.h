#ifndef CHASSIS_POWER_CONTOR_H
#define CHASSIS_POWER_CONTOR_H

#include "struct_typedef.h"

/**
 * @brief 平衡底盘功率控制计算
 * @param[in]  I_bal_L     左轮平衡电流 (Current LSB)
 * @param[in]  I_mov_L     左轮移动电流 (Current LSB, 含速度和转向)
 * @param[in]  I_bal_R     右轮平衡电流
 * @param[in]  I_mov_R     右轮移动电流
 * @param[in]  speed_L_rpm 左轮转速 (RPM)
 * @param[in]  speed_R_rpm 右轮转速 (RPM)
 * @param[out] I_mov_scale 移动分量的缩放系数 (0.0f ~ 1.0f)
 */
void Chassis_Power_Limit_Calc(float I_bal_L, float I_mov_L, 
                              float I_bal_R, float I_mov_R,
                              float speed_L_rpm, float speed_R_rpm,
                              float *I_mov_scale);

#endif