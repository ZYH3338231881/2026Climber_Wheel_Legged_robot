/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "struct_typedef.h"
enum PID_MODE { PID_POSITION = 0, PID_DELTA };

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 N; // [0,1]滤波器系数, 0为不对输入数据滤波

    fp32 max_out;   //最大输出
    fp32 max_iout;  //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];   //微分项 0最新 1上一次 2上上次
    fp32 error[3];  //误差项 0最新 1上一次 2上上次

} pid_type_def;

typedef struct Pid_t
{
    // PID 参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;
    fp32 N;  // 滤波器系数

    // 饱和限制
    fp32 max_out;   //最大输出
    fp32 max_iout;  //最大积分输出

    fp32 ref;  // 设定值
    fp32 fdb;  // 反馈值

    // PID 输出
    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
} Pid_t;

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
extern void PID_init(
    pid_type_def * pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
extern fp32 PID_calc(pid_type_def * pid, fp32 ref, fp32 set);

/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
extern void PID_clear(pid_type_def * pid);

#endif
/*------------------------------ End of File ------------------------------*/
