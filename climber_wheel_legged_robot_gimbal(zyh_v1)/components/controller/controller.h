#ifndef _CONTROLLER_H
#define _CONTROLLER_H
#include "main.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "user_lib.h"
#include <math.h>
typedef __packed struct
{
    float c[3]; // G(s) = 1/(c2s^2 + c1s + c0)系统传递函数

    float Ref;                //当前参考信号
    float Last_Ref;           //上次当前参考信号

    float DeadBand;           //死区范围

    uint32_t DWT_CNT;         //DWT计数器值，用于精确计时
    float dt;                 //时间步长

    float LPF_RC; // RC = 1/omegac   低通滤波器时间常数 RC = 1/ωc

    float Ref_dot;           //参考信号的一阶导数
    float Ref_ddot;          //参考信号的二阶导数
    float Last_Ref_dot;      //参考信号上次的一阶导数

    uint16_t Ref_dot_OLS_Order;//一阶导数计算的OLS阶数
    Ordinary_Least_Squares_t Ref_dot_OLS;//用于计算一阶导数的OLS结构体
    uint16_t Ref_ddot_OLS_Order;//二阶导数计算的OLS阶数
    Ordinary_Least_Squares_t Ref_ddot_OLS;//用于计算二阶导数的OLS结构体

    float Output;
    float MaxOut;

} Feedforward_t;

#endif