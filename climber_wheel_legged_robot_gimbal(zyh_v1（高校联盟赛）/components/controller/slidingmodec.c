#include <math.h>
#include <stdint.h>
#include "slidingmodec.h"

// 饱和函数实现（防止抖振）
float sat_function(float s) {
    // 简单的符号函数，可根据需要改为线性饱和区
    return (s > 0.0f) ? 1.0f : ((s < 0.0f) ? -1.0f : 0.0f);
}
// 控制器初始化函数
void SMC_Init(SMC* smc, float C, float epsilon, float error_eps,float K, float J, float u_max) {
    smc->C = C;
    smc->epsilon = epsilon;
    smc->error_eps = error_eps;
    smc->K = K;
    smc->J = J;
    smc->u_max = u_max;
    
    // 状态变量清零
    smc->ref = 0.0f;
    smc->refl = 0.0f;
    smc->dref = 0.0f;
    smc->ddref = 0.0f;
    smc->angle = 0.0f;
    smc->ang_vel = 0.0f;
    smc->error = 0.0f;
    smc->s = 0.0f;
    smc->u = 0.0f;
    
    // 绑定饱和函数
    smc->sat = sat_function;
}
													 // 核心控制循环函数
void SMC_Tick(SMC* smc, float angle_now, float angle_vel) {
    // 读取参数
    smc->angle = angle_now;
    smc->ang_vel = angle_vel;
    smc->error = smc->angle - smc->ref;
    
    // 离散微分计算参考速度/加速度
    smc->ddref = (smc->ref - smc->refl) - smc->dref; // 参考角加速度
    smc->dref = (smc->ref - smc->refl);              // 参考角速度
    
    // 误差下限处理（死区）
    if (fabsf(smc->error) < smc->error_eps) {
        smc->u = 0.0f;
        return;
    }
 
    // 计算滑模面 s = C*error + (ang_vel - dref)
    smc->s = smc->C * smc->error + (smc->ang_vel - smc->dref);
    
    // 计算控制力矩 核心公式
    smc->u = smc->J * (smc->ddref - smc->C * (smc->ang_vel - smc->dref) - smc->epsilon * smc->sat(smc->s) - smc->K * smc->s);
    
    // 控制量限幅
    if (smc->u > smc->u_max) smc->u = smc->u_max;
    if (smc->u < -smc->u_max) smc->u = -smc->u_max;
    
    // 参数更新
    smc->refl = smc->ref;
}
