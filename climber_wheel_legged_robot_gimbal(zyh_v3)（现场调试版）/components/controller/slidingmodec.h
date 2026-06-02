#ifndef SLIDING_MODE_C_H
#define SLIDING_MODE_C_H
/*
  author   zyh  2026.1.23  滑膜控制器设置
	
	目前用于轮腿yaw轴电机6020的控制 --电机电流环控制 注意上位机参数修改 kp ki 注意必须是较新的
*/


// SMC控制器结构体
typedef struct {
    // 控制参数（需手动配置）
    float C;          // 滑模面系数
    float epsilon;    // 鲁棒项增益
    float error_eps;  // 误差死区阈值
    float K;          // 滑模面反馈增益
    float J;          // 转动惯量
    float u_max;      // 最大输出力矩
    
    // 状态变量（内部使用）
    float ref;        // 当前参考角度
    float refl;       // 上一周期参考角度
    float dref;       // 参考角速度
    float ddref;      // 参考角加速度
    
    float angle;      // 当前角度
    float ang_vel;    // 当前角速度
    float error;      // 角度误差
    float s;          // 滑模面
    float u;          // 输出力矩
    
    // 饱和函数
    float (*sat)(float);
} SMC;
// 饱和函数实现（防止抖振）
float sat_function(float s);
// 控制器初始化函数
void SMC_Init(SMC* smc, float C, float epsilon, float error_eps, float K, float J, float u_max);
// 核心控制循环函数
void SMC_Tick(SMC* smc, float angle_now, float angle_vel);
#endif // SLIDING_MODE_C_H