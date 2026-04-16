#include "chassis_power_control.h"
#include "CAN_receive.h" 
#include "cmsis_os.h" // 用于获取时间戳或延时(如果需要)
#include <math.h>
#include "computer_rec.h"

/* ================= 参数定义区域 ================= */

// 功率模型参数 (根据您的拟合结果)
// P_loss = k1*I^2 + k2*w^2 + C
static const float k1 = 1.23e-07f;       
static const float k2 = 1.453e-07f;      
static const float constant = 1.081f;    // 单个电机的静态损耗(含电调/电路板)

// 转子力矩系数 (用于 P_mech 计算，无需修改)
static const float toque_coeff = 1.99688994e-6f; 

// 安全系数：将预测功率放大 1.1 倍
// 作用：给物理世界的未知损耗留出 10% 的余量，防止裁判系统检测值比我们算的大
#define POWER_SAFETY_FACTOR  1.1f 

// 输出平滑参数：每次循环 scale 恢复的最大步长
// 假设控制频率 1kHz，0.005 代表 200ms 才能从 0 恢复到 1，非常平滑
#define SCALE_RISE_RATE      0.005f


// 定义滤波系数 (0.0 ~ 1.0)
// 值越小：滤波越强，数据越平滑，但在急加速时延迟越大
// 值越大：数据越接近原始值，噪声越大
// 建议 0.1 ~ 0.2 之间
#define POWER_SPEED_LPF_ALPHA  0.15f



float p_bal_L;
float p_bal_R;
float p_base_total;

float p_full_L;
float p_full_R;
float p_full_total;


float actual_p_L ;
float actual_p_R ;
float p_actual_total;

extern JudgementDataTypedef JudgementData;
/* ================= 函数实现 ================= */

/**
 * @brief 单个电机功率预测模型
 * @param I_cmd     电调发送的电流值 (LSB: -16384 ~ 16384)
 * @param speed_rpm 电机转子转速 (rpm)
 * @return float    预测功率 (W)
 */
static float Predict_Motor_Power(float I_cmd, float speed_rpm)
{
    // 1. 机械功率 P = T * w
    // 使用 fabsf 取绝对值，执行"最保守策略"：
    // 无论电机是加速还是制动(发电)，都认为它在满额消耗功率。
    // 这避免了因回充效率估算不准导致的超功率。
    float p_mech = toque_coeff * fabsf(I_cmd * speed_rpm);
    
    // 2. 焦耳热损耗 P = I^2 * R ...
    // k1 * I^2 + k2 * w^2 + C
    float p_loss = k1 * I_cmd * I_cmd + k2 * speed_rpm * speed_rpm + constant;
    
    // 3. 总预测值 + 安全系数
    float p_total = (p_mech + p_loss) * POWER_SAFETY_FACTOR;
    
    // 兜底：功率不能为负
    if (p_total < 0.0f) return 0.0f;
    
    return p_total;
}

/**
 * @brief 底盘功率限制核心计算函数
 * @param I_bal_L      左轮平衡分量电流 (LSB)
 * @param I_mov_L      左轮移动分量电流 (LSB)
 * @param I_bal_R      右轮平衡分量电流 (LSB)
 * @param I_mov_R      右轮移动分量电流 (LSB)
 * @param speed_L_rpm  左轮转速 (RPM, 建议输入滤波后的值)
 * @param speed_R_rpm  右轮转速 (RPM, 建议输入滤波后的值)
 * @param I_mov_scale  [输出] 移动分量缩放系数 (0.0 ~ 1.0)
 */
// 静态变量用于保存上一帧的 Scale，实现输出滤波
static float last_I_mov_scale = 1.0f;
float max_power = 90;
void Chassis_Power_Limit_Calc(float I_bal_L, float I_mov_L, 
                              float I_bal_R, float I_mov_R,
                              float speed_L_rpm, float speed_R_rpm,
                              float *I_mov_scale)
{
	
	
	// ============================================================
    // [新增] 手动速度滤波 (Low Pass Filter)
    // ============================================================
    // 使用 static 变量保存上一次的滤波值，左右轮分开
    static float speed_L_filt = 0.0f;
    static float speed_R_filt = 0.0f;
    static uint8_t first_run = 1;

    // 第一次运行时直接赋值，避免从0开始爬升导致初始预测偏小
    if (first_run)
    {
        speed_L_filt = speed_L_rpm;
        speed_R_filt = speed_R_rpm;
        first_run = 0;
    }

    // 滤波公式: New = Old * (1-Alpha) + Raw * Alpha
    speed_L_filt = speed_L_filt * (1.0f - POWER_SPEED_LPF_ALPHA) + speed_L_rpm * POWER_SPEED_LPF_ALPHA;
    speed_R_filt = speed_R_filt * (1.0f - POWER_SPEED_LPF_ALPHA) + speed_R_rpm * POWER_SPEED_LPF_ALPHA;
		
		
		
    // ------------------------------------------------------------
    // 1. 获取裁判系统限制 & 缓冲能量
    // ------------------------------------------------------------
     max_power = 300;
    float buffer_energy = JudgementData.power_heat_data_t.buffer_energy;
    
    // 掉线保护：如果裁判系统没数据，默认限制 90W (步兵标准)
    if (max_power == 0) max_power = 90.0f;

    // ------------------------------------------------------------
    // 2. 动态缓冲能量策略 (线性插值)
    // ------------------------------------------------------------
    float w_buffer = 1.0f;

    if (buffer_energy > 60.0f)
    {
        // 能量充裕 (>60J)：允许超频 30%
        w_buffer = 1.3f; 
    }
    else if (buffer_energy < 10.0f)
    {
        // 能量告急 (<10J)：强制缩减到 50%，优先保电容不掉电
        w_buffer = 1.0f; 
    }
    else
    {
        // 线性区间 (10J ~ 60J)
        // 随着能量降低，系数从 1.0 线性降到 0.5
        // 公式：y = kx + b -> 10J对应0.5, 60J对应1.3(或1.0)
        // 这里采用保守策略：60J时1.0，10J时0.5
        float linear_factor = (buffer_energy - 10.0f) / 50.0f; // 0.0 ~ 1.0
//        w_buffer = 0.5f + (0.5f * linear_factor);
			 w_buffer = 1.0f;
    }

    // 计算最终允许的功率阈值
    float power_limit_final = max_power * w_buffer;

    // ------------------------------------------------------------
    // 3. 预测功率 (核心)
    // ------------------------------------------------------------
    
    // A. 预测 "仅维持平衡" 需要的功率 (P_base)
    // 也就是如果此时切断所有移动指令，车子还要耗多少电
     p_bal_L = Predict_Motor_Power(I_bal_L, speed_L_filt);
     p_bal_R = Predict_Motor_Power(I_bal_R, speed_R_filt);
     p_base_total = p_bal_L + p_bal_R;

    // B. 预测 "全速执行" 需要的功率 (P_full)
    // 也就是当前的真实指令会耗多少电
     p_full_L = Predict_Motor_Power(I_bal_L + I_mov_L, speed_L_filt);
     p_full_R = Predict_Motor_Power(I_bal_R + I_mov_R, speed_R_filt);
     p_full_total = p_full_L + p_full_R;
		

    // 临时变量用于计算当前帧的目标 scale
    float calculated_scale = 1.0f;

    // ------------------------------------------------------------
    // 4. 计算缩放系数 (逻辑判断)
    // ------------------------------------------------------------

    // 情况 A: 预测的总功率在限制范围内 -> 安全，满额输出
    if (p_full_total <= power_limit_final)
    {
        calculated_scale = 1.0f;
    }
    // 情况 B: 预测超功率 -> 需要处理
    else
    {
        // [关键逻辑]: 判断是 "加速" 还是 "刹车/顶墙"
        
        // 如果 P_full < P_base：
        // 说明移动电流与平衡电流方向相反，互相抵消。
        // 此时总功率反而比 "啥都不干(P_base)" 要小。
        // 如果我们限制移动电流 (scale -> 0)，抵消效应消失，功率会飙升到 P_base。
        // 所以这种情况下，维持 1.0 才是功率最小、最安全的选择。
        if (p_full_total < p_base_total)
        {
             calculated_scale = 1.0f;
        }
        // 如果 P_full >= P_base：
        // 说明移动电流在增加总负荷（正常加速阶段），需要限制。
        else
        {
            // 严重情况：光是维持平衡就已经超标了
            if (p_base_total >= power_limit_final)
            {
                // 如果电容还有点电(>10J)，保留极小动力(0.0)维持控制回路闭环
                // 如果电容彻底干了，scale=0
                calculated_scale = 0.0f;
            }
            else
            {
                // 正常缩放公式：(Limit - Base) / (Full - Base)
                float power_available = power_limit_final - p_base_total;
                float power_demand = p_full_total - p_base_total;
                
                if (power_demand > 0.0001f)
                {
//                    float linear_scale = power_available / power_demand;
//										calculated_scale = sqrtf(linear_scale);
									
									calculated_scale = power_available / power_demand;
                }
                else
                {
                    calculated_scale = 0.0f;
                }
            }
        }
    }
    
    // ------------------------------------------------------------
    // 5. 安全钳位
    // ------------------------------------------------------------
    if (calculated_scale > 1.0f) calculated_scale = 1.0f;
    if (calculated_scale < 0.0f) calculated_scale = 0.0f;

    // ------------------------------------------------------------
    // 6. 输出非对称滤波 (快降慢升) - 消除震荡的核心
    // ------------------------------------------------------------
    
    if (calculated_scale < last_I_mov_scale)
    {
        // 趋势：功率突增，需要立刻限制
        // 响应：瞬时赋值，不滤波！保命要紧。
        *I_mov_scale = calculated_scale;
    }
    else
    {
        // 趋势：功率回落，限制正在放松
        // 响应：缓慢增加 scale，防止电流瞬间跳变导致再次超功率（死区震荡）
        
        if (calculated_scale > last_I_mov_scale + SCALE_RISE_RATE)
        {
            *I_mov_scale = last_I_mov_scale + SCALE_RISE_RATE;
        }
        else
        {
            *I_mov_scale = calculated_scale;
        }
    }

		 actual_p_L = Predict_Motor_Power(I_bal_L + (*I_mov_scale) * I_mov_L, speed_L_filt);
     actual_p_R = Predict_Motor_Power(I_bal_R + (*I_mov_scale) * I_mov_R, speed_R_filt);
     p_actual_total = actual_p_L + actual_p_R;
		
    // 更新历史值
    last_I_mov_scale = *I_mov_scale;
}