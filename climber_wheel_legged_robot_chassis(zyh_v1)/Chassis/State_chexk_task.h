#ifndef STATE_CHECK_H
#define STATE_CHECK_H
#include "robot_param_balanced_infantry.h"
#include "stdbool.h"
#include "struct_typedef.h"

// 定义起立状态
typedef enum {
	  STATE_NORMAL,
    STATE_stretchleg,        // 伸腿态
    STATE_Backleg,       // 后甩腿
    STATE_COMPLETE   // 左腿完成状态
} State;

// 失控检测状态
typedef enum {
    LOSS_CONTROL_NORMAL,     // 正常
    LOSS_CONTROL_DETECTED,   // 检测到失控
    LOSS_CONTROL_CONFIRMED,  // 确认失控
    LOSS_CONTROL_OVERTURN    // 翻车状态
} LossControlState;



// 失控检测参数
typedef struct {
    float max_phi_error;      // 最大phi位置误差阈值
    float max_theta_error;    // 最大theta角度误差阈值
    float overturn_phi_threshold; // 翻车phi角度阈值
    uint32_t detect_count;    // 检测计数
    uint32_t confirm_threshold; // 确认阈值
    LossControlState state;   // 当前状态
} LossControlDetector;

#endif
