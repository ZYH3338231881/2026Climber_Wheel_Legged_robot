#ifndef JUDGE_PAINT_TASK_H
#define JUDGE_PAINT_TASK_H

#include "main.h"

// ==========================================
// 屏幕中心坐标与 UI 缩放基准配置
// ==========================================
#define SCREEN_X_CENTER 960
#define SCREEN_Y_CENTER 540

// 腿部 UI 映射参数 (放大 800 倍，使得 0.4m 的腿长在屏幕上占 320 像素)
#define LEG_UI_SCALE 800.0f  
#define HIP_L_X      400     // 左髋关节固定 X 坐标
#define HIP_L_Y      500     // 左髋关节固定 Y 坐标
#define HIP_R_X      1520    // 右髋关节固定 X 坐标
#define HIP_R_Y      500     // 右髋关节固定 Y 坐标

// 罗盘参数
#define COMPASS_X    960     // 罗盘中心 X (正下方)
#define COMPASS_Y    180     // 罗盘中心 Y
#define COMPASS_R    60      // 罗盘半径

void judge_painting_init(void);
void judge_painting_update(void); 

#endif