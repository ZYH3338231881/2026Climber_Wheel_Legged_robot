#ifndef _AUTOGIMBAL_H_
#define _AUTOGIMBAL_H_
#include "string.h"
#include "main.h"
#define BUFLENGTH  128 //最大接收数据
#define DATALENGTH 16//有效数据
// 在 AutoGimbal.h 中更新结构体定义
typedef struct
{
    uint8_t real_receive[BUFLENGTH];    // 原始接收数据
    uint8_t mode;                       // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
	  uint8_t last_mode;
    float yaw;                          // Yaw角度（弧度）
    float yaw_vel;                      // Yaw角速度
    float yaw_acc;                      // Yaw角加速度
    float pitch;                        // Pitch角度（弧度）
    float pitch_vel;                    // Pitch角速度
    float pitch_acc;                    // Pitch角加速度
} visionDataStu_t;

void AUTO_control_init(void);

#endif
