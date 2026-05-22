/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       shoot_fric.c/h
  * @brief      使用摩擦轮的发射机构控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *  V1.1.0     2025-1-15       CJH             1. 实现基本功能
  *  V2.0.0     2025-3-3        CJH             1. 兼容了达妙4310拨弹盘和大疆2006拨弹盘
  *                                             2. 完善了单发功能，上位机火控功能
  *                                             3. 增加了热量限制
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/
#ifndef SHOOT_FRIC_H
#define SHOOT_FRIC_H
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "shoot.h"
#include "math.h"
#include "user_lib.h"
#include "arm_math.h"
#include "shoot_task.h"
//电机ID
#define TRIGGER_MOTOR_ID 3
#define FRIC_MOTOR_R_ID 2
#define FRIC_MOTOR_L_ID 1

//电机can口
#define TRIGGER_MOTOR_CAN 1
#define FRIC_MOTOR_R_CAN 2
#define FRIC_MOTOR_L_CAN 2

//电机std_id
#define STD_ID 0x200
//单环拨弹速度
//摩擦轮速度
#define FRIC_R_SPEED                  (-600.0f) 
#define FRIC_L_SPEED                  (600.0f) 
#define FRIC_SPEED_LIMIT            (600.0f) 

/*ECD parameters------------*/
//电机反馈码盘值范围
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191

//电机rpm 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED 0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18


//拨弹轮电机PID速度环
#define TRIGGER_SPEED_PID_KP (100.0f)
#define TRIGGER_SPEED_PID_KI (0.5f)
#define TRIGGER_SPEED_PID_KD (0.1f)

#define TRIGGER_SPEED_PID_MAX_OUT (16384.0f)
#define TRIGGER_SPEED_PID_MAX_IOUT (1000.0f)

//拨弹轮电机PID角度环
#define TRIGGER_ANGEL_PID_KP (30.0f)
#define TRIGGER_ANGEL_PID_KI (0.05f)
#define TRIGGER_ANGEL_PID_KD (1.0f)

#define TRIGGER_ANGEL_PID_MAX_OUT (300.0f)
#define TRIGGER_ANGEL_PID_MAX_IOUT (30.0f)

//摩擦轮电机PID
#define FRIC_SPEED_PID_KP (15.0f)
#define FIRC_SPEED_PID_KI (0.6f)
#define FRIC_SPEED_PID_KD (1.0f)

#define FRIC_PID_MAX_OUT (16000.0f)
#define FRIC_PID_MAX_IOUT (1000.0f)

#define SHOOT_HEAT_REMAIN_VALUE     80//89

//电机种类
#define TRIGGER_MOTOR_TYPE ((MotorType_e)DJI_M2006)
#define FRIC_MOTOR_TYPE ((MotorType_e)DJI_M3508)

//堵转时间
#define BLOCK_TIME                  1000
//堵转检测速度
#define BLOCK_TRIGGER_SPEED         5.0f
//回转时间回转速度
#define REVERSE_TIME                1250
#define REVERSE_SPEED               (-20.0f) 


typedef enum 
{
    LOAD_STOP,      // 停止拨盘
    LAOD_BULLET,    // 单发模式
    LOAD_BURSTFIRE,  // 连发模式,对速度闭环
    LOAD_BLOCK       // 堵转，模式
} LoadMode_e;

typedef enum 
{
    FRIC_NOT_READY,      // 未准备发射
    FRIC_READY,          // 准备发射
} FricState_e;

typedef struct feedback
{
  fp32 trigger_angel_fdb;// 拨弹盘输出轴位置
  fp32 trigger_speed_fdb;// 拨弹盘输出轴速度
  fp32 fric_speed_fdb_L;   // 摩擦轮输出轴速度
  fp32 fric_speed_fdb_R;
} Fdb;

typedef struct reference
{
  fp32 trigger_angel_ref;// 拨弹盘位置期望
  fp32 trigger_speed_ref;// 拨弹盘速度期望
  fp32 fric_speed_ref_L;   // 摩擦轮速度期望
  fp32 fric_speed_ref_R;
} Ref;

typedef struct
{
  const RC_ctrl_t * rc;  // 射击使用的遥控器指针
  LoadMode_e mode;       // 射击模式
  FricState_e state;     // 摩擦轮状态

  Motor_s fric_motor[2];  // 摩擦轮电机
  Motor_s trigger_motor;  // 拨弹盘电机

    //pid
  pid_type_def trigger_angel_pid;
  pid_type_def trigger_speed_pid;
  pid_type_def fric_pid[2];

    //block_reverse
  uint16_t reverse_time;
  uint16_t block_time;
  fp32 last_trigger_vel;
  fp32 last_fric_vel;
    
    //feedback
  Fdb FDB;
  
    //reference
  Ref REF;

    //flag
  uint16_t fric_flag; //    摩擦轮状态
  uint16_t move_flag; //    拨弹盘角度状态，用于判断单发射击执行情况
  uint16_t shoot_flag;//    鼠标左键状态，用于判断弹发射击启动

   //ecd
  int16_t last_ecd; //     上一个ecd
  int16_t ecd_count;//     ecd计数

  // heat
  uint16_t heat_limit;
  uint16_t heat;
  uint16_t mr_time;
} Shoot_s;



extern void ShootInit(void);

extern void ShootSetMode(void);

extern void ShootObserver(void);

extern void ShootReference(void);

extern void ShootConsole(void);

extern void ShootSendCmd(void);

#endif  // SHOOT_TYPE == SHOOT_FRIC

