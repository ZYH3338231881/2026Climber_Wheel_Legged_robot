#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H
#include "struct_typedef.h"
#include "remote_control.h"
#include <stdio.h>
#include "motor.h"
#include "pid.h"
#include "user_lib.h"

#define GIMBAL_UPPER_LIMIT_PITCH (0.3f)  //弧度值
#define GIMBAL_LOWER_LIMIT_PITCH (-0.3f)

//YAW ANGLE
#define KP_GIMBAL_YAW_ANGLE (150)//150
#define KI_GIMBAL_YAW_ANGLE (0)
#define KD_GIMBAL_YAW_ANGLE (1400)//70
#define MAX_IOUT_GIMBAL_YAW_ANGLE (3)
#define MAX_OUT_GIMBAL_YAW_ANGLE (15)
//VELOCITY:角速度
#define KP_GIMBAL_YAW_VELOCITY (1500)//1200
#define KI_GIMBAL_YAW_VELOCITY (2)
#define KD_GIMBAL_YAW_VELOCITY (100)
#define MAX_IOUT_GIMBAL_YAW_VELOCITY (5000)
#define MAX_OUT_GIMBAL_YAW_VELOCITY (16384)

//#define KP_GIMBAL_YAW_ANGLE (14)
//#define KI_GIMBAL_YAW_ANGLE (0.005)
//#define KD_GIMBAL_YAW_ANGLE (70)
//#define MAX_IOUT_GIMBAL_YAW_ANGLE (5)
//#define MAX_OUT_GIMBAL_YAW_ANGLE (100)
////VELOCITY:角速度
//#define KP_GIMBAL_YAW_VELOCITY (1500)
//#define KI_GIMBAL_YAW_VELOCITY (4.5)
//#define KD_GIMBAL_YAW_VELOCITY (10)
//#define MAX_IOUT_GIMBAL_YAW_VELOCITY (9000)
//#define MAX_OUT_GIMBAL_YAW_VELOCITY (16384)


//先单环后串级
//PITCH ANGLE
#define KP_GIMBAL_PITCH_ANGLE (170)     //保证位置响应较快
#define KI_GIMBAL_PITCH_ANGLE (0)        
#define KD_GIMBAL_PITCH_ANGLE (1300)       //位置稳态，过多可能冲过头
#define MAX_IOUT_GIMBAL_PITCH_ANGLE (5)
#define MAX_OUT_GIMBAL_PITCH_ANGLE (20)  //在速度单环pid下实际可能出现的最大转速
//VELOCITY:角速度
#define KP_GIMBAL_PITCH_VELOCITY (1500)         //保证速度响应较快800
#define KI_GIMBAL_PITCH_VELOCITY (5)           
#define KD_GIMBAL_PITCH_VELOCITY (300)          
#define MAX_IOUT_GIMBAL_PITCH_VELOCITY (7000)   //修正速度静差
#define MAX_OUT_GIMBAL_PITCH_VELOCITY (16384)  //6020最大电流

//电机id
#define GIMBAL_DIRECT_YAW_ID ((uint8_t)1)
#define GIMBAL_DIRECT_PITCH_ID ((uint8_t)2)

//云台电流发送ID
#define GIMBAL_CAN_CMD_YAW (1) //CAN1
#define GIMBAL_CAN_CMD_PITCH (2) //CAN2
#define GIMBAL_STDID_1 (0x1FE)


//电机can口
#define GIMBAL_DIRECT_YAW_CAN ((uint8_t)1)
#define GIMBAL_DIRECT_PITCH_CAN ((uint8_t)2)

//电机种类
#define GIMBAL_DIRECT_YAW_MOTOR_TYPE ((MotorType_e)DJI_M6020)
#define GIMBAL_DIRECT_PITCH_MOTOR_TYPE ((MotorType_e)DJI_M6020)

//旋转方向
#define GIMBAL_DIRECT_YAW_DIRECTION (1)
#define GIMBAL_DIRECT_PITCH_DIRECTION (1)

//减速比
#define GIMBAL_DIRECT_YAW_REDUCTION_RATIO (1)
#define GIMBAL_DIRECT_PITCH_REDUCTION_RATIO (1)

//电机运行模式
#define GIMBAL_DIRECT_YAW_MODE (0)
#define GIMBAL_DIRECT_PITCH_MODE (0)

//电机角度中值设置
//#define GIMBAL_DIRECT_PITCH_MID (-3.14082575f)  //云台初始化正对齐的时候使用的pitch轴正中心量
#define GIMBAL_DIRECT_YAW_MID (0.452524424)    //云台初始化正对齐的时候使用的yaw轴正中心量

//gimbal_init-------------------------------
#define GIMBAL_INIT_TIME (uint32_t)1000

/**
 * @brief 云台模式
 */
typedef enum {
    GIMBAL_ZERO_FORCE,  // 云台无力，所有控制量置0
    GIMBAL_HAND,         // 云台手动控制(IMU反馈)
    GIMBAL_AUTO_AIM,    //自瞄模式
   	GIMBAL_INIT,        //云台初始化
} GimbalMode_e;
/**
 * @brief 状态、期望和限制值
 */
typedef struct
{
    float pitch;
    float yaw;
	  float roll;
} Values_t;
typedef struct
{
    pid_type_def yaw_angle;
    pid_type_def yaw_velocity;  //角速度

    pid_type_def pitch_angle;
    pid_type_def pitch_velocity;
    
} PID_t;
typedef struct
{
    const RC_ctrl_t * rc;  // n遥控器指针
    GimbalMode_e mode,last_mode,mode_before_rc_err;  // 模式

    /*-------------------- Motors --------------------*/
    Motor_s yaw,pitch;
    /*-------------------- Values --------------------*/
    Values_t reference;    // 期望值
    Values_t feedback_pos,feedback_vel;     // 状态值(目前专供给IMU数据)
    Values_t upper_limit;  // 上限值
    Values_t lower_limit;  // 下限值

    PID_t pid;  // PID控制器

    float angle_zero_for_imu; //pitch电机处于中值时imupitch的角度

    uint32_t init_start_time,init_timer;

    bool init_continue; //是否继续进行校准模式
	
	  fp32 Signal_generation;    //信号产生   
	
	  uint32_t last_time;  // (ms)上一次更新时间
    uint32_t duration;   // (ms)任务周期
} Gimbal_s;

typedef struct LPF
{
	LowPassFilter_t GIMBAL_INIT;//云台初始化滤波
} LPF_t;



void GimbalInit(void);
void GimbalSetMode(void);
void GimbalObserver(void);
void GimbalReference(void);
void GimbalConsole(void);
void GimbalSendCmd(void);


#endif
