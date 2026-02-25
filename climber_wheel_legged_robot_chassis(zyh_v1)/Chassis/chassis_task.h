/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_task.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
	*  v1         2025.10.15      zyh             Climber
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "struct_typedef.h"
#include "bsp_rc.h"
#include "motor.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "kalman_filter.h"
#include "motor.h"

// 一些内部的配置
#define TAKE_OFF_DETECT 1 // 启用离地检测
#define OPEN_CALIBRATE 0    // 启用校准功能
#define ENBAL_keyboad 0    // 启用键鼠功能
#define OPEN_CHASSIS_FOLLOW_GIMBAL 1//启用底盘跟随云台功能

// 机器人状态
#define NORMAL_STEP        0  // 正常状态
#define JUMP_STEP_SQUST    1  // 跳跃状态——蹲下  
#define JUMP_STEP_JUMP     2  // 跳跃状态——跳跃
#define JUMP_STEP_RECOVERY 3  // 跳跃状态——收腿

#define MAX_STEP_TIME           5000  // 最大步骤时间

#define NORMAL_STEP_TIME        0  // 正常状态
#define JUMP_STEP_TIME_SQUST    50  // 跳跃状态——蹲下
#define JUMP_STEP_TIME_JUMP     50  // 跳跃状态——跳跃
#define JUMP_STEP_TIME_RECOVERY 200  // 跳跃状态——收腿

// motor parameters ---------------------
#define JOINT_CAN (1)  //fdcan1
#define WHEEL_CAN (2)  //fdcan2

#define J0_DIRECTION (-1)
#define J1_DIRECTION (-1)
#define J2_DIRECTION (1)
#define J3_DIRECTION (1)

#define W0_DIRECTION ( -1)
#define W1_DIRECTION (  1)

//PID parameters ---------------------
//yaw轴跟踪角度环PID参数
#define KP_CHASSIS_YAW_ANGLE        (0)
#define KI_CHASSIS_YAW_ANGLE        (0)
#define KD_CHASSIS_YAW_ANGLE        (0)
#define MAX_IOUT_CHASSIS_YAW_ANGLE  (0)
#define MAX_OUT_CHASSIS_YAW_ANGLE   (0)

//yaw轴跟踪速度环PID参数
#define KP_CHASSIS_YAW_VELOCITY        (2.0f)
#define KI_CHASSIS_YAW_VELOCITY        (0.05f)
#define KD_CHASSIS_YAW_VELOCITY        (10.0f)
#define MAX_IOUT_CHASSIS_YAW_VELOCITY  (0.5f)
#define MAX_OUT_CHASSIS_YAW_VELOCITY   (10.0f)


// vel_add PID参数
#define KP_CHASSIS_VEL_ADD        (0) //0.1
#define KI_CHASSIS_VEL_ADD        (0)//0.005
#define KD_CHASSIS_VEL_ADD        (0)//0.001
#define MAX_IOUT_CHASSIS_VEL_ADD  (0)//0.5
#define MAX_OUT_CHASSIS_VEL_ADD   (0)//1.0

/*========== Start of locomotion control pid ==========*/

//pitch轴跟踪角度环PID参数
#define KP_CHASSIS_PITCH_ANGLE        (1000)
#define KI_CHASSIS_PITCH_ANGLE        (0)
#define KD_CHASSIS_PITCH_ANGLE        (1500)
#define MAX_IOUT_CHASSIS_PITCH_ANGLE  (0)
#define MAX_OUT_CHASSIS_PITCH_ANGLE   (60)

// 腿长跟踪长度环PID参数
#define KP_CHASSIS_LEG_LENGTH_LENGTH        (3000.0f)
#define KI_CHASSIS_LEG_LENGTH_LENGTH        (0.0f)
#define KD_CHASSIS_LEG_LENGTH_LENGTH        (9000.0f)
#define MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH  (0.0f)
#define MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH   (250.0f)
#define N_LEG_LENGTH_LENGTH                 (0.2f)

// 起立用的pid
#define KP_CHASSIS_STAND_UP       (1200)
#define KI_CHASSIS_STAND_UP       (0)
#define KD_CHASSIS_STAND_UP       (100)
#define MAX_IOUT_CHASSIS_STAND_UP (0)
#define MAX_OUT_CHASSIS_STAND_UP  (270)

// 磨轮子用的pid
#define KP_CHASSIS_WHEEL      (0)
#define KI_CHASSIS_WHEEL       (0)
#define KD_CHASSIS_WHEEL       (0)
#define MAX_IOUT_CHASSIS_WHEEL (0)
#define MAX_OUT_CHASSIS_WHEEL  (0)

//// 云台跟随用的单环
//#define KP_CHASSIS_FOLLOW_GIMBAL       (3)
//#define KI_CHASSIS_FOLLOW_GIMBAL       (0)
//#define KD_CHASSIS_FOLLOW_GIMBAL       (500)
//#define MAX_IOUT_CHASSIS_FOLLOW_GIMBAL (0)
//#define MAX_OUT_CHASSIS_FOLLOW_GIMBAL  (3)

// 云台跟随用的pid角度环
#define KP_CHASSIS_FOLLOW_GIMBAL       (10)
#define KI_CHASSIS_FOLLOW_GIMBAL       (0)
#define KD_CHASSIS_FOLLOW_GIMBAL       (10)
#define MAX_IOUT_CHASSIS_FOLLOW_GIMBAL (0)
#define MAX_OUT_CHASSIS_FOLLOW_GIMBAL  (4)


//防劈叉PID
#define KP_CHASSIS_TP       (12)
#define KI_CHASSIS_TP       (0)
#define KD_CHASSIS_TP       (200)
#define MAX_IOUT_CHASSIS_TP (1)
#define MAX_OUT_CHASSIS_TP  (4)
  
//追腿部PID 参数
//左腿追右腿
#define KP_CHASSIS_CAHSE_LEG_L_to_R        (170)
#define KI_CHASSIS_CAHSE_LEG_L_to_R        (0)
#define KD_CHASSIS_CAHSE_LEG_L_to_R        (1500)
#define MAX_IOUT_CHASSIS_CAHSE_LEG_L_to_R  (0)
#define MAX_OUT_CHASSIS_CAHSE_LEG_L_to_R   (10)
//右腿追左腿
#define KP_CHASSIS_CAHSE_LEG_R_to_L        (170)
#define KI_CHASSIS_CAHSE_LEG_R_to_L        (0)
#define KD_CHASSIS_CAHSE_LEG_R_to_L        (1500)
#define MAX_IOUT_CHASSIS_CAHSE_LEG_R_to_L  (0)
#define MAX_OUT_CHASSIS_CAHSE_LEG_R_to_L   (10)



//LPF parameters ---------------------
#define LEG_DDL0_LPF_ALPHA           (0.6f)
#define LEG_DDPHI0_LPF_ALPHA         (0.0f)
#define LEG_DDTHETA_LPF_ALPHA        (0.0f)
#define LEG_SUPPORT_FORCE_LPF_ALPHA  (0.9f)
#define CHASSIS_PICTH_ALPHA           (0.0f)
#define L_LEG_THETA_ALPHA             (0.8f)
#define R_LEG_THETA_ALPHA             (0.8f)
#define dtheta_THETA_ALPHA             (0.8f)


//offest parameters ---------------------


#define CALIBRATE_VELOCITY 2.0f        // rad/s  校准速度

#define CALIBRATE_STOP_VELOCITY 0.05f  // rad/s 校准停止速度
#define CALIBRATE_STOP_TIME 200        // ms   校准停止时间

#define Leg_Stretch_velocity 2.0f      // (rad/s) 伸腿速度
#define Leg_Stretch_stop_time 300 // ms  伸腿停止时间
#define Leg_Stretch_stop_velocity 0.05f

#define Leg_back_velocity 1.0f          // (rad/s) 起立向后摔腿速度
#define Leg_back_stop_time 300 // ms   起立向后摔腿停止时间
#define Leg_back_stop_velocity 0.05f

#define VEL_PROCESS_NOISE 25   // 速度过程噪声
#define VEL_MEASURE_NOISE 800  // 速度测量噪声
// 同时估计加速度和速度时对加速度的噪声
// 更好的方法是设置为动态,当有冲击时/加加速度大时更相信轮速
#define ACC_PROCESS_NOISE 2000  // 加速度过程噪声
#define ACC_MEASURE_NOISE 0.01  // 加速度测量噪声

#define STAND_UP_KP (4) // 起立收腿MIT_KP
#define STAND_UP_KD (0.8)  // 起立收腿MIT_KD
#define CALIBRATE_VEL_KD  (4)  // 校准MIT速度控制KD
#define STAND_UP_VEL_KD  (5)  // 起立MIT速度控制KD
#define ZERO_FORCE_VEL_KP (0)  // 无力MIT速度控制KP


#define J0_ANGLE_OFFSET     (-0.0964508057+M_PI) // (rad)关节0角度偏移量(电机0点到水平线的夹角)
#define J1_ANGLE_OFFSET     (0.0507124329)         // (rad)关节1角度偏移量(电机0点到水平线的夹角)
#define J2_ANGLE_OFFSET     (-0.0857124329+ M_PI)  // (rad)关节2角度偏移量(电机0点到水平线的夹角)
#define J3_ANGLE_OFFSET     (-0.0500469208 )        // (rad)关节3角度偏移量(电机0点到水平线的夹角)


#define WHEEL_DEADZONE (0.01f)  // (m/s)轮子速度死区       //待考量
//#define BODY_MASS            (13.0f)      // (kg)机身重量   

#define WHEEL_MASS           (0.5f)      // (kg)轮子重量  
#define WHEEL_RADIUS         (0.09f)    // (m)轮子半径     
#define WHEEL_START_TORQUE   (0.3f)      // (Nm)轮子起动力矩  //待考量  
#define WHEEL_BASE           (0.42)  // (m)驱动轮轴距  

// 支持力阈值，当支持力小于这个值时认为离地
#define TAKE_OFF_FN_THRESHOLD (10.0f)  //待考量，建议可以用vofa打印出来
// 触地状态切换时间阈值，当时间接触或离地时间超过这个值时切换触地状态
#define TOUCH_TOGGLE_THRESHOLD (50)
#define MIN_LEG_LENGTH       ( 0.14f)         //最短腿长  
#define MAX_LEG_LENGTH       (0.35f)          //最大腿长  
#define MAX_DELTA_ROD_ANGLE (0.25f) // (rad)腿摆角最大变化量  待考量
#define MIN_LEG_ANGLE        (M_PI_2 - MAX_DELTA_ROD_ANGLE) //最小角度  待考量
#define MAX_LEG_ANGLE        (M_PI_2 + MAX_DELTA_ROD_ANGLE)  //最大角度  待考量
#define MAX_PITCH            (0.3f)  //最大pitch轴   //待考量
#define MIN_PITCH           (-MAX_PITCH)//最小pitch轴//待考量
#define SPRING_X_CONST  0.0515f   //弹簧力臂长度
#define SPRING_F_RAW    300.0f    //弹簧原始力



#define MAX_TP    (15)    //限制最大旋转扭矩
#define MIN_TP    (-MAX_TP)

#define CHASSIS_RC_DEADLINE    20 // 摇杆死区
#define CHASSIS_RC_PITCH_DEADLINE 100  //ROLL轴摇杆死区
#define CHASSIS_X_CHANNEL      1  // 前后的遥控器通道号码
#define CHASSIS_WZ_CHANNEL     2  // 旋转的遥控器通道号码

#define RC_TO_ONE 0.0015151515151515f  // (1/660)遥控器通道值归一化系数
#define MAX_SPEED_VECTOR_VX  (4.5f)
#define MAX_SPEED_VECTOR_WZ  (15.0f)

#define PITCH_VEL_LIMIT_FACTOR  (0.1f)    // pitch角速度抑制比例系数
#define FF_RATIO                (0.25f)   // 前馈比例系数
#define max_joint_tor_move      (20.0f)   // (Nm)运动时关节最大扭矩
#define max_joint_tor_stand     (30.0f)  // 	起立时候的关节最大扭矩
#define min_joint_tor_move      (-max_joint_tor_move)  // 
#define min_joint_tor_stand     (-max_joint_tor_stand)  // 

#define X_ADD_RATIO 1   //位移设定缩放系数
typedef struct __Imu
{
    float angle[3];  // rad 欧拉角数据
    float gyro[3];   // rad/s 陀螺仪数据
    float accel[3];  // m/s^2 加速度计数据
} Imu_t;


typedef struct Body
{
    float x;          // (m)机体位移距离
    float x_obv;      // (m)机体位移观测值
    float x_dot;      // (m/s)机体速度直接反馈值
    float x_dot_obv;  // (m/s)机体速度观测值
    float x_acc;      // (m/s^2)机体x轴加速度直接反馈值
    float x_acc_obv;  // (m/s^2)机体x轴加速度观测值

    float x_accel;  // 机体坐标系下x轴加速度
    float y_accel;  // 机体坐标系下y轴加速度
    float z_accel;  // 机体坐标系下z轴加速度

    float gx, gy, gz;  //重力加速度在机体坐标系下的分量，用于消除重力加速度对加速度计的影响

    float phi;
    float phi_dot;

    float roll;
    float roll_dot;
    float pitch;
    float pitch_dot;
    float yaw;
    float yaw_dot;
} Body_t;

typedef struct Gimbal
{
  fp32 gimbal_yaw_6020;
} Gimbal_t;

typedef struct
{
    float x_accel;  // 世界坐标系下x轴加速度
    float y_accel;  // 世界坐标系下y轴加速度
    float z_accel;  // 世界坐标系下z轴加速度
} World_t;

//状态向量
typedef struct LegState
{
    float theta;
    float theta_dot;
    float x;
    float x_dot;
    float phi;
    float phi_dot;
} LegState_t;
typedef struct  // 底盘速度向量结构体
{
    float vx;  // (m/s) x方向速度
    float vy;  // (m/s) y方向速度
    float wz;  // (rad/s) 旋转速度
} ChassisSpeedVector_t;
typedef struct  // 底盘速度向量结构体
{
    float vx;  // (m/s) x方向速度
    float vy;  // (m/s) y方向速度
    float wz;  // (rad/s) 旋转速度
} ComputerSpeedVector_t;



typedef struct Leg
{
    struct rod
    {
        float Phi0;    // rad
        float dPhi0;   // rad/s
        float ddPhi0;  // rad/s^2

        float L0;    // m
        float dL0;   // m/s
        float ddL0;  // m/s^2

        float Theta;    // rad
        float dTheta;   // rad/s
        float ddTheta;  // rad/s^2

        float F;   // N         根据关节反馈力矩和雅可比矩阵反接出来的支持力，只由关节提供
        float Tp;  // N*m       旋转扭矩
				float F_spring; //弹簧在竖直方向上的分力
			  float F_costheta;      //支持力在竖直上的分量
			  float Tp_sintheta;     //旋转力在竖直上的分量
    } rod;    //rod（中文）连杆

    struct joint
    {
        float T1, T2;        // N*m
        float Phi1, Phi4;    // rad
        float dPhi1, dPhi4;  // rad/s
    } joint;

    struct wheel
    {
        float Angle;     // rad
        float Velocity;  // rad/s
    } wheel;

    float J[2][2];           // 雅可比矩阵
    float Fn;                // N 支撑力
    uint32_t take_off_time;  // 离地时间
    uint32_t touch_time;     // 触地时间
    bool is_take_off;        // 是否离地
} Leg_t;
/**
 * @brief 期望
 */
typedef struct
{
    Body_t body;
    LegState_t leg_state[2];  // 0-左 1-右
    float rod_L0[2];          // 0-左 1-右
    float rod_Angle[2];       // 0-左 1-右
    ChassisSpeedVector_t speed_vector;
} Ref_t;
/**
 * @brief 状态
 */
typedef struct
{
    Body_t body;
    World_t world;
    Leg_t leg[2];             // 0-左 1-右
	  float two_leg_err;        //两腿之间的差值
    LegState_t leg_state[2];  // 0-左 1-右
    ChassisSpeedVector_t speed_vector;
	  Gimbal_t gimbal;
} Fdb_t;

typedef struct
{
    pid_type_def yaw_velocity;

    pid_type_def pitch_angle;
    


    pid_type_def leg_length_length[2];
    pid_type_def leg_length_speed[2];
    pid_type_def chassis_follow_gimbal;
    pid_type_def leg_angle_angle;
    pid_type_def stand_up;
    pid_type_def wheel_stop[2];
	  pid_type_def leg_tp;
    pid_type_def leg_chase_L_to_R;
	  pid_type_def leg_chase_R_to_L;

} PID_t;


typedef struct LPF
{
    LowPassFilter_t leg_l0_accel_filter[2];
    LowPassFilter_t leg_phi0_accel_filter[2];
    LowPassFilter_t leg_theta_accel_filter[2];
    LowPassFilter_t support_force_filter[2];
    LowPassFilter_t pitch;//机体翻滚角度
	  LowPassFilter_t L_theta;//摆杆与竖直方向夹角滤波
	  LowPassFilter_t R_theta;//摆杆与竖直方向夹角滤波
	  LowPassFilter_t VX_filter;//键鼠信号阶跃滤波
	


} LPF_t;

typedef struct LQR
{
  fp32 wheel_theta;
	fp32 wheel_theta_dot;
	fp32 wheel_x;
	fp32 wheel_vel;
	fp32 wheel_phi;
	fp32 wheel_phi_dot;
	fp32 joint_theta;
	fp32 joint_theta_dot;
	fp32 joint_x;
	fp32 joint_vel;
	fp32 joint_phi;
	fp32 joint_phi_dot;
} LQR_OUT_t;

typedef struct
{
    struct
    {
        KalmanFilter_t v_kf;  // 观测车体速度
    } body;
} Observer_t;

typedef struct Calibrate
{
    uint32_t cali_cnt;      //记录遥控器摇杆保持校准姿态的次数（等效于时间）
    float velocity[4];      //关节电机速度
    uint32_t stpo_time[4];  //停止时间
    bool reached[4];        //是否到达限位
    bool calibrated;        //完成校准
    bool toggle;            //切换校准状态
} Calibrate_s;




typedef enum {
    CHASSIS_OFF=0,        // 底盘关闭
    CHASSIS_SAFE=1,       // 底盘无力，所有控制量置0
	CHASSIS_STAND_UP=2,   // 底盘起立，从倒地状态到站立状态的中间过程
    CHASSIS_FREE=3,       // 底盘不跟随云台 以底盘为坐标系移动
    CHASSIS_CALIBRATE=4,  // 底盘校准
    CHASSIS_FOLLOW_GIMBAL_YAW=5,  // 底盘跟随云台（运动方向为云台坐标系方向，需进行坐标转换）
    CHASSIS_READY=6,
    CHASSIS_CRASHING=7,   // 底盘接地状态，进行缓冲
    CHASSIS_AUTO=8,       // 底盘自动模式
    CHASSIS_OFF_HOOK=9,   // 底盘脱困模式
    CHASSIS_CUSTOM=12,      // 自定义模式
} ChassisMode_e;
typedef struct Cmd
{
    struct leg
    {
        struct rod_cmd
        {
            float F;   // N
            float Tp;  // N*m
        } rod;
        struct joint_cmd
        {
            float T[2];    // N*m
            float Pos[2];  // rad
        } joint;
        struct wheel_cmd
        {
            float T;  // N*m
        } wheel;
    } leg[2];  // 0-左 1-右
} Cmd_t;


/**
 * @brief  底盘数据结构体
 * @note   底盘坐标使用右手系，前进方向为x轴，左方向为y轴，上方向为z轴
 */
typedef struct
{
    const RC_ctrl_t * rc;  // 底盘使用的遥控器指针
	  Imu_t * imu;     // imu数据
	  ChassisMode_e mode;    // 底盘模式
    ChassisMode_e last_mode;
    /*-------------------- Motors --------------------*/
    // 平衡底盘有2个驱动轮电机和4个关节电机
    Motor_s joint_motor[4];
    Motor_s wheel_motor[2];  // 驱动轮电机 0-左轮，1-右轮
    /*-------------------- Values --------------------*/
	
    Ref_t ref;  // 期望值
    Fdb_t fdb;  // 状态值
  	Cmd_t cmd;  // 控制量

    int8_t step;           // 底盘运行步骤号
    uint32_t step_time;    // (ms)底盘步骤运行时间
	
	  PID_t pid;  // PID控制器
	  LPF_t lpf;  // 低通滤波器
	
    uint32_t last_time;  // (ms)上一次更新时间
    uint32_t duration;   // (ms)任务周期
		
		LQR_OUT_t lqr_out;

} Chassis_s;

typedef struct __CTOM_message
{
  fp32 gimbal_yaw_6020;
	
}CTOM_message_t;



extern void chassis_task(void const * pvParameters);
#endif
