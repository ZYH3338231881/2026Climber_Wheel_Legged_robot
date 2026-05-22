/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       motor.c/h
  * @brief      电机相关部分定义
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"
#include "robot_typedef.h"
#include "stdbool.h"
#include "struct_typedef.h"

#define RPM_TO_OMEGA 0.1047197551f    // (1/60*2*pi) (rpm)->(rad/s)
#define DEGREE_TO_RAD 0.0174532925f   // (pi/180) (degree)->(rad)
#define RAD_TO_DEGREE 57.2957795131f  // (180/pi) (rad)->(degree)

#define MOTOR_STABLE_RUNNING_TIME 10  // (ms)电机稳定运行时间

/*-------------------- DJI Motor --------------------*/

// 电机编码值规整 0—8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#define DJI_GM6020_ECD_TO_RAD 0.000766990394f  // (2*pi/8192) 电机编码器值转换为弧度
#define DJI_GM3508_RPM_TO_OMEGA 0.0055115661f  // (1/60*2*pi/19) m3508(减速比19:1) (rpm)->(rad/s)
#define DJI_GM2006_RPM_TO_OMEGA 0.0029088821f  // (1/60*2*pi/36) m2006(减速比36:1) (rpm)->(rad/s)

#define DJI_GM2006_MAX_CURRENT 10000
#define DJI_GM3508_MAX_CURRENT 16384
#define DJI_GM6020_MAX_VOLTAGE 30000

// 电机模式
#define DJI_CURRENT_MODE ((uint16_t)0xAA)  // 电流控制模式
#define DJI_VOLTAGE_MODE ((uint16_t)0xBB)  // 电压控制模式

#define DJI_3508_MODE_CURRENT_1 ((uint16_t)0x200)  // 3508电流控制模式
#define DJI_2006_MODE_CURRENT_1 ((uint16_t)0x200)  // 2006电流控制模式

#define DJI_3508_MODE_CURRENT_2 ((uint16_t)0x1FF)  // 3508电流控制模式
#define DJI_2006_MODE_CURRENT_2 ((uint16_t)0x1FF)  // 2006电流控制模式

#define DJI_6020_MODE_VOLTAGE_1 ((uint16_t)0x1FF)  // 6020电压控制模式
#define DJI_6020_MODE_VOLTAGE_2 ((uint16_t)0x2FF)  // 6020电压控制模式

// #define DJI_6020_MODE_CURRENT_1 0x2FE  // 6020电流控制模式

typedef struct _DjiMotorMeasure
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;

    uint32_t last_fdb_time;  //上次反馈时间
} DjiMotorMeasure_t;

/*-------------------- CyberGear --------------------*/
#define CYBERGEAR_NUM 5

// clang-format off
#define CYBERGEAR_MODE_TORQUE  0x000
#define CYBERGEAR_MODE_POS     0x100
#define CYBERGEAR_MODE_SPEED   0x200
// clang-format on

typedef enum _CybergearModeState {
    UNDEFINED_MODE = -1,  //未定义模式
    RESET_MODE = 0,       //Reset模式[复位]
    CALI_MODE = 1,        //Cali 模式[标定]
    RUN_MODE = 2          //Motor模式[运行]
} CybergearModeState_e;   //电机模式状态

typedef struct
{
    uint32_t info : 24;
    uint32_t communication_type : 5;
    uint32_t res : 3;
} __attribute__((packed)) RxCanInfo_s;  // 解码内容缓存

typedef struct
{
    uint32_t FE : 8;
    uint32_t motor_id : 16;
    uint32_t communication_type : 5;
    uint32_t res : 3;
    uint32_t MCU_id;
} __attribute__((packed)) RxCanInfoType_0_s;  // 通信类型0解码内容

typedef struct
{
    uint32_t master_can_id : 8;
    uint32_t motor_id : 8;
    uint32_t under_voltage_fault : 1;
    uint32_t over_current_fault : 1;
    uint32_t over_temperature_fault : 1;
    uint32_t magnetic_encoding_fault : 1;
    uint32_t HALL_encoding_failure : 1;
    uint32_t unmarked : 1;
    uint32_t mode_state : 2;
    uint32_t communication_type : 5;
    uint32_t res : 3;
} __attribute__((packed)) RxCanInfoType_2_s;  // 通信类型2解码内容

typedef struct
{
    RxCanInfo_s ext_id;
    uint8_t rx_data[8];

    uint32_t last_fdb_time;  //上次反馈时间
} CybergearMeasure_s;

/*-------------------- DM Motor --------------------*/
#define DM_NUM 4

// clang-format off
#define DM_MODE_MIT      0x000
#define DM_MODE_POS      0x100
#define DM_MODE_SPEED    0x200
#define DM_MODE_POSI     0x300

#define DM_STATE_DISABLE                0x00
#define DM_STATE_ENABLE                 0x01
#define DM_STATE_OVERVOLTAGE            0x08
#define DM_STATE_UNDERVOLTAGE           0x09
#define DM_STATE_OVERCURRENT            0x0A
#define DM_STATE_MOS_OVER_TEMPERATURE   0x0B
#define DM_STATE_COIL_OVER_TEMPERATURE  0x0C
#define DM_STATE_COMMUNICATION_LOSS     0x0D
#define DM_STATE_OVERLOAD               0x0E

//当时设置错误的8009P参数，其实下面这个参数应该是4310的
//DM_P应该是弧度两圈，但是默认的12.5会有问题，会丢精度
//#define DM_P_MIN   -12.56637f
//#define DM_P_MAX    12.56637f
//#define DM_V_MIN   -30.0f
//#define DM_V_MAX    30.0f
//#define DM_KP_MIN   0.0f
//#define DM_KP_MAX   500.0f
//#define DM_KD_MIN   0.0f
//#define DM_KD_MAX   5.0f
//#define DM_T_MIN   -10.0f
//#define DM_T_MAX    10.0f

//下面是正确的8009P参数
#define DM_P_MIN   -12.56637f
#define DM_P_MAX    12.56637f
#define DM_V_MIN   -45.0f
#define DM_V_MAX    45.0f
#define DM_KP_MIN   0.0f
#define DM_KP_MAX   500.0f
#define DM_KD_MIN   0.0f
#define DM_KD_MAX   5.0f
#define DM_T_MIN   -54.0f
#define DM_T_MAX    54.0f
// clang-format on
typedef struct
{
    int id;
    int state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;

    float pos;
    float vel;
    float tor;
    float Kp;
    float Kd;

    float t_mos;
    float t_rotor;

    uint32_t last_fdb_time;  //上次反馈时间
} DmMeasure_s;

/*-------------------- LK Motor --------------------*/
#define LK_NUM 4

// clang-format off
#define LK_MAX_MULTICONTROL_IQ  2000
#define LK_MIN_MULTICONTROL_IQ -2000
#define LK_MAX_MF_CONTROL_IQ    2048
#define LK_MIN_MF_CONTROL_IQ   -2048
#define LK_MAX_MULTICONTROL_CURRENT  32.0f
#define LK_MIN_MULTICONTROL_CURRENT -32.0f
#define LK_MAX_MF_CONTROL_CURRENT  16.5f
#define LK_MIN_MF_CONTROL_CURRENT -16.5f

#define LK_MAX_MF_TORQUE  2.41f
#define LK_MIN_MF_TORQUE -2.41f

#define MF_CONTROL_TO_CURRENT 0.008056640625f  // (16.5/2048)(A)控制量转换为电流
// clang-format on

typedef struct
{
    int8_t ctrl_id;
    int8_t temprature;
    int16_t iq;
    int16_t speed;
    uint16_t encoder;

    uint32_t last_fdb_time;  //上次反馈时间
} LkMeasure_s;

/*-------------------- Motor struct --------------------*/

/**
 * @brief  通用电机结构体
 * @note   包括电机的信息、状态量和控制量
 * @note   电机信息部分的参数不影响电机的反馈数据，由用户自行使用与处理
 */
typedef struct __Motor
{
    /*电机信息*/
    uint8_t id;             // 电机ID
    MotorType_e type;       // 电机类型
    uint8_t can;            // 电机所用CAN口
    float reduction_ratio;  // 电机减速比，例如2006为36:1，则reduction_ratio=36
    int8_t direction;       // 电机和(执行机构在模型中定义的旋转方向)的关系（1或-1），例如：
    uint16_t mode;          // 电机模式
    bool offline;           // 电机是否离线

    /*状态量*/
    struct __fdb
    {
        float acc;  // (rad/s^2)电机加速度

        float vel;   // (rad/s) 电机反馈转速
        float tor;   // (N*m)   电机反馈力矩
        float pos;   // (rad)   电机反馈位置
        float temp;  // (℃)    电机反馈温度
        float curr;  // (A)     电机反馈电流

        int16_t round;  // (r)电机旋转圈数(用于计算输出轴位置)
        uint16_t ecd;   // 电机编码器值
        uint8_t state;  // 电机状态
    } fdb;

    /*设定值*/
    struct __set
    {
        float curr;  // (A)     电机设定电流
        float volt;  // (V)     电机设定电压
        float tor;   // (N*m)   电机设定力矩
        float vel;   // (rad/s) 电机设定转速
        float pos;   // (rad)   电机设定位置

        float value;  // 可发送的直接控制量，无单位
    } set;

} Motor_s;



/*-------------------- Motor function --------------------*/
extern void DmFdbData(DmMeasure_s * dm_measure, uint8_t * rx_data);
extern void GetDmFdbData(Motor_s *motor,DmMeasure_s *dm_measure);
extern void DjiFdbData(DjiMotorMeasure_t * dji_measure, uint8_t * rx_data);
extern void GetDjiFdbData(Motor_s * p_motor, const DjiMotorMeasure_t * p_dji_motor_measure);
extern void GetMotorMeasure(Motor_s * p_motor);
#endif  // MOTOR_H
