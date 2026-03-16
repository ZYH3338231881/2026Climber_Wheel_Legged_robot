#ifndef MOTOR_H
#define MOTOR_H
#include "main.h"
#include "stdbool.h"
#define RPM_TO_OMEGA 0.1047197551f    // (1/60*2*pi) (rpm)->(rad/s)
#define MOTOR_STABLE_RUNNING_TIME 10  // (ms)电机稳定运行时间
// 可用电机类型
typedef enum __MotorType {
    DJI_M2006 = 0,
    DJI_M3508,
    DJI_M6020,
	  DM_4310,
} MotorType_e;
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
typedef struct _DjiMotorMeasure
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;

    uint32_t last_fdb_time;  //上次反馈时间
} DjiMotorMeasure_t;
void MotorInit( Motor_s * p_motor, uint8_t id, uint8_t can, MotorType_e motor_type, int8_t direction,float reduction_ratio, uint16_t mode);

#endif  // MOTOR_H
