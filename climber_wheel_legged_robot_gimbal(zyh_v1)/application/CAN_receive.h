/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "motor.h"
#include "can.h"
// clang-format off
/*DJI电机用相关ID定义*/
typedef enum {
    DJI_M1_ID  = 0x201,   // 3508/2006电机ID
    DJI_M2_ID  = 0x202,   // 3508/2006电机ID
    DJI_M3_ID  = 0x203,   // 3508/2006电机ID
    DJI_M4_ID  = 0x204,   // 3508/2006电机ID
    DJI_M5_ID  = 0x205,   // 3508/2006电机ID (/6020电机ID 如分不清关系不建议使用)
    DJI_M6_ID  = 0x206,   // 3508/2006电机ID (/6020电机ID 如分不清关系不建议使用)
    DJI_M7_ID  = 0x207,   // 3508/2006电机ID (/6020电机ID 如分不清关系不建议使用)
    DJI_M8_ID  = 0x208,   // 3508/2006电机ID (/6020电机ID 如分不清关系不建议使用)
    DJI_M9_ID  = 0x209,   // 6020电机ID
    DJI_M10_ID = 0x20A,  // 6020电机ID
    DJI_M11_ID = 0x20B,  // 6020电机ID
} DJI_Motor_ID;

// clang-format off  DM4310
#define DM_MODE_MIT      0x000
#define DM_MODE_POS      0x100
#define DM_MODE_SPEED    0x200
#define DM_MODE_POSI     0x300
#define DM_P_MIN   -3.14159f
#define DM_P_MAX    3.14159f
#define DM_V_MIN   -30.0f
#define DM_V_MAX    30.0f
#define DM_KP_MIN   0.0f
#define DM_KP_MAX   500.0f
#define DM_KD_MIN   0.0f
#define DM_KD_MAX   5.0f
#define DM_T_MIN   -10.0f
#define DM_T_MAX    10.0f

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
typedef struct __MTOC_message
{
	uint16_t power_heat;
	uint16_t bullet_count;
  fp32 bullet_speed;	
	int16_t mouse_RL;
	int16_t mouse_UD;
	int16_t yaw_control;
	int16_t pitch_control;
	uint16_t key_v;
	uint8_t mode;
	uint8_t mouse_press_l;
  uint8_t mouse_press_r;
	fp32 chassis_yaw_speed;
	uint8_t receive_chassis_thing;

}MTOC_message_t;
typedef enum __DmMotorType{
    DM_M1_ID = 0x51,
    DM_M2_ID,
    DM_M3_ID,
    DM_M4_ID,
    DM_M5_ID,
    DM_M6_ID,
} DmMotorType_e;



extern MTOC_message_t mtoc_mesasge;

typedef CAN_HandleTypeDef hcan_t;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

const DjiMotorMeasure_t * GetDjiMotorMeasurePoint(uint8_t can, uint8_t i);
static void GetDjiFdbData(Motor_s *p_motor, const DjiMotorMeasure_t * p_dji_motor_measure);                               \
void DjiFdbData(DjiMotorMeasure_t * dji_measure, uint8_t * rx_data);
void GetMotorMeasure(Motor_s * p_motor);
static void DecodeStdIdData(hcan_t * CAN, CAN_RxHeaderTypeDef * rx_header, uint8_t rx_data[8]);
void CanCmdDjiMotor(uint8_t can, uint16_t std_id, int16_t curr_1, int16_t curr_2, int16_t curr_3, int16_t curr_4);
void SendData(uint8_t can,uint16_t std_id,uint8_t *data);
void DmMitCtrl(Motor_s * motor, float kp, float kd);

void CToM_sendControl(uint8_t can, uint16_t std_id, int16_t yaw);

void DmEnable(Motor_s * motor);

#endif
