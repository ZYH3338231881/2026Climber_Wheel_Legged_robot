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
typedef struct __MTOC_message
{
	uint16_t power_heat;
	uint16_t bullet_count;
  fp32 bullet_speed;	
}MTOC_message_t;

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

void CToM_sendControl(uint8_t can, uint16_t std_id, int16_t yaw);


#endif
