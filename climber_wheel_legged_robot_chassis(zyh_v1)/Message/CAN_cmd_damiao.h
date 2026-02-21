/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       can_cmd_dm.c/h
  * @brief      CAN发送函数，通过CAN信号控制达妙电机 8009.
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     May-15-2024     Penguin         1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#ifndef CAN_CMD_DAMIAO_H
#define CAN_CMD_DAMIAO_H

#include "motor.h"
#include "bsp_can.h"

#ifndef CAN_N
#define CAN_N
#define CAN_1 hcan1
#define CAN_2 hcan2
#endif

/*-------------------- User functions --------------------*/

extern void DmEnable(Motor_s * motor);
extern void DmDisable(Motor_s * motor);
extern void DmSavePosZero(Motor_s * motor);

extern void DmMitStop(Motor_s * motor);
extern void DmMitCtrl(Motor_s * motor, float kp, float kd);
extern void DmMitCtrlTorque(Motor_s * motor);
extern void DmMitCtrlVelocity(Motor_s * motor, float kd);
extern void DmMitCtrlPosition(Motor_s * motor, float kp, float kd);

extern void DmPosCtrl(Motor_s * motor);    //TODO：测试可用性
extern void DmSpeedCtrl(Motor_s * motor);  //TODO：测试可用性
extern void CanCmdDjiMotor(hcan_t * hcan, uint16_t std_id, int16_t curr_1, int16_t curr_2, int16_t curr_3, int16_t curr_4);
extern void MToC_sendControl(uint8_t can, uint16_t std_id, int16_t power_heat , fp32 bullet_speed);
#endif /* CAN_CMD_DAMIAO_H */
/************************ END OF FILE ************************/
