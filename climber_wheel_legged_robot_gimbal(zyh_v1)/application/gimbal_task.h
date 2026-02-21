#ifndef GIMBAL_TASK_H 
#define GIMBAL_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "user_lib.h"


 

/**
  * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
extern void gimbal_task(void const *pvParameters);
void send_NX_vision(void);

#endif
