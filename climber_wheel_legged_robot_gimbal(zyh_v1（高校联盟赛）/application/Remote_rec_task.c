#include "gimbal_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "IMU_task.h"
#include "pid.h"
#include "stdio.h"
#include "Remote_rec_task.h"
#include "user_lib.h"
extern osThreadId Remote_rec_taskHandle;
void remote_rec_task(void const *pvParameters)
{
    while (1)
    {
      osDelay(1);
    }
}


