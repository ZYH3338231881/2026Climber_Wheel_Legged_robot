#include "chassis_task.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "string.h"
#include "bsp_delay.h"
#include "stdio.h"
#include "State_chexk_task.h"
extern Chassis_s CHASSIS ;
extern void standup_STATE();//腿部姿态调整

State StandUP_state_L=STATE_NORMAL;   
State StandUP_state_R=STATE_NORMAL;   


//主要用于各种状态机的检测
void State_check_task(void const * pvParameters)
{
    StandUP_state_L=STATE_stretchleg; 
    StandUP_state_R=STATE_stretchleg;   
  
    while (1) {
		if (CHASSIS.mode==CHASSIS_OFF_HOOK)
		{
			standup_STATE();
		}
		
		osDelay(1);
    }
}

void standup_STATE()
{
			
	switch (StandUP_state_L) 
	{
		case STATE_stretchleg:
        // printf("当前状态: 伸腿态\n");

            if (CHASSIS.fdb.leg[0].rod.L0>0.28f) 
			{
                // printf("条件1满足，进入后甩腿\n");
                StandUP_state_L = STATE_Backleg;
            }
        break;
								
		case STATE_Backleg:
		// printf("当前状态: 后甩腿态\n");
			if ((fabs(CHASSIS.fdb.leg[0].rod.Phi0-0.5)<0.1f))
			{
				// printf("条件满足，左腿进入正常态");
				StandUP_state_L = STATE_COMPLETE;
			}
		break;

		case STATE_COMPLETE:
			// printf("当前状态: 完成态\n");
			if (CHASSIS.mode==CHASSIS_OFF_HOOK&&CHASSIS.last_mode!=CHASSIS_OFF_HOOK)
			{
				// printf("  ");
				StandUP_state_L = STATE_stretchleg;
			}
		break;
		
		default: 
		{
		StandUP_state_L=STATE_NORMAL;
		}
	}
switch (StandUP_state_R) 
{
    case STATE_stretchleg:
        // printf("当前状态: 伸腿态\n");
        if (CHASSIS.fdb.leg[1].rod.L0 > 0.28) 
		{
            // printf("条件1满足，进入后甩腿\n");
            StandUP_state_R = STATE_Backleg;
        }
    break;
                                
    case STATE_Backleg:
        // printf("当前状态: 后甩腿态\n");
        if ((fabs(CHASSIS.fdb.leg[1].rod.Phi0 - 0.5) < 0.1))
		{
            // printf("条件满足，左腿进入正常态");
            StandUP_state_R = STATE_COMPLETE;
        }
	break;
    case STATE_COMPLETE:
        // printf("当前状态: 完成态\n");
        if (CHASSIS.mode == CHASSIS_OFF_HOOK && CHASSIS.last_mode != CHASSIS_OFF_HOOK) 
		{
            // printf("条件满足，左腿进入正常态");
            StandUP_state_R = STATE_stretchleg;
        }
        break;
        
    default: 
        {
            StandUP_state_R = STATE_NORMAL;  // 修正：应该是 StandUP_state_R 而不是 L
        }
}				
}

