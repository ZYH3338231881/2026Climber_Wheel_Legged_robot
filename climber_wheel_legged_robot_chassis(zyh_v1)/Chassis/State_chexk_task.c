#include "chassis_task.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "string.h"
#include "bsp_delay.h"
#include "stdio.h"
#include "State_chexk_task.h"
extern Chassis_s CHASSIS ;
extern void standup_STATE();//腿部姿态调整
extern void check_loss_control();//疯车检测
extern void init_loss_control_detector(void);
State StandUP_state_L=STATE_NORMAL;   
State StandUP_state_R=STATE_NORMAL;   
LossControlDetector leg_loss_control[2]; // 左右腿失控检测器


//主要用于各种状态机的检测
void State_check_task(void const * pvParameters)
{
    StandUP_state_L=STATE_stretchleg; 
    StandUP_state_R=STATE_stretchleg;   
    init_loss_control_detector();

    while (1) {
		if (CHASSIS.mode==CHASSIS_OFF_HOOK)
		{
			standup_STATE();
		}
        if (CHASSIS.mode==CHASSIS_FREE)
        {
            check_loss_control();
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
            StandUP_state_R = STATE_NORMAL;  
        }
}
}

void init_loss_control_detector(void)
{
    // 初始化左腿失控检测器
    leg_loss_control[0].max_phi_error = 0.1f;      // 最大phi位置误差阈值
    leg_loss_control[0].max_theta_error = 0.2f;    // 最大theta角度误差阈值
    leg_loss_control[0].overturn_phi_threshold = 1.5f; // 翻车phi角度阈值
    leg_loss_control[0].confirm_threshold = 100;   // 确认阈值
    leg_loss_control[0].detect_count = 0;
    leg_loss_control[0].state = LOSS_CONTROL_NORMAL;
    
    // 初始化右腿失控检测器
    leg_loss_control[1].max_phi_error = 0.1f;      // 最大phi位置误差阈值
    leg_loss_control[1].max_theta_error = 0.2f;    // 最大theta角度误差阈值
    leg_loss_control[1].overturn_phi_threshold = 1.5f; // 翻车phi角度阈值
    leg_loss_control[1].confirm_threshold = 100;   // 确认阈值
    leg_loss_control[1].detect_count = 0;
    leg_loss_control[1].state = LOSS_CONTROL_NORMAL;
}
void check_loss_control()
{
      for (int i = 0; i < 2; i++) // 检查左右腿
    {
        LossControlDetector *detector = &leg_loss_control[i];
        
        // 计算phi位置误差和theta角度误差//
        float phi_error = fabs(CHASSIS.fdb.leg[i].rod.Phi0-0.0f);
        float theta_error = fabs(CHASSIS.fdb.leg[i].rod.Theta-0.0f);
        float current_phi = fabs(CHASSIS.fdb.leg[i].rod.Phi0);
        
        switch (detector->state)
        {
            case LOSS_CONTROL_NORMAL:
                // 检查是否翻车
                if (current_phi > detector->overturn_phi_threshold)
                {
                    detector->state = LOSS_CONTROL_OVERTURN;
                    printf("Leg %d overturn detected!\n", i);
                    printf("Current phi: %.3f\n", current_phi);
                    // 可以在这里添加翻车后的处理代码
                }
                // 检查是否失控
                else if (phi_error > detector->max_phi_error || theta_error > detector->max_theta_error)
                {
                    detector->detect_count++;
                    if (detector->detect_count >= detector->confirm_threshold)
                    {
                        detector->state = LOSS_CONTROL_CONFIRMED;
                        printf("Leg %d loss of control confirmed!\n", i);
                        printf("Phi error: %.3f, Theta error: %.3f\n", phi_error, theta_error);
                        // 可以在这里添加失控后的处理代码
                    }
                }
                else
                {
                    detector->detect_count = 0;
                }
                break;
                
            case LOSS_CONTROL_CONFIRMED:
                // 保持失控状态，直到手动复位
                break;
                
            case LOSS_CONTROL_OVERTURN:
                // 保持翻车状态，直到手动复位
                break;
                
            default:
                detector->state = LOSS_CONTROL_NORMAL;
                break;
        }
    }
}    
