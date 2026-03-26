#include "chassis_task.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "string.h"
#include "bsp_delay.h"
#include "stdio.h"
#include "State_chexk_task.h"
extern Chassis_s CHASSIS ;
extern void check_loss_control();//疯车检测
extern void init_loss_control_detector(void);

LossControlDetector leg_loss_control[2]; // 左右腿失控检测器


//主要用于各种状态机的检测
void State_check_task(void const * pvParameters)
{

    init_loss_control_detector();

    while (1) {

//    if (CHASSIS.mode==CHASSIS_FREE)
//    {
            check_loss_control();
//    }
        
		
		osDelay(1);
    }
}


void init_loss_control_detector(void)
{
    // 初始化左腿失控检测器
    leg_loss_control[0].max_phi_error = 0.6f;      // 最大phi位置误差阈值
    leg_loss_control[0].max_theta_error = 0.9f;    // 最大theta角度误差阈值
    leg_loss_control[0].overturn_phi_threshold = 1.2f; // 翻车phi角度阈值
    leg_loss_control[0].confirm_threshold = 600;   // 确认阈值  
    leg_loss_control[0].detect_count = 0;
    leg_loss_control[0].state = LOSS_CONTROL_NORMAL;
    
    // 初始化右腿失控检测器
    leg_loss_control[1].max_phi_error = 0.6f;      // 最大phi位置误差阈值
    leg_loss_control[1].max_theta_error = 0.9f;    // 最大theta角度误差阈值
    leg_loss_control[1].overturn_phi_threshold = 1.2f; // 翻车phi角度阈值
    leg_loss_control[1].confirm_threshold = 600;   // 确认阈值
    leg_loss_control[1].detect_count = 0;
    leg_loss_control[1].state = LOSS_CONTROL_NORMAL;
}
void check_loss_control()
{
      for (int i = 0; i < 2; i++) // 检查左右腿
    {
        LossControlDetector *detector = &leg_loss_control[i];
        
        // 计算phi位置误差和theta角度误差
        float phi_error = fabs(CHASSIS.fdb.body.phi-0.0f);
        float theta_error = fabs(CHASSIS.fdb.leg[i].rod.Theta-0.0f);
        float current_phi = fabs(CHASSIS.fdb.body.phi);
        
        switch (detector->state)
        {
            case LOSS_CONTROL_NORMAL:
                // 检查是否翻车
                if (current_phi > detector->overturn_phi_threshold)
                {
                    detector->state = LOSS_CONTROL_OVERTURN;

                    // 
                }
                // 检查是否失控
                else if (phi_error > detector->max_phi_error || theta_error > detector->max_theta_error)
                {
                    detector->detect_count++;
                    if (detector->detect_count >= detector->confirm_threshold)
                    {
                        detector->state = LOSS_CONTROL_CONFIRMED;

                    }
                }
                else
                {
                    detector->detect_count = 0;
                }
                break;
                
            case LOSS_CONTROL_CONFIRMED:
                // 检查是否恢复正常
                if (phi_error <= detector->max_phi_error && theta_error <= detector->max_theta_error)
                {
                    detector->detect_count--;
                    if (detector->detect_count <= 0)
                    {
                        detector->state = LOSS_CONTROL_NORMAL;
                        detector->detect_count = 0;
                    }
                }
								 else if (current_phi > detector->overturn_phi_threshold)
                {
                    detector->state = LOSS_CONTROL_OVERTURN;

                    // 
                }
                else
                {
                    detector->detect_count = detector->confirm_threshold; // 保持确认状态
                }
                break;
                
            case LOSS_CONTROL_OVERTURN:
                // 检查是否恢复正常
                if (current_phi <= detector->overturn_phi_threshold * 0.8f) // 留有一定余量
                {
                    detector->state = LOSS_CONTROL_NORMAL;
                    detector->detect_count = 0;
                }
                break;
                
            default:
                detector->state = LOSS_CONTROL_NORMAL;
                break;
        }
    }
}    



