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
#include "sign_gengerator.h"
#include "usart.h"
#include "IMU_solve.h"
#include "AutoGimbal.h"
#include "shoot_fric_trigger.h"
#define GIMBAL_TASK_INIT_TIME 500
//控制周期500HZ
extern visionDataStu_t visionDataStu;
extern Shoot_s SHOOT;
//云台控制所有相关数据
extern Gimbal_s gimbal_direct;
extern void send_nuc();
extern INS_t INS;
extern void vofa_watch();
extern osThreadId gimbalTaskHandle;
void gimbal_task(void const *pvParameters)
{
	//在陀螺仪完成初始化任务后会恢复任务
	vTaskDelay(GIMBAL_TASK_INIT_TIME);
	GimbalInit();
//单环测试
//	gimbal_direct.yaw.set.vel=10;
//	gimbal_direct.pitch.set.vel=10;
    while (1)
    {
			GimbalSetMode();
			GimbalObserver();
			GimbalReference();
			GimbalConsole();
			GimbalSendCmd();
	    send_nuc();
			osDelay(2);
    }
}


	
void send_nuc()
{
    typedef struct __attribute__((packed)) GimbalToVision 
    { 
       uint8_t head[2]; 
       uint8_t mode;   //  0: 空闲, 1: 自瞄, 2: 小符, 3: 大符 
       float q[4];     //  wxyz顺序 
       float yaw; 
       float yaw_vel; 
       float pitch; 
       float pitch_vel; 
       float bullet_speed; 
       uint16_t bullet_count;   //  子弹累计发送次数 
       uint8_t tail[2]; 
    } GimbalToVision_t;

    GimbalToVision_t message;
    
    message.head[0] = 'G';
    message.head[1] = 'V'; 
    
    if (keyboard_data.Remote_Mouse_KeyR) {
        message.mode = 1; 
    } else {
        message.mode = 0; 
    }
    
    message.q[0] = INS.q[0];
    message.q[1] = INS.q[1];
    message.q[2] = INS.q[2];
    message.q[3] = INS.q[3];
    
    message.yaw = gimbal_direct.feedback_pos.yaw; 
    message.yaw_vel = gimbal_direct.feedback_vel.yaw; 
    
    message.pitch = gimbal_direct.feedback_pos.pitch;
    message.pitch_vel = gimbal_direct.feedback_vel.pitch;
    
    message.bullet_speed = mtoc_mesasge.bullet_speed; 
    message.bullet_count = mtoc_mesasge.bullet_count; 

    message.tail[0] = 'E';
    message.tail[1] = 'N';
    
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&message, sizeof(message));
}


