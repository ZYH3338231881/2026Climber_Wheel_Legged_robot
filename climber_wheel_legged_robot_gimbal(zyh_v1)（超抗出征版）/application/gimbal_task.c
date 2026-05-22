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
//????????500HZ
extern visionDataStu_t visionDataStu;
extern Shoot_s SHOOT;
//??????????????????
extern Gimbal_s gimbal_direct;
extern void send_nuc();
extern INS_t INS;
extern void vofa_watch();
extern osThreadId gimbalTaskHandle;
void gimbal_task(void const *pvParameters)
{
	//???????????????????????????
	vTaskDelay(GIMBAL_TASK_INIT_TIME);
	GimbalInit();
//????????
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
			AutoGimbal_Heartbeat_Check();  // ???????
			osDelay(1);
    }
}
void send_nuc()
{
    typedef struct __attribute__((packed)) GimbalToVision 
    { 
       uint8_t head[2]; 
       uint8_t mode;  // 0: ????, 1: ????, 2: ��??, 3: ??? 
       float yaw; 
       float yaw_vel; 
       float pitch; 
       float pitch_vel; 
       float roll; 
       float bullet_speed; 
       uint16_t bullet_count;  // ???????????? 
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
    
    message.yaw = gimbal_direct.feedback_pos.yaw; 
    message.yaw_vel = gimbal_direct.feedback_vel.yaw; 
    
    message.pitch = -gimbal_direct.feedback_pos.pitch;
    message.pitch_vel = -gimbal_direct.feedback_vel.pitch;
    
    message.roll = gimbal_direct.feedback_pos.roll;
    message.bullet_speed = mtoc_mesasge.bullet_speed; 
    message.bullet_count = mtoc_mesasge.bullet_count; 

    message.tail[0] = 'E';
    message.tail[1] = 'N';
    
    HAL_UART_Transmit(&huart6, (uint8_t*)&message, sizeof(message),100);
}


