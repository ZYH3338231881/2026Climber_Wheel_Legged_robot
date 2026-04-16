/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "chassis_task.h"
#include "stdio.h"
#include "usart.h"
#include "CAN_receive.h"
#include "CAN_cmd_damiao.h"
#include "fdcan.h"
extern Chassis_s CHASSIS ;

extern float ddot_z_M ;
extern float l0 ;
extern float v_l0;
extern float theta;
extern float w_theta ;

extern float dot_v_l0 ;
extern float dot_w_theta ;
extern float ddot_z_w ;
extern float F_ff;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId Chassis_TaskHandle;
osThreadId IMU_taskHandle;
osThreadId State_checkHandle;
osThreadId MY_UI_TaskHandle;
osThreadId Talk_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
extern void chassis_task(void const * argument);
void imu_task(void const * argument);
void State_check_task(void const * argument);
void UI_Task(void const * argument);
void Talk_Task_start(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Chassis_Task */
  osThreadDef(Chassis_Task, chassis_task, osPriorityAboveNormal, 0, 512);
  Chassis_TaskHandle = osThreadCreate(osThread(Chassis_Task), NULL);

  /* definition and creation of IMU_task */
  osThreadDef(IMU_task, imu_task, osPriorityRealtime, 0, 1024);
  IMU_taskHandle = osThreadCreate(osThread(IMU_task), NULL);

  /* definition and creation of State_check */
  osThreadDef(State_check, State_check_task, osPriorityNormal, 0, 128);
  State_checkHandle = osThreadCreate(osThread(State_check), NULL);

  /* definition and creation of MY_UI_Task */
  osThreadDef(MY_UI_Task, UI_Task, osPriorityNormal, 0, 512);
  MY_UI_TaskHandle = osThreadCreate(osThread(MY_UI_Task), NULL);

  /* definition and creation of Talk_Task */
  osThreadDef(Talk_Task, Talk_Task_start, osPriorityNormal, 0, 128);
  Talk_TaskHandle = osThreadCreate(osThread(Talk_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @ retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
//    float M3508_velocity_pid[3] = {150, 0, 0};
//    PID_init(&CHASSIS.pid.wheel_stop[0], PID_POSITION,M3508_velocity_pid, 16384,1000);
//		PID_init(&CHASSIS.pid.wheel_stop[1], PID_POSITION,M3508_velocity_pid, 16384,1000);


  /* Infinite loop */
  for(;;)
  {
		
//      CHASSIS.wheel_motor[0].set.curr=PID_calc(&CHASSIS.pid.wheel_stop[0], CHASSIS.wheel_motor[0].fdb.vel, 400); 
//		 CHASSIS.wheel_motor[1].set.curr= PID_calc(&CHASSIS.pid.wheel_stop[1], CHASSIS.wheel_motor[1].fdb.vel, 400);
//	      CanCmdDjiMotor(&hfdcan2,0x200,CHASSIS.wheel_motor[1].set.curr,CHASSIS.wheel_motor[0].set.curr,0,0);

//printf_DMA("%f,%f\n",CHASSIS.fdb.leg_state[0].x_dot,CHASSIS.fdb.leg_state[0].x);
//printf("%f,%f,%f,%f\n",CHASSIS.wheel_motor[0].set.tor,CHASSIS.fdb.leg_state[0].x_dot,CHASSIS.wheel_motor[1].set.tor,CHASSIS.fdb.leg_state[1].x_dot);
//腿长PID
//printf_DMA("%f,%f\n",CHASSIS.pid.stand_up.fdb,CHASSIS.pid.stand_up.set);	
//printf("%f,%f\n",CHASSIS.fdb.leg_state[0].theta,CHASSIS.ref.leg_state[0].theta);	 
//YAW轴速度环调试
//printf_DMA("%f,%f\n",CHASSIS.fdb.body.yaw_dot,CHASSIS.ref.speed_vector.wz);
//防劈叉以及滤波后数据比较
//printf_DMA("%f,%f,%f\n",CHASSIS.fdb.two_leg_err,CHASSIS.pid.leg_tp.fdb,CHASSIS.pid.leg_tp.set);		
//x[2] = x2_OFFSET + (CHASSIS.fdb.leg_state[i].x         - CHASSIS.ref.leg_state[i].x);
//printf("%f,%f\n",CHASSIS.fdb.leg_state[0].x,CHASSIS.ref.leg_state[0].x);
//离地情况检测
// printf_DMA("%d,%d\n",CHASSIS.fdb.leg[0].is_take_off,CHASSIS.fdb.leg[1].is_take_off);
//地面支持力情况检测
//printf_DMA("%f,%f,%d\n", CHASSIS.fdb.leg[0].Fn,ddot_z_w,CHASSIS.fdb.leg[0].is_take_off);
//  离地瞬间轮毂情况
//printf_DMA("%f,%d\n",CHASSIS.cmd.leg[0].wheel.T,CHASSIS.fdb.leg[0].is_take_off);
//关节反馈解得摆杆支持力  F  竖直方向
//printf_DMA("%f,%f\n",CHASSIS.fdb.leg[0].rod.F_costheta,CHASSIS.fdb.leg[1].rod.F_costheta);
//关节扭力在竖直方向的分量
//printf_DMA("%f,%f\n",CHASSIS.fdb.leg[0].rod.Tp_sintheta,CHASSIS.fdb.leg[1].rod.Tp_sintheta);
// 检查世界坐标系运动情况
//printf_DMA("%f,%f,%f\n",CHASSIS.fdb.world.x_accel,CHASSIS.fdb.world.y_accel,CHASSIS.fdb.world.z_accel);
//检查两腿的F
//printf_DMA("%f,%f\n",CHASSIS.fdb.leg[0].rod.Tp_sintheta,CHASSIS.fdb.leg[1].rod.Tp_sintheta);
//printf_DMA("%f,%f\n",CHASSIS.fdb.leg[0].rod.F_costheta,CHASSIS.fdb.leg[1].rod.F_costheta);
//pitch轴协调控制
//printf_DMA("%f,%f\n",CHASSIS.pid.pitch_angle.fdb,CHASSIS.pid.pitch_angle.set);
//phi0情况和phi0_dot
//printf_DMA("%f,%f\n",CHASSIS.fdb.body.pitch,CHASSIS.fdb.body.phi_dot);
//DM8009  峰值扭矩应该是40NM   3508减速箱  4.9NM
printf_DMA("%f\n",CHASSIS.cmd.leg[0].rod.F);
//轮毂输出力矩
//printf_DMA("%f,%f,%f,%f,%f,%f\n",CHASSIS.lqr_out.wheel_theta,CHASSIS.lqr_out.wheel_theta_dot,CHASSIS.lqr_out.wheel_x,CHASSIS.lqr_out.wheel_vel,CHASSIS.lqr_out.wheel_phi,CHASSIS.lqr_out.wheel_phi_dot);
//printf_DMA("%f,%f\n",CHASSIS.lqr_out.wheel_theta,CHASSIS.lqr_out.wheel_theta_dot);
//printf_DMA("%f,%f\n",CHASSIS.lqr_out.wheel_phi,CHASSIS.lqr_out.wheel_phi_dot);
//关节输出力矩
//printf_DMA("%f,%f,%f\n",CHASSIS.lqr_out.joint_theta,CHASSIS.lqr_out.wheel_theta,CHASSIS.fdb.leg[0].rod.Theta);
//printf_DMA("%f\n",  CHASSIS.fdb.body.x );
//printf_DMA("%f\n",CHASSIS.wheel_motor[0].set.tor);
//printf_DMA("%d,%d\n",CHASSIS.joint_motor[0].fdb.state,CHASSIS.joint_motor[1].fdb.state);
//printf_DMA("%f,%f\n",CHASSIS.joint_motor[0].fdb.tor,CHASSIS.joint_motor[1].fdb.tor);
//printf_DMA("%f,%f\n",CHASSIS.pid.leg_length_length[0].fdb,CHASSIS.pid.leg_length_length[0].set);
//检测轮毂电流大小
//printf_DMA("%f,%f\n",CHASSIS.wheel_motor[0].set.curr,CHASSIS.wheel_motor[1].set.curr);
//加速度情况检测
//printf_DMA("%f,%f\n",CHASSIS.fdb.body.y_accel,CHASSIS.fdb.world.x_accel);
//腿长变换情况检测
//printf_DMA("%f,%f,%f\n",CHASSIS.fdb.leg[1].rod.dL0,CHASSIS.fdb.leg[1].rod.ddL0,CHASSIS.lpf.leg_l0_accel_filter->out);
// printf_DMA("%f\n",ddot_z_M );//失重一瞬间会突然变成一个负值
// printf_DMA("%f\n",dot_v_l0 * cosf(theta) );//突然失重会变成正值
//printf_DMA("%f\n",ddot_z_w);
//跳跃参数查看
//printf_DMA("%f,%f\n",CHASSIS.cmd.leg[0].rod.F,CHASSIS.cmd.leg[1].rod.F);
//printf_DMA("%d,%f,%f\n",CHASSIS.step,CHASSIS.fdb.leg[0].rod.L0,CHASSIS.ref.rod_L0[0]);
//printf_DMA("%f,%f\n",CHASSIS.fdb.leg[0].rod.L0,CHASSIS.fdb.leg[1].rod.L0);
//左腿追右腿PID参数
//printf_DMA("%f,%f\n",CHASSIS.fdb.leg[0].rod.Theta,CHASSIS.fdb.leg[1].rod.Theta);
//虚拟摆杆和轮子扭矩参数
//printf_DMA("%f,%f\n",CHASSIS.cmd.leg[1].rod.Tp,CHASSIS.cmd.leg[0].rod.Tp);

//printf_DMA("%f\n",F_ff);
//printf_DMA("%f,%f\n",CHASSIS.fdb.leg_state[0].x_dot,CHASSIS.ref.leg_state[0].x_dot);
//printf_DMA("%d\n",CHASSIS.step);
//printf_DMA("%f,%f\n",CHASSIS.fdb.body.phi,CHASSIS.fdb.leg[0].rod.Theta);
//printf_DMA("%f,%f\n",CHASSIS.joint_motor[0].fdb.tor,CHASSIS.joint_motor[0].set.tor);


    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_imu_task */
/**
* @brief Function implementing the IMU_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu_task */
__weak void imu_task(void const * argument)
{
  /* USER CODE BEGIN imu_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END imu_task */
}

/* USER CODE BEGIN Header_State_check_task */
/**
* @brief Function implementing the State_check thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_State_check_task */
__weak void State_check_task(void const * argument)
{
  /* USER CODE BEGIN State_check_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END State_check_task */
}

/* USER CODE BEGIN Header_UI_Task */
/**
* @brief Function implementing the MY_UI_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UI_Task */
__weak void UI_Task(void const * argument)
{
  /* USER CODE BEGIN UI_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UI_Task */
}

/* USER CODE BEGIN Header_Talk_Task_start */
/**
* @brief Function implementing the Talk_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Talk_Task_start */
__weak void Talk_Task_start(void const * argument)
{
  /* USER CODE BEGIN Talk_Task_start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Talk_Task_start */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
