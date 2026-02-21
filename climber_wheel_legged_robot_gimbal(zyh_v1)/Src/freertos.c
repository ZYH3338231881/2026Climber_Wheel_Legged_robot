/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "stdio.h"
#include "usart.h"
#include <stdarg.h>

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 1);
    return ch;
}
volatile uint8_t  usart_dma_tx_over = 1;
int printf_DMA(const char *format,...)
{
  va_list arg;
  static char SendBuff[600] = {0};
  int rv;
  while(!usart_dma_tx_over);//�ȴ�ǰһ��DMA�������
 
  va_start(arg,format);
  rv = vsnprintf((char*)SendBuff,sizeof(SendBuff)+1,(char*)format,arg);
  va_end(arg);
	
 
  HAL_UART_Transmit_DMA(&huart1,(uint8_t *)SendBuff,rv);
  usart_dma_tx_over = 0;//��0ȫ�ֱ�־��������ɺ�������1
 
  return rv;
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
  		 usart_dma_tx_over = 1;
	}
}


void vofa_watch()
{
//	      printf_DMA(" %f\n",gimbal_direct.y);
//	      printf_DMA("%f,%f\n",gimbal_direct.feedback_vel.pitch,gimbal_direct.pitch.set.vel);  //pitch�����ٶȻ�
//		    printf_DMA("%f,%f\n",gimbal_direct.feedback_pos.pitch,gimbal_direct.pitch.set.pos);  //pitch����pid
//				printf_DMA("%f,%f,%f,%f\n",gimbal_direct.feedback_pos.pitch,gimbal_direct.pitch.set.pos,gimbal_direct.feedback_vel.pitch,gimbal_direct.pitch.set.vel);
//	      printf_DMA("%f,%f\n",gimbal_direct.feedback_vel.yaw,gimbal_direct. yaw.set.vel); // yaw�����ٶȻ�
//		    printf("%f,%f\n",gimbal_direct.feedback_pos.yaw,gimbal_direct.yaw.set.pos); // yaw����pid
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId testHandle;
osThreadId imuTaskHandle;
osThreadId gimbalTaskHandle;
osThreadId Remote_rec_taskHandle;
osThreadId musicTaskHandle;
osThreadId shootTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//osThreadId calibrate_tast_handle;

/* USER CODE END FunctionPrototypes */

void test_task(void const * argument);
extern void IMU_task(void const * argument);
void gimbal_task(void const * argument);
extern void remote_rec_task(void const * argument);
extern void music_task(void const * argument);
extern void shoot_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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
  /* definition and creation of test */
  osThreadDef(test, test_task, osPriorityNormal, 0, 128);
  testHandle = osThreadCreate(osThread(test), NULL);

  /* definition and creation of imuTask */
  osThreadDef(imuTask, IMU_task, osPriorityRealtime, 0, 1024);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

  /* definition and creation of gimbalTask */
  osThreadDef(gimbalTask, gimbal_task, osPriorityNormal, 0, 512);
  gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);

  /* definition and creation of Remote_rec_task */
  osThreadDef(Remote_rec_task, remote_rec_task, osPriorityAboveNormal, 0, 128);
  Remote_rec_taskHandle = osThreadCreate(osThread(Remote_rec_task), NULL);

  /* definition and creation of musicTask */
  osThreadDef(musicTask, music_task, osPriorityBelowNormal, 0, 256);
  musicTaskHandle = osThreadCreate(osThread(musicTask), NULL);

  /* definition and creation of shootTask */
  osThreadDef(shootTask, shoot_task, osPriorityHigh, 0, 512);
  shootTaskHandle = osThreadCreate(osThread(shootTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
//    osThreadDef(cali, calibrate_task, osPriorityNormal, 0, 512);
//    calibrate_tast_handle = osThreadCreate(osThread(cali), NULL);

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_test_task */
/**
  * @brief  Function implementing the test thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_test_task */
void test_task(void const * argument)
{
  /* USER CODE BEGIN test_task */
  /* Infinite loop */
  for(;;)
  {
   

    vofa_watch();



		osDelay(1);
  }
  /* USER CODE END test_task */
}

/* USER CODE BEGIN Header_gimbal_task */
/**
* @brief Function implementing the gimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_task */
__weak void gimbal_task(void const * argument)
{
  /* USER CODE BEGIN gimbal_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbal_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
