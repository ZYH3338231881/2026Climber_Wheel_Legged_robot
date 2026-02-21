#include "judge_paint_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "judge_send_app.h"
#include "computer_rec.h"
#include "math.h"
#include "CAN_receive.h"
#include "ui_interface.h"
#include "ui_frame1.h"
#include "ui.h"
#include "task.h"
#include "usart.h"
void UI_Task(void const * argument)
{
  /* USER CODE BEGIN UI_Task */

  /* Infinite loop */
  for(;;)
  {
		ui_init_frame1_Ungroup();
		osDelay(1);

  }
  /* USER CODE END UI_Task */
}
//		#define SEND_MESSAGE(message, length)	HAL_UART_Transmit_DMA(&huart7,message,length);osDelay(30)

