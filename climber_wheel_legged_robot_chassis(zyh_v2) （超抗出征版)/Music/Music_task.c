#include "cmsis_os.h"
#include "Music_task.h"
#include "bsp_BuzzerSongs.h"
#include <stdbool.h> // ������ bool, true, false
extern TIM_HandleTypeDef htim12;


void Music_task_start(void const * argument)
{
  /* USER CODE BEGIN Music_task_start */
	BSP_Buzzer_Init(9000,1);
  static bool played = false;
  /* Infinite loop */
  for(;;)
  {
    BuzzerSongs_Play_Gala_You();
    osDelay(1);
  } 
  /* USER CODE END Music_task_start */
}  