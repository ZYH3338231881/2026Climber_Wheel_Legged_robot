#include "cmsis_os.h"
#include "Music_task.h"

void Music_task_start(void const * argument)
{
  /* USER CODE BEGIN Music_task_start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Music_task_start */
}