/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       shoot_task.c/h
  * @brief      完成射击控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "shoot_task.h"

#include "attribute_typedef.h"
#include "cmsis_os.h"
#include "shoot_fric_trigger.h"

#ifndef SHOOT_TASK_INIT_TIME
#define SHOOT_TASK_INIT_TIME 201
#endif  // SHOOT_TASK_INIT_TIME

#ifndef SHOOT_CONTROL_TIME
#define SHOOT_CONTROL_TIME 2
#endif  // SHOOT_CONTROL_TIME

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t shoot_high_water;
#endif

__weak void ShootPublish(void);
__weak void ShootInit(void);
__weak void ShootHandleException(void);
__weak void ShootSetMode(void);
__weak void ShootObserver(void);
__weak void ShootReference(void);
__weak void ShootConsole(void);
__weak void ShootSendCmd(void);

/**
 * @brief          射击任务
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void shoot_task(void const * pvParameters)
{
    ShootPublish();
    // 等待陀螺仪任务更新陀螺仪数据
    vTaskDelay(SHOOT_TASK_INIT_TIME);
    // 射击初始化
    ShootInit();

    while (1) {
        // 更新状态量
        ShootObserver();
        // 处理异常
        ShootHandleException();
        // 设置射击模式
        ShootSetMode();
        // 设置目标量
        ShootReference();
        // 计算控制量
        ShootConsole();
        // 发送控制量
        ShootSendCmd();

        // 系统延时
        vTaskDelay(1);

#if INCLUDE_uxTaskGetStackHighWaterMark
        shoot_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

__weak void ShootPublish(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ShootInit(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ShootHandleException(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ShootSetMode(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ShootObserver(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ShootReference(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ShootConsole(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ShootSendCmd(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
