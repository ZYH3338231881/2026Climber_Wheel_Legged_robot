/**
  ****************************(C) COPYRIGHT 2025 PolarBear****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             主要利用陀螺仪bmi088，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *  V3.0.0     Apr-05-2025     Penguin         1. 采用王工开源的陀螺仪EKF解算
  *       
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 PolarBear****************************
*/

#include "IMU_task.h"

#include "IMU.h"
#include "IMU_solve.h"
#include "ahrs.h"
#include "BMI088driver.h"
#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "cmsis_os.h"
#include "ist8310driver.h"
#include "main.h"
#include "math.h"
#include "pid.h"
#include "sign_gengerator.h"
#include "gimbal_behaviour.h"

// clang-format off
#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm给定


#define __HEAT_IMU 1  // 加热IMU(防止Debug时因断点导致pid失效产生过热，烧坏IMU)
#define __IMU_CONTROL_TEMPERATURE 35 // (度)IMU目标控制温度

#define IMU_CALI_MAX_COUNT 100

#define RAW_GYRO_X_ADDRESS_OFFSET 1
#define RAW_GYRO_Y_ADDRESS_OFFSET 0
#define RAW_GYRO_Z_ADDRESS_OFFSET 2

#define RAW_GYRO_X_DIRECTION (1)
#define RAW_GYRO_Y_DIRECTION (-1)
#define RAW_GYRO_Z_DIRECTION (1)

#define RAW_ACCEL_X_ADDRESS_OFFSET 1
#define RAW_ACCEL_Y_ADDRESS_OFFSET 0
#define RAW_ACCEL_Z_ADDRESS_OFFSET 2

#define RAW_ACCEL_X_DIRECTION (1)
#define RAW_ACCEL_Y_DIRECTION (-1)
#define RAW_ACCEL_Z_DIRECTION (1)
		
    

extern Gimbal_s gimbal_direct;


static void imu_temp_control(fp32 temp);

static void imu_cmd_spi_dma(void);

static void imu_rotate(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);

static void board_rotate(fp32 gyro[3], fp32 accel[3]);

static void UpdateImuData(void);

extern SPI_HandleTypeDef hspi1;


static TaskHandle_t INS_task_local_handler;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};


uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};



volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;

#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f} \

bmi088_real_data_t bmi088_real_data;
fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 gyro_cali_offset[3];

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
fp32 accel_cali_offset[3];

ist8310_real_data_t ist8310_real_data;
fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];
fp32 mag_cali_offset[3];

static uint8_t first_temperate;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
pid_type_def imu_temp_pid;

static const float timing_time = 0.001f;   //tast run time , unit s.任务运行的时间 单位 s





 fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
 fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
 fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
 fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.欧拉角 单位 rad
fp32 INS_angle_last[3] = {0.0f, 0.0f, 0.0f};
// clang-format on

Imu_t IMU_DATA = {0.0f};

#define __BOARD_INSTALL_SPIN_MATRIX    \
{1.0f, 0.0f, 0.0f},                     \
{0.0f, 1.0f, 0.0f},                     \
{0.0f, 0.0f, 1.0f} \

static fp32 board_rotate_matrix[3][3] = {__BOARD_INSTALL_SPIN_MATRIX};

/**
  * @brief          imu任务, 初始化 bmi088, ist8310, 计算欧拉角
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void IMU_task(void const * pvParameters)
{
    // 发布IMU数据

    // clang-format off
    //wait a time
    osDelay(INS_TASK_INIT_TIME);
    while(BMI088_init())
    {
        osDelay(100);
    }
    while(ist8310_init())
    {
        osDelay(100);
    }

    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    // rotate
    imu_rotate(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
    board_rotate(INS_gyro, INS_accel);

    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
//    AHRS_init(INS_quat, INS_accel, INS_mag);

    //获取当前任务的任务句柄，
    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

    //set spi frequency
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }


    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1;

    gEstimateKF_Init(1, 2000);
    IMU_QuaternionEKF_Init(10, 0.001, 1000000, 0.9996);

    while (1)
    {
        //wait spi DMA tansmit done
        //等待SPI DMA传输
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }


        if(gyro_update_flag & (1 << IMU_NOTIFY_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_NOTIFY_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
        }

        if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);

        }

        if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }
        
        // rotate
        imu_rotate(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
        board_rotate(INS_gyro, INS_accel);

        // 更新加速度
        gEstimateKF_Update(INS_gyro[0],  INS_gyro[1],  INS_gyro[2],
                           INS_accel[0], INS_accel[1], INS_accel[2],
                           timing_time);

        // 更新欧拉角
        IMU_QuaternionEKF_Update(INS_gyro[0], INS_gyro[1], INS_gyro[2],
                                 gVec[0], gVec[1], gVec[2],
                                 timing_time);
        // clang-format on

        UpdateImuData();
				

				
    }
}

static void UpdateImuData(void)
{
	  IMU_DATA.gyro[AX_X] = INS_gyro[AX_X];
    IMU_DATA.gyro[AX_Y] = INS_gyro[AX_Y];
    IMU_DATA.gyro[AX_Z] = INS_gyro[AX_Z];
	
    IMU_DATA.angle[AX_X] = INS.angle[AX_X];
    IMU_DATA.angle[AX_Y] = INS.angle[AX_Y];
    IMU_DATA.angle[AX_Z] = INS.angle[AX_Z];

    IMU_DATA.accel[AX_X] = gVec[AX_X];
    IMU_DATA.accel[AX_Y] = gVec[AX_Y];
    IMU_DATA.accel[AX_Z] = gVec[AX_Z];
}

// clang-format off

/**
 * @brief          旋转陀螺仪,加速度计和磁力计,因为设备有不同安装方式
 * @param[out]     gyro: 旋转
 * @param[out]     accel: 旋转
 * @param[out]     mag: 旋转
 * @param[in]      bmi088: 陀螺仪和加速度计数据
 * @param[in]      ist8310: 磁力计数据
 * @retval         none
 */
static void imu_rotate(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2];
        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2];
    }
}

/**
 * @brief          旋转陀螺仪,加速度计,因为C板有不同安装方式
 * @param[in]      imu: 被旋转的imu值
 * @param[in]      ins: 陀螺仪数据
 * @param[out]     rotate_matrix: 旋转矩阵
 * @retval         none
 */
static void board_rotate(fp32 gyro[3], fp32 accel[3]){
    float tmp_gyro[3];
    float tmp_accel[3];
    for (uint8_t i = 0; i < 3; i++) 
    {
        tmp_gyro[i]  = gyro[0] * board_rotate_matrix[i][0]  + gyro[1] * board_rotate_matrix[i][1]  + gyro[2] * board_rotate_matrix[i][2];
        tmp_accel[i] = accel[0] * board_rotate_matrix[i][0] + accel[1] * board_rotate_matrix[i][1] + accel[2] * board_rotate_matrix[i][2];
    }
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i]  = tmp_gyro[i];
        accel[i] = tmp_accel[i];
    }
}


/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    
    if (!__HEAT_IMU)
    {
        return;
    }
    else if (first_temperate)
    {
        PID_calc(&imu_temp_pid, temp, __IMU_CONTROL_TEMPERATURE);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        //in beginning, max power
        if (temp > __IMU_CONTROL_TEMPERATURE)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}

/**
  * @brief          获取四元数
  * @param[in]      none
  * @retval         INS_quat的指针
  */
const fp32 *get_INS_quat_point  
	




(void)
{
    return INS_quat;
}

/**
  * @brief          获取欧拉角, 0:yaw, 1:pitch, 2:roll 单位 rad
  * @param[in]      none
  * @retval         INS_angle的指针
  */
const fp32 *get_INS_angle_point(void)
{
    return INS_angle;
}

/**
  * @brief          获取角速度,0:x轴, 1:y轴, 2:roll轴 单位 rad/s
  * @param[in]      none
  * @retval         INS_gyro的指针
  */
extern const fp32 *get_gyro_data_point(void)
{
    return INS_gyro;
}

/**
  * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 m/s2
  * @param[in]      none
  * @retval         INS_accel的指针
  */
extern const fp32 *get_accel_data_point(void)
{
    return INS_accel;
}

/**
  * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 ut
  * @param[in]      none
  * @retval         INS_mag的指针
  */
extern const fp32 *get_mag_data_point(void)
{
    return INS_mag;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == INT1_GYRO_Pin)
    {
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == DRDY_IST8310_Pin)
    {
        mag_update_flag |= 1 << IMU_DR_SHFITS;
    }
    else if(GPIO_Pin == GPIO_PIN_0)
    {

        //wake up the task
        //唤醒任务
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

    }


}

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_cmd_spi_dma(void)
{
    UBaseType_t uxSavedInterruptStatus;
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    //开启陀螺仪的DMA传输
    if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
    && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    //开启加速度计的DMA传输
    if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
    && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    



    if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
    && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}


void DMA2_Stream2_IRQHandler(void)
{

    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        //陀螺仪读取完毕
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
            
        }

        //accel read over
        //加速度计读取完毕
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        //温度读取完毕
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        
        imu_cmd_spi_dma();

        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}
// clang-format on

/******************************************************************/
/* API                                                            */
/*----------------------------------------------------------------*/
/* function:      GetImuAngle                                     */
/*                GetImuVelocity                                  */
/*                GetImuAccel                                     */
/*                GetYawBias                                      */
/******************************************************************/

/**
  * @brief          获取欧拉角
  * @param[in]      axis:轴id，可配合定义好的轴id宏使用
  * @retval         (rad) axis轴的角度值
  */
inline float GetImuAngle(uint8_t axis) { return IMU_DATA.angle[axis]; }
/**
  * @brief          获取角速度
  * @param[in]      axis:轴id，可配合定义好的轴id宏使用
  * @retval         (rad/s) axis轴的角速度
  */
inline float GetImuVelocity(uint8_t axis) { return IMU_DATA.gyro[axis]; }
/**
  * @brief          获取角速度
  * @param[in]      axis:轴id，可配合定义好的轴id宏使用
  * @retval         (m/s^2) axis轴上的加速度
  */
inline float GetImuAccel(uint8_t axis) { return IMU_DATA.accel[axis]; }
/**
  * @brief          获取yaw零飘修正值
  * @retval         (rad/s) yaw零飘修正值
  */
float GetYawBias(void) { return IMU_DATA.gyro[AX_Z]; }

float get_raw_accel(uint8_t axis)
{
    switch (axis) {
        case AX_X: {
            return RAW_ACCEL_X_DIRECTION * bmi088_real_data.accel[RAW_ACCEL_X_ADDRESS_OFFSET];
        }
        case AX_Y: {
            return RAW_ACCEL_Y_DIRECTION * bmi088_real_data.accel[RAW_ACCEL_Y_ADDRESS_OFFSET];
        }
        case AX_Z: {
            return RAW_ACCEL_Z_DIRECTION * bmi088_real_data.accel[RAW_ACCEL_Z_ADDRESS_OFFSET];
        }
        default: {
            return 0.0f;
        }
    }
}
float get_raw_gyro(uint8_t axis)
{
    switch (axis) {
        case AX_X: {
            return RAW_GYRO_X_DIRECTION * bmi088_real_data.gyro[RAW_GYRO_X_ADDRESS_OFFSET];
        }
        case AX_Y: {
            return RAW_GYRO_Y_DIRECTION * bmi088_real_data.gyro[RAW_GYRO_Y_ADDRESS_OFFSET];
        }
        case AX_Z: {
            return RAW_GYRO_Z_DIRECTION * bmi088_real_data.gyro[RAW_GYRO_Z_ADDRESS_OFFSET];
        }
        default: {
            return 0.0f;
        }
    }
}
/*------------------------------ End of File ------------------------------*/