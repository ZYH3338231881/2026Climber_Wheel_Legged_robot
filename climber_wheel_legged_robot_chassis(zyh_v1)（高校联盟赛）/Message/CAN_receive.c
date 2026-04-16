#include "CAN_receive.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "stm32h7xx_hal.h"
DmMeasure_s CAN1_DM_MEASURE[DM_NUM];
DjiMotorMeasure_t CAN2_DJI_MEASURE[5]; //3508 两个 6020 一个

/**
 * @brief        DmFdbData: 获取DM电机反馈数据函数
 * @param[out]   dm_measure 达妙电机数据缓存
 * @param[in]    rx_data 指向包含反馈数据的数组指针
 * @note         从接收到的数据中提取DM电机的反馈信息，包括电机ID、状态、位置、速度、扭矩以及相关温度参数
 */
void DmFdbData(DmMeasure_s * dm_measure, uint8_t * rx_data)
{
    dm_measure->id = (rx_data[0]) & 0x0F;
    dm_measure->state = (rx_data[0]) >> 4;
    dm_measure->p_int = (rx_data[1] << 8) | rx_data[2];
    dm_measure->v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    dm_measure->t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    dm_measure->pos = uint_to_float(dm_measure->p_int, DM_P_MIN, DM_P_MAX, 16);  
    dm_measure->vel = uint_to_float(dm_measure->v_int, DM_V_MIN, DM_V_MAX, 12);  
    dm_measure->tor = uint_to_float(dm_measure->t_int, DM_T_MIN, DM_T_MAX, 12);  
    dm_measure->t_mos = (float)(rx_data[6]);
    dm_measure->t_rotor = (float)(rx_data[7]);

    dm_measure->last_fdb_time = HAL_GetTick();
}
/**
 * @brief          获取DM电机反馈数据
 * @param[out]     motor 电机结构体 
 * @param[in]      dm_measure 电机反馈数据缓存区
 * @return         none
 */
void GetDmFdbData(Motor_s *motor, DmMeasure_s *dm_measure)
{
    motor->fdb.pos = dm_measure->pos;
    motor->fdb.vel = dm_measure->vel;
    motor->fdb.tor = dm_measure->tor;
    motor->fdb.temp = dm_measure->t_mos;
    motor->fdb.state = dm_measure->state;

    uint32_t now = HAL_GetTick();
    if (now - dm_measure->last_fdb_time > MOTOR_STABLE_RUNNING_TIME) {
        motor->offline = true;
    } else {
        motor->offline = false;
    }
}
/**
 * @brief        DjiFdbData: 获取DJI电机反馈数据函数
 * @param[out]   dji_measure dji电机数据缓存
 * @param[in]    rx_data 反馈数据
 */
void DjiFdbData(DjiMotorMeasure_t * dji_measure, uint8_t * rx_data)
{
    dji_measure->last_ecd = dji_measure->ecd;
    dji_measure->ecd = (uint16_t)((rx_data)[0] << 8 | (rx_data)[1]);
    dji_measure->speed_rpm = (uint16_t)((rx_data)[2] << 8 | (rx_data)[3]);
    dji_measure->given_current = (uint16_t)((rx_data)[4] << 8 | (rx_data)[5]);
    dji_measure->temperate = (rx_data)[6];

    dji_measure->last_fdb_time = HAL_GetTick();
}

/**
 * @brief          获取DJI电机反馈数据
 * @param[out]     p_motor 电机结构体 
 * @param[in]      p_dji_motor_measure 电机反馈数据缓存区
 * @return         none
 */
void GetDjiFdbData(Motor_s * p_motor, const DjiMotorMeasure_t * p_dji_motor_measure)
{
    p_motor->fdb.vel = p_dji_motor_measure->speed_rpm * RPM_TO_OMEGA;
    p_motor->fdb.pos = p_dji_motor_measure->ecd * 2 * M_PI / 8192 - M_PI;
    p_motor->fdb.temp = p_dji_motor_measure->temperate;
    p_motor->fdb.curr = p_dji_motor_measure->given_current;
    p_motor->fdb.ecd = p_dji_motor_measure->ecd;

    uint32_t now = HAL_GetTick();
    if (now - p_dji_motor_measure->last_fdb_time > MOTOR_STABLE_RUNNING_TIME) {
        p_motor->offline = true;
    } else {
        p_motor->offline = false;
    }
}

/**
 * @brief          获取DJI电机接收数据指针
 * @param[in]      can can口 (1 or 2)
 * @param[in]      i 电机接收数据索引,范围[0,11]
 * @return         DJI_Motor_Measure_Data
 * @note           如果输入值超出范围则返回CAN1_DJI_motor[1]
 */
const DjiMotorMeasure_t * GetDjiMotorMeasurePoint(uint8_t can, uint8_t i)
{
    if (i < 12) {
        if (can == 2) {
            return &CAN2_DJI_MEASURE[i];
        }
    }
    return &CAN2_DJI_MEASURE[1];
}

void GetMotorMeasure(Motor_s * p_motor)
{
	switch (p_motor->type) {
		case DJI_M3508:
		{
			 const DjiMotorMeasure_t *p_dji_motor_measure =  GetDjiMotorMeasurePoint(p_motor->can, p_motor->id - 1);
        GetDjiFdbData(p_motor, p_dji_motor_measure);
		}break;
		case DM_8009:
		{
			if(p_motor->can==1)
			{
			  GetDmFdbData(p_motor, &CAN1_DM_MEASURE[p_motor->id - 1]);
			}
		}break;
		
	}
}


