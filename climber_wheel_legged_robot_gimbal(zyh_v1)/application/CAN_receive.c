/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "main.h"
#include "cmsis_os.h"
#include "gimbal_behaviour.h"
#include "string.h"
typedef CAN_HandleTypeDef hcan_t;
// 接收数据
static DjiMotorMeasure_t CAN1_DJI_MEASURE[11];
static DjiMotorMeasure_t CAN2_DJI_MEASURE[11];
extern Gimbal_s gimbal_direct;
typedef struct __CanCtrlData
{
    hcan_t * hcan;
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
} CanCtrlData_s;



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
        if (can == 1) {
            return &CAN1_DJI_MEASURE[i];
        } else if (can == 2) {
            return &CAN2_DJI_MEASURE[i];
        }
    }
    return &CAN1_DJI_MEASURE[1];
}
//motor data read
static void GetDjiFdbData(Motor_s *p_motor, const DjiMotorMeasure_t * p_dji_motor_measure)                                    \
{
	  p_motor->fdb.vel = p_dji_motor_measure->speed_rpm * RPM_TO_OMEGA;
    p_motor->fdb.pos = p_dji_motor_measure->ecd * 2 * M_PI / 8192 - M_PI;  //[-pi~pi]
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

void GetMotorMeasure(Motor_s * p_motor)
{
    switch (p_motor->type) {
        case DJI_M2006:
        case DJI_M3508: {
            const DjiMotorMeasure_t * p_dji_motor_measure =GetDjiMotorMeasurePoint(p_motor->can, p_motor->id - 1);
            GetDjiFdbData(p_motor, p_dji_motor_measure);
        } break;
        case DJI_M6020: {
            const DjiMotorMeasure_t * p_dji_motor_measure =GetDjiMotorMeasurePoint(p_motor->can, p_motor->id + 3);
						GetDjiFdbData(p_motor, p_dji_motor_measure);
        } break;

        default:
            break;
    }
}


/**
 * @brief          若接收到的数据标识符为StdId则对应解码
 * @note           解码数据包括DJI电机数据与板间通信数据
 * @param[in]      CAN CAN口(CAN_1或CAN_2)
 * @param[in]      rx_header CAN接收数据头
 * @param[in]      rx_data CAN接收数据
 */
static void DecodeStdIdData(hcan_t * CAN, CAN_RxHeaderTypeDef * rx_header, uint8_t rx_data[8])
{
    switch (rx_header->StdId) {  //电机解码
        case DJI_M1_ID:
        case DJI_M2_ID:
        case DJI_M3_ID:
        case DJI_M4_ID:
        case DJI_M5_ID:
        case DJI_M6_ID:
        case DJI_M7_ID:
        case DJI_M8_ID:
        case DJI_M9_ID:
        case DJI_M10_ID:
        case DJI_M11_ID: {  // 以上ID为DJI电机标识符
            static uint8_t i = 0;
            i = rx_header->StdId - DJI_M1_ID;
            if (CAN == &hcan1)  // 接收到的数据是通过 CAN1 接收的
            {
                DjiFdbData(&CAN1_DJI_MEASURE[i], rx_data);
            } else if (CAN == &hcan2)  // 接收到的数据是通过 CAN2 接收的
            {
                DjiFdbData(&CAN2_DJI_MEASURE[i], rx_data);
            }
            return;
        }
        default: {
            break;
        }
    }

}
MTOC_message_t mtoc_mesasge;

void M_communication_c(MTOC_message_t *mtoc_mesasge,uint8_t *rx_data)
{
   static uint8_t is_first_msg = 1;
   uint16_t new_power_heat = (uint16_t)(rx_data[0] << 8 | rx_data[1]);
   
   if (is_first_msg)
   {
       mtoc_mesasge->power_heat = new_power_heat;
       is_first_msg = 0;
       return;
   }

   if(new_power_heat > mtoc_mesasge->power_heat)
   {
       mtoc_mesasge->bullet_count += (new_power_heat - mtoc_mesasge->power_heat + 5) / 10;
   }
   mtoc_mesasge->power_heat = new_power_heat;
	 
	 mtoc_mesasge->bullet_speed=(int16_t)(rx_data[2] << 8 | rx_data[3]) / 1000.0f;
}

//gimbal--can1--fifo1  云台接收控制主要有3  两个6020  一个拨盘电机发射
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);

    if (rx_header.IDE == CAN_ID_STD)  // 接收到的数据标识符为StdId
    {
        DecodeStdIdData(hcan, &rx_header, rx_data);
    }
		 if (rx_header.StdId == 0x555)  // 接收到的数据标识符为StdId
    {
		M_communication_c(&mtoc_mesasge,rx_data);
    }
}


//can2--通讯--fifo0
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (rx_header.IDE == CAN_ID_STD)  // 接收到的数据标识符为StdId
    {
        DecodeStdIdData(hcan, &rx_header, rx_data);
    }
		

}


CanCtrlData_s CAN_CTRL_DATA = {
    .tx_header.IDE = CAN_ID_STD,
    .tx_header.RTR = CAN_RTR_DATA,
    .tx_header.DLC = 8,
};

/**
 * @brief          发送控制数据
 * @param[in]      can_handle 选择CAN1或CAN2
 * @param[in]      tx_header  CAN发送数据header
 * @param[in]      tx_data    发送数据
 * @return         none
 */
void CAN_SendTxMessage(CanCtrlData_s * can_ctrl_data)
{
    uint32_t send_mail_box;
    uint8_t cnt = 20;  // 重复检测次数

    uint32_t free_TxMailbox =
        HAL_CAN_GetTxMailboxesFreeLevel(can_ctrl_data->hcan);  // 检测是否有空闲邮箱
    while (free_TxMailbox < 3 && cnt--) {                      // 等待空闲邮箱数达到3
        free_TxMailbox = HAL_CAN_GetTxMailboxesFreeLevel(can_ctrl_data->hcan);
    }
    HAL_CAN_AddTxMessage(
        can_ctrl_data->hcan, &can_ctrl_data->tx_header, can_ctrl_data->tx_data, &send_mail_box);
}
/**
 * @brief          通过CAN控制DJI电机(支持GM3508 GM2006 GM6020)
 * @param[in]      can 发送数据使用的can口(1/2)
 * @param[in]      std_id 发送数据使用的std_id
 * @param[in]      curr_1 电机控制电流(id=1/5)
 * @param[in]      curr_2 电机控制电流(id=2/6)
 * @param[in]      curr_3 电机控制电流(id=3/7)
 * @param[in]      curr_4 电机控制电流(id=4/8)
 * @return         none
 * @note           
 */
void CanCmdDjiMotor(uint8_t can, uint16_t std_id, int16_t curr_1, int16_t curr_2, int16_t curr_3, int16_t curr_4)
{
    hcan_t * hcan = NULL;
    if (can == 1)
        hcan = &hcan1;
    else if (can == 2)
        hcan = &hcan2;
    if (hcan == NULL) return;

    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = std_id;
    CAN_CTRL_DATA.tx_data[0] = (curr_1 >> 8);
    CAN_CTRL_DATA.tx_data[1] = curr_1;
    CAN_CTRL_DATA.tx_data[2] = (curr_2 >> 8);
    CAN_CTRL_DATA.tx_data[3] = curr_2;
    CAN_CTRL_DATA.tx_data[4] = (curr_3 >> 8);
    CAN_CTRL_DATA.tx_data[5] = curr_3;
    CAN_CTRL_DATA.tx_data[6] = (curr_4 >> 8);
    CAN_CTRL_DATA.tx_data[7] = curr_4;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
 * @brief          发送数据
 * @param[in]      hcan CAN句柄
 * @param[in]      std_id 数据包ID
 * @param[in]      data 包含8个字节的数据的指针
 * @retval         none
 */
void SendData(uint8_t can,uint16_t std_id,uint8_t *data)
{
	if (can == 1)
        CAN_CTRL_DATA.hcan = &hcan1;
    else if (can == 2)
        CAN_CTRL_DATA.hcan = &hcan2;
    else
        return;
    CAN_CTRL_DATA.tx_header.StdId = std_id;

    memcpy(CAN_CTRL_DATA.tx_data, data, 8);

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}
//C板发送给妙板

void CToM_sendControl(uint8_t can, uint16_t std_id, int16_t yaw)
{
	 
    hcan_t * hcan = NULL;
    if (can == 1)
        hcan = &hcan1;
    else if (can == 2)
        hcan = &hcan2;
    if (hcan == NULL) return;
                                                
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = std_id;
    CAN_CTRL_DATA.tx_data[0] = yaw>>8;
    CAN_CTRL_DATA.tx_data[1] = yaw;
    CAN_CTRL_DATA.tx_data[2] = 0;
    CAN_CTRL_DATA.tx_data[3] = 0;
    CAN_CTRL_DATA.tx_data[4] = 0;
    CAN_CTRL_DATA.tx_data[5] = 0;
    CAN_CTRL_DATA.tx_data[6] = 0;
    CAN_CTRL_DATA.tx_data[7] = 0;	
    CAN_SendTxMessage(&CAN_CTRL_DATA);
}



