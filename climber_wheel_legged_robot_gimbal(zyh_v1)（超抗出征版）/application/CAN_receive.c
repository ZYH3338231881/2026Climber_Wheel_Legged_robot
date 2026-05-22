/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ??????CAN?��???????????????????,CAN???????????????????????.
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
#include "AutoGimbal.h"
#include "AutoGimbal.h"
typedef CAN_HandleTypeDef hcan_t;
// ????????
static DjiMotorMeasure_t CAN1_DJI_MEASURE[11];
static DjiMotorMeasure_t CAN2_DJI_MEASURE[11];
static DmMeasure_s CAN1_DM_MEASURE[4];
static DmMeasure_s CAN2_DM_MEASURE[4];

extern Gimbal_s gimbal_direct;
extern visionDataStu_t visionDataStu;

typedef struct __CanCtrlData
{
    hcan_t * hcan;
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
} CanCtrlData_s;

CanCtrlData_s CAN_CTRL_DATA = {
    .tx_header.IDE = CAN_ID_STD,
    .tx_header.RTR = CAN_RTR_DATA,
    .tx_header.DLC = 8,
};
/**
 * @brief          ???DJI??????????????
 * @param[in]      can can?? (1 or 2)
 * @param[in]      i ???????????????,??��[0,11]
 * @return         DJI_Motor_Measure_Data
 * @note           ??????????????��???CAN1_DJI_motor[1]
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
 * @brief        DjiFdbData: ???DJI??????????????
 * @param[out]   dji_measure dji??????????
 * @param[in]    rx_data ????????
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
 * @brief        DmFdbData: ???DM??????????????
 * @param[out]   dm_measure ?????????????
 * @param[in]    rx_data ??????????????????????
 * @note         ?????????????????DM????????????????????ID??????��?��????????????????????
 */
void DmFdbData(DmMeasure_s * dm_measure, uint8_t * rx_data)
{
    dm_measure->id = (rx_data[0]) & 0x0F;
    dm_measure->state = (rx_data[0]) >> 4;
    dm_measure->p_int = (rx_data[1] << 8) | rx_data[2];
    dm_measure->v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    dm_measure->t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    dm_measure->pos = uint_to_float(dm_measure->p_int, DM_P_MIN, DM_P_MAX, 16);  // (-12.5,12.5)
    dm_measure->vel = uint_to_float(dm_measure->v_int, DM_V_MIN, DM_V_MAX, 12);  // (-45.0,45.0)
    dm_measure->tor = uint_to_float(dm_measure->t_int, DM_T_MIN, DM_T_MAX, 12);  // (-18.0,18.0)
    dm_measure->t_mos = (float)(rx_data[6]);
    dm_measure->t_rotor = (float)(rx_data[7]);

    dm_measure->last_fdb_time = HAL_GetTick();
}
/**
 * @brief          ???DM???????????
 * @param[out]     motor ??????? 
 * @param[in]      dm_measure ????????????????
 * @return         none
 */
static void GetDmFdbData(Motor_s * motor, const DmMeasure_s * dm_measure)
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
				case DM_4310:   
					{
					     if (p_motor->can == 1) {
                GetDmFdbData(p_motor, &CAN1_DM_MEASURE[p_motor->id - 1]);
            } else {
                GetDmFdbData(p_motor, &CAN2_DM_MEASURE[p_motor->id - 1]);
            }
			  	}break;
					
        default:
            break;
    }
}


/**
 * @brief          ??????????????????StdId????????
 * @note           ???????????DJI??????????????????
 * @param[in]      CAN CAN??(CAN_1??CAN_2)
 * @param[in]      rx_header CAN?????????
 * @param[in]      rx_data CAN????????
 */
static void DecodeStdIdData(hcan_t * CAN, CAN_RxHeaderTypeDef * rx_header, uint8_t rx_data[8])
{
    switch (rx_header->StdId) {  //???????
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
        case DJI_M11_ID: {  // ????ID?DJI????????
            static uint8_t i = 0;
            i = rx_header->StdId - DJI_M1_ID;
            if (CAN == &hcan1)  // ???????????????? CAN1 ?????
            {
                DjiFdbData(&CAN1_DJI_MEASURE[i], rx_data);
            } else if (CAN == &hcan2)  // ???????????????? CAN2 ?????
            {
                DjiFdbData(&CAN2_DJI_MEASURE[i], rx_data);
            }
            return;
        }
				
				case DM_M1_ID:
        case DM_M2_ID:
        case DM_M3_ID:
        case DM_M4_ID:
        case DM_M5_ID:
        case DM_M6_ID: {  // ????ID?DM????????
            static uint8_t i = 0;
            i = rx_header->StdId - DM_M1_ID;
            if (CAN == &hcan1)  // ???????????????? CAN1 ?????
            {
                DmFdbData(&CAN1_DM_MEASURE[i], rx_data);
            } else if (CAN == &hcan2)  // ???????????????? CAN2 ?????
            {
                DmFdbData(&CAN2_DM_MEASURE[i], rx_data);
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
	 
	 mtoc_mesasge->key_v=(uint16_t)((rx_data[4] << 8 | rx_data[5]));
	 
	 mtoc_mesasge->mode=rx_data[6];
	 
	 mtoc_mesasge->receive_chassis_thing=rx_data[7];
}

void M_communication_c_556(MTOC_message_t *mtoc_mesasge,uint8_t *rx_data)
{
	mtoc_mesasge->mouse_RL=(uint16_t)((rx_data[0] << 8 | rx_data[1]));
	
	mtoc_mesasge->mouse_UD=(uint16_t)((rx_data[2] << 8 | rx_data[3]));
	
	mtoc_mesasge->yaw_control=(uint16_t)((rx_data[4] << 8 | rx_data[5]));
	
	mtoc_mesasge->pitch_control=(uint16_t)((rx_data[6] << 8 | rx_data[7]));
}

void M_communication_c_557(MTOC_message_t *mtoc_mesasge,uint8_t *rx_data)
{
	  mtoc_mesasge->mouse_press_l=rx_data[0];
	  mtoc_mesasge->mouse_press_r=rx_data[1];
	
	  mtoc_mesasge->chassis_yaw_speed=(int16_t)(rx_data[2] << 8 | rx_data[3]) / 1000.0f;
	
    mtoc_mesasge->fric_flag=rx_data[4];
	  mtoc_mesasge->trigger_flag=rx_data[5];
		mtoc_mesasge->shoot_heat_limit=(int16_t)(rx_data[6] << 8 | rx_data[7]) / 1000.0f;
	
}


//gimbal--can1--fifo1  ???????????????3  ????6020  ????????????? 
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);

    if (rx_header.IDE == CAN_ID_STD)  // ????????????????StdId
    {
        DecodeStdIdData(hcan, &rx_header, rx_data);
    }
		 if (rx_header.StdId == 0x555)  // ????????????????StdId
    {
			M_communication_c(&mtoc_mesasge,rx_data);
    }
		 if (rx_header.StdId == 0x556)
		{
			M_communication_c_556(&mtoc_mesasge,rx_data);
		}
		if (rx_header.StdId == 0x557)
		{
			M_communication_c_557(&mtoc_mesasge,rx_data);
		}
		if(rx_header.StdId==0x02)//??????yaw
		{
			DmFdbData(&CAN1_DM_MEASURE[0],rx_data);
		}
}


//can2--??--fifo0
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (rx_header.IDE == CAN_ID_STD)  // ????????????????StdId
    {
        DecodeStdIdData(hcan, &rx_header, rx_data);
    }
		if(rx_header.StdId==0x04)//??????pitch
		{
			DmFdbData(&CAN2_DM_MEASURE[2],rx_data);
		}
		

}



/**
 * @brief          ???????????
 * @param[in]      can_handle ???CAN1??CAN2
 * @param[in]      tx_header  CAN????????header
 * @param[in]      tx_data    ????????
 * @return         none
 */
void CAN_SendTxMessage(CanCtrlData_s * can_ctrl_data)
{
    uint32_t send_mail_box;
    uint8_t cnt = 20;  // ?????????

    uint32_t free_TxMailbox =
        HAL_CAN_GetTxMailboxesFreeLevel(can_ctrl_data->hcan);  // ???????��???????
    while (free_TxMailbox < 3 && cnt--) {                      // ???????????????3
        free_TxMailbox = HAL_CAN_GetTxMailboxesFreeLevel(can_ctrl_data->hcan);
    }
    HAL_CAN_AddTxMessage(
        can_ctrl_data->hcan, &can_ctrl_data->tx_header, can_ctrl_data->tx_data, &send_mail_box);
}
/**
 * @brief          ???CAN????DJI???(???GM3508 GM2006 GM6020)
 * @param[in]      can ????????????can??(1/2)
 * @param[in]      std_id ????????????std_id
 * @param[in]      curr_1 ??????????(id=1/5)
 * @param[in]      curr_2 ??????????(id=2/6)
 * @param[in]      curr_3 ??????????(id=3/7)
 * @param[in]      curr_4 ??????????(id=4/8)
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
 * @brief          ????????
 * @param[in]      hcan CAN???
 * @param[in]      std_id ?????ID
 * @param[in]      data ????8??????????????
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
/**
 * @brief      ???can??????
 * @param[in]  motor ???????
 * @return     can??????
 * @note       ???????????��?can???????????can?????????????????????????????
 */
static hcan_t * GetHcanPoint(Motor_s * motor)
{
    if (!( motor->type == DM_4310)) return NULL;

    if (motor->can == 1)
        return &hcan1;
    else if (motor->can == 2)
        return &hcan2;

    return NULL;
}
/**
************************************************************************
* @brief      	EnableMotorMode: ????????????
* @param[in]    hcan:     ???CAN_HandleTypeDef???????
* @param[in]    motor_id: ???ID??????????
* @param[in]    mode_id:  ??ID??????????????
* @retval     	void
* @details    	???CAN???????????????????????????????
************************************************************************
**/
static void EnableMotorMode(hcan_t * hcan, uint16_t motor_id, uint16_t mode_id)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + mode_id;

    CAN_CTRL_DATA.tx_data[0] = 0xFF;
    CAN_CTRL_DATA.tx_data[1] = 0xFF;
    CAN_CTRL_DATA.tx_data[2] = 0xFF;
    CAN_CTRL_DATA.tx_data[3] = 0xFF;
    CAN_CTRL_DATA.tx_data[4] = 0xFF;
    CAN_CTRL_DATA.tx_data[5] = 0xFF;
    CAN_CTRL_DATA.tx_data[6] = 0xFF;
    CAN_CTRL_DATA.tx_data[7] = 0xFC;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
 * @brief          ?????????
 * @param[in]      motor ???????
 * @param[in]      mode_id ??ID
 * @retval         none
 */
void DmEnable(Motor_s * motor)
{
    hcan_t * hcan = GetHcanPoint(motor);
    if (hcan == NULL) return;

    EnableMotorMode(hcan, motor->id, motor->mode);
}
/**
************************************************************************
* @brief      	MitCtrl: MIT??????????????
* @param[in]    hcan:			???CAN_HandleTypeDef??????????????CAN????
* @param[in]    motor_id:	    ???ID??????????
* @param[in]    pos:			��??????
* @param[in]    vel:			???????
* @param[in]    kp:				��????????
* @param[in]    kd:				��????????
* @param[in]    torq:			???????
* @retval     	void
* @details    	???CAN????????????MIT???????????
************************************************************************
**/
static void MitCtrl(
    hcan_t * hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = motor_id + DM_MODE_MIT;

    pos_tmp = float_to_uint(pos, DM_P_MIN, DM_P_MAX, 16);
    vel_tmp = float_to_uint(vel, DM_V_MIN, DM_V_MAX, 12);
    kp_tmp = float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12);
    kd_tmp = float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12);
    tor_tmp = float_to_uint(torq, DM_T_MIN, DM_T_MAX, 12);

    CAN_CTRL_DATA.tx_data[0] = (pos_tmp >> 8);
    CAN_CTRL_DATA.tx_data[1] = pos_tmp;
    CAN_CTRL_DATA.tx_data[2] = (vel_tmp >> 4);
    CAN_CTRL_DATA.tx_data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    CAN_CTRL_DATA.tx_data[4] = kp_tmp;
    CAN_CTRL_DATA.tx_data[5] = (kd_tmp >> 4);
    CAN_CTRL_DATA.tx_data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    CAN_CTRL_DATA.tx_data[7] = tor_tmp;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
 * @brief          ??????MIT????
 * @param[in]      motor ???????
 * @retval         none
 */
void DmMitCtrl(Motor_s * motor, float kp, float kd)
{
    hcan_t * hcan = GetHcanPoint(motor);
    if (hcan == NULL) return;

    MitCtrl(hcan, motor->id, motor->set.pos, motor->set.vel, kp, kd, motor->set.tor);
}







//C?��?????? can??????
void CToM_sendControl(uint8_t can, uint16_t std_id, int16_t yaw,uint8_t motor_offline,bool aim_live)
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
    CAN_CTRL_DATA.tx_data[2] = motor_offline;//?????????��
    CAN_CTRL_DATA.tx_data[3] = visionDataStu.aim_live; //?????????��
    CAN_CTRL_DATA.tx_data[4] = visionDataStu.mode;
    CAN_CTRL_DATA.tx_data[5] = 0;
    CAN_CTRL_DATA.tx_data[6] = 0;
    CAN_CTRL_DATA.tx_data[7] = 0;	
    CAN_SendTxMessage(&CAN_CTRL_DATA);
}





