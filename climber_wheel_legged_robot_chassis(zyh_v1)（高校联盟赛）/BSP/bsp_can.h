#ifndef _BSP_CAN_H
#define _BSP_CAN_H


#include "main.h"
//#include "CAN_receive.h"

typedef FDCAN_HandleTypeDef hcan_t;

typedef struct __CanCtrlData
{
    hcan_t * hcan;
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
} CanCtrlData_s;


void FDCAN1_Config(void);
void FDCAN2_Config(void);
void FDCAN3_Config(void);
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);



#endif
