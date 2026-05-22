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
//超级电容接收数据信息
typedef struct
{
    uint8_t errorCode;//错误代码
    float chassisPower;//底盘实时功率
    uint16_t chassisPowerLimit;//底盘功率限制，超出会进行功率补偿
    uint8_t capEnergy;//值域为：0-255 255是满电
}SuperCap_rx_t;



void FDCAN1_Config(void);
void FDCAN2_Config(void);
void FDCAN3_Config(void);
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);



#endif
