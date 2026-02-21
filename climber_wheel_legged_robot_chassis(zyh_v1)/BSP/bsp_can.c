#include "bsp_can.h"
#include "fdcan.h"
#include "string.h"
#include "motor.h"
#include "chassis_task.h"
#include "CAN_receive.h"
FDCAN_RxHeaderTypeDef RxHeader1;  
uint8_t g_Can1RxData[64];


FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t g_Can2RxData[64];
extern CTOM_message_t ctom_message;
extern DmMeasure_s CAN1_DM_MEASURE[4];
extern DjiMotorMeasure_t CAN2_DJI_MEASURE[5];
void C_communication_M(CTOM_message_t *ctom_mesasge,uint8_t *rx_data)
{
   ctom_mesasge->gimbal_yaw_6020 = (int16_t)(rx_data[0] << 8 | rx_data[1]) / 1000.0f;
}


void FDCAN1_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */	
	sFilterConfig.IdType = FDCAN_STANDARD_ID;//扩展ID不接收
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x00000000; // 在这里设置不滤除所有ID
  sFilterConfig.FilterID2 = 0x00000000; // 
  if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
		
/* 全局过滤设置 */
/* 接收到消息ID与标准ID过滤不匹配，不接受 */
/* 接收到消息ID与扩展ID过滤不匹配，不接受 */
/* 过滤标准ID远程帧 */ 
/* 过滤扩展ID远程帧 */ 
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

	/* 开启RX FIFO0的新数据中断 */
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
 

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

void FDCAN2_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */
  sFilterConfig.IdType =  FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  sFilterConfig.FilterID1 = 0x00000000;
  sFilterConfig.FilterID2 = 0x00000000;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }


  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
}

void FDCAN3_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */
  sFilterConfig.IdType =  FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;//只接受范围内的报文
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x400;
  sFilterConfig.FilterID2 = 0x7FF;
  if (HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }


  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
}



uint8_t canx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
	FDCAN_TxHeaderTypeDef TxHeader;

	TxHeader.Identifier = id;                 // CAN ID
  TxHeader.IdType =  FDCAN_STANDARD_ID ;    //标准ID
  TxHeader.TxFrameType = FDCAN_DATA_FRAME; //数据帧
	TxHeader.DataLength = len;   //用的是FD_CAN的经典CAN模式  0~8个字节
  TxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch=FDCAN_BRS_OFF;    //关闭波特率切换
  TxHeader.FDFormat=FDCAN_CLASSIC_CAN;     //经典CAN
  TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker=0;

	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data)!=HAL_OK) 
		return 1;//发送
	return 0;	
}

/*                 fdcan1接收                         */
uint8_t rx_data1[8] = {0};
uint16_t rec_id1;
/*               下面FIFO要根据cubemx去设置         */
void fdcan1_rx_callback(void)
{
  FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t len;
	
	if(HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&pRxHeader, rx_data1)==HAL_OK)
	{
		rec_id1 = pRxHeader.Identifier;

	if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_8) //经典CAN里面只有8个字节  四个DM8009P关节电机
	{		if(rec_id1==0x101)                        
		 {
		   DmFdbData(&CAN1_DM_MEASURE[0],rx_data1);  //左前电机
		 }
		 if(rec_id1==0x102)
		 {
		   DmFdbData(&CAN1_DM_MEASURE[1],rx_data1);  //左后电机
		 }
		 if(rec_id1==0x103)
		 {
		   DmFdbData(&CAN1_DM_MEASURE[2],rx_data1);  //右前电机
		 }
		 if(rec_id1==0x104)
		 {
		   DmFdbData(&CAN1_DM_MEASURE[3],rx_data1);  //右后电机
		 }
	}
	}
}

/*                 fdcan2接收                         */
uint8_t rx_data2[8] = {0};
uint16_t rec_id2;
void fdcan2_rx_callback(void)
{
  FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t len;
	
	if(HAL_FDCAN_GetRxMessage(&hfdcan2,FDCAN_RX_FIFO1,&pRxHeader, rx_data2)==HAL_OK)
	{
		rec_id2 = pRxHeader.Identifier;

	if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_8) //经典CAN里面只有8个字节   大疆3508电机  左0x0202   右0x0201
	{
			if(rec_id2==0x0202)
		 {
			 DjiFdbData(&CAN2_DJI_MEASURE[0], rx_data2);
		 }
		 if(rec_id2==0x0201)
		 {
  		 DjiFdbData(&CAN2_DJI_MEASURE[1], rx_data2);
		 }
	}
	
	}
}
/*                 fdcan3接收                         */
uint8_t rx_data3[8] = {0};
uint16_t rec_id3;
void fdcan3_rx_callback(void)
{
	
	FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t len;
	
	if(HAL_FDCAN_GetRxMessage(&hfdcan3,FDCAN_RX_FIFO0,&pRxHeader, rx_data3)==HAL_OK)
	{
		rec_id3 = pRxHeader.Identifier;

	if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_8) //经典CAN里面只有8个字节   大疆3508电机  左0x0202   右0x0201
	{
      if(rec_id3==0x666)//C板下发数据
     {
       C_communication_M(&ctom_message,rx_data3);
     }

	}
	}
}

/* 修正后的RX FIFO0回调函数 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    /* 首先检查是否真的是"新消息"中断，而不是溢出等其他中断 */
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {
        if (hfdcan == &hfdcan1) {
            fdcan1_rx_callback(); // CAN1使用FIFO0
        }
        if (hfdcan == &hfdcan3) {
            fdcan3_rx_callback(); // CAN3使用FIFO0
        }
        // CAN2配置为使用FIFO1，所以不应在FIFO0的回调中处理
    }
}

/* 修正后的RX FIFO1回调函数 - 注意参数名改为RxFifo1ITs */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) // 注意：参数名已修正！
{
    /* 检查FIFO1的新消息中断 */
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0) {
        if (hfdcan == &hfdcan2) {
            fdcan2_rx_callback(); // CAN2使用FIFO1
        }
        // CAN1和CAN3不使用FIFO1，所以不在这里处理
    }  
}




