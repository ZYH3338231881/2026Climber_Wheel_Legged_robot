#include "TOF_distance.h"
extern UART_HandleTypeDef huart10;
extern DMA_HandleTypeDef hdma_usart10_rx;
TOF_data_t tof_data;
uint8_t data_length;
uint8_t sbus_rx_buffer[BUFLENGTH];

void TOF_control_init(void)
{
	__HAL_UART_ENABLE_IT(&huart10,UART_IT_IDLE);   //使能串口空闲中断  两次消息间隔会触发
	HAL_UART_Receive_DMA(&huart10,sbus_rx_buffer,BUFLENGTH);  //打开串口DMA接收  声明缓存数组  缓存数组长度
}

//利用串口接收不定长数据

//修改串口接收解析函数
void Usart10Receive_IDLE(void)
{
    HAL_UART_DMAStop(&huart10);//要停止DMA的接收来处理数据
     data_length = BUFLENGTH - __HAL_DMA_GET_COUNTER(&hdma_usart10_rx);
    
		if( sbus_rx_buffer[0] == 0x59 && sbus_rx_buffer[1] == 0x59)
    {
      tof_data.distance=  (float)(sbus_rx_buffer[2] + sbus_rx_buffer[3] * 256);
			tof_data.strength = sbus_rx_buffer[4] + sbus_rx_buffer[5] * 256;
    }


    memset(sbus_rx_buffer, 0, BUFLENGTH);
    HAL_UART_Receive_DMA(&huart10, sbus_rx_buffer, BUFLENGTH);
}


void USART10_IRQHandler(void)
{
	if(RESET!=__HAL_UART_GET_FLAG(&huart10,UART_FLAG_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart10);
		Usart10Receive_IDLE();
	}
	HAL_UART_IRQHandler(&huart10);
}	

