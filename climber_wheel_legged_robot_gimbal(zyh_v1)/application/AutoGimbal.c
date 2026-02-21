#include "AutoGimbal.h"
#include "gimbal_behaviour.h"
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
visionDataStu_t visionDataStu;
uint8_t data_length;
uint8_t sbus_rx_buffer[BUFLENGTH];
// 定义接收协议结构体
typedef struct __attribute__((packed)) VisionToGimbal
{
    uint8_t head[2];              // 帧头 'V', 'G'
    uint8_t mode;                 // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
    float yaw;                    // Yaw角度（弧度）
    float yaw_vel;                // Yaw角速度
    float yaw_acc;                // Yaw角加速度
    float pitch;                  // Pitch角度（弧度）
    float pitch_vel;              // Pitch角速度
    float pitch_acc;              // Pitch角加速度
    uint8_t tail[2];              // 帧尾 'E', 'N'
} VisionToGimbal_t;
void AUTO_control_init(void)
{
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);   //使能串口空闲中断  两次消息间隔会触发
	HAL_UART_Receive_DMA(&huart1,sbus_rx_buffer,BUFLENGTH);  //打开串口DMA接收  声明缓存数组  缓存数组长度
}

//利用串口接收不定长数据

//修改串口接收解析函数
void Usart6Receive_IDLE(void)
{
    HAL_UART_DMAStop(&huart1);//要停止DMA的接收来处理数据
    data_length = BUFLENGTH - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    
    // 检查数据长度是否符合VisionToGimbal协议（至少需要头部+必要字段+尾部）
    if(data_length >= sizeof(VisionToGimbal_t))
    {
        VisionToGimbal_t* recv_data = (VisionToGimbal_t*)sbus_rx_buffer;
        
        // 检查帧头和帧尾是否正确
        if(recv_data->head[0] == 'V' && recv_data->head[1] == 'G' && 
           recv_data->tail[0] == 'E' && recv_data->tail[1] == 'N')
        {
            // 复制解析后的数据
            memcpy(visionDataStu.real_receive, sbus_rx_buffer, data_length);
            
            // 解析协议数据
            visionDataStu.mode = recv_data->mode;            // 模式
            visionDataStu.yaw = recv_data->yaw;              // Yaw角度（弧度）
            visionDataStu.yaw_vel = recv_data->yaw_vel;      // Yaw角速度
            visionDataStu.yaw_acc = recv_data->yaw_acc;      // Yaw角加速度
            visionDataStu.pitch = recv_data->pitch;          // Pitch角度（弧度）
            visionDataStu.pitch_vel = recv_data->pitch_vel;  // Pitch角速度
            visionDataStu.pitch_acc = recv_data->pitch_acc;  // Pitch角加速度
            
            // 根据协议，您可以在这里添加对数据的进一步处理
            // 例如：设置云台目标角度、控制射击等
        }
    }

    memset(sbus_rx_buffer, 0, BUFLENGTH);
    HAL_UART_Receive_DMA(&huart1, sbus_rx_buffer, BUFLENGTH);
}


void USART1_IRQHandler(void)
{
	if(RESET!=__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		Usart6Receive_IDLE();
	}
	HAL_UART_IRQHandler(&huart1);
}	

