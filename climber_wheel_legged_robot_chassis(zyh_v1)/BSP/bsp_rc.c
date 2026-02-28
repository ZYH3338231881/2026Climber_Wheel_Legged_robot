#include "bsp_rc.h"
#include "string.h"
#include "usart.h"
#include "Keyboard.h"
#define SBUS_HEAD 0X0F
#define SBUS_END 0X00
#define REMOTE_RC_OFFSET 1024
#define REMOTE_TOGGLE_DUAL_VAL 1024
#define REMOTE_TOGGLE_THRE_VAL_A 600
#define REMOTE_TOGGLE_THRE_VAL_B 1400
#define DEAD_AREA	120
#define abs(x) x>0?x:-x

uint8_t rx_buff[BUFF_SIZE];

RC_ctrl_t rc_ctrl = rc_Init;
Keyboard_Data keyboard_data;

/*
 *函数简介:键盘数据处理
 *参数说明:接收数据
 *返回类型:无
 *备注:无
 */
void Keyboard_DataProcess(Keyboard_Data *keyboard,RC_ctrl_t *rc_ctrl)
{
	
	

	keyboard->Remote_Key_W=rc_ctrl->key.v&0x0001;//前进
	keyboard->Remote_Key_S=(rc_ctrl->key.v>>1)&0x0001;//后退
	keyboard->Remote_Key_B=(rc_ctrl->key.v>>15)&0x0001; //跳跃
	keyboard->Remote_Key_Ctrl=(rc_ctrl->key.v>>5)&0X0001;//CTRL
  keyboard->Remote_Key_Shift=(rc_ctrl->key.v>>4)&0X0001;//SHIFT
	keyboard->Remote_Key_R=(rc_ctrl->key.v>>8)&0x0001;//R

	keyboard->Remote_Mouse_KeyL=rc_ctrl->mouse.press_l&0x0001;//手动射击
	keyboard->Remote_Mouse_KeyR=rc_ctrl->mouse.press_r&0x0001;//自瞄射击开火
    
	keyboard->Remote_Mouse_RL=rc_ctrl->mouse.x;//鼠标左右
	keyboard->Remote_Mouse_DU=-rc_ctrl->mouse.y;//鼠标上下

	
}
static void sbus_frame_parse(RC_ctrl_t *remoter, uint8_t *sbus_buf)
{
	//用于处理接收到的遥控器（rc）数据，将接收到的字节解码
    if (sbus_buf == NULL || remoter == NULL)
    {
        return;
    }

    remoter->ch0 = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    remoter->ch1 = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    remoter->ch2 = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    remoter->ch3 = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    remoter->sw1 = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    remoter->sw2= ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    remoter->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    remoter->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    remoter->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    remoter->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    remoter->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    remoter->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    remoter->ch4 = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL
		
		 Keyboard_DataProcess(&keyboard_data,remoter);

		 
		
    remoter->ch0 -= RC_CH_VALUE_OFFSET;
    remoter->ch1 -= RC_CH_VALUE_OFFSET;
    remoter->ch2 -= RC_CH_VALUE_OFFSET;
    remoter->ch3 -= RC_CH_VALUE_OFFSET;
    remoter->ch4 -= RC_CH_VALUE_OFFSET;
}

const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{

	if(huart->Instance == UART5)
	{
		if (Size == BUFF_SIZE)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE); // 接收完毕后重启
			sbus_frame_parse(&rc_ctrl, rx_buff);
//			memset(rx_buff, 0, BUFF_SIZE);
		}
	}
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == UART5)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5,(uint8_t *)rx_buff, 18); // 接收发生错误后重启
		memset(rx_buff, 0, 18);							   // 清除接收缓存		
	}
}
