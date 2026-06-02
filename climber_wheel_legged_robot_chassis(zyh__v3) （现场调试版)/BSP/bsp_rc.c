#include "bsp_rc.h"
#include "string.h"
#include "usart.h"
#include "Keyboard.h"
#include "chassis_task.h"
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
extern CTOM_message_t ctom_message;

/*
 *函数简介:键盘数据处理
 *参数说明:接收数据
 *返回类型:无
 *备注:无
 */
 
void Keyboard_DataProcess(Keyboard_Data *keyboard,RC_ctrl_t *rc_ctrl)
{
	
	// 核心解析函数：将遥控器数据解析为键鼠结构体
    // -------------------------- 1. 基础按键赋值 --------------------------
    // 方向键
    keyboard->Remote_Key_W = (rc_ctrl->key.v & 0x0001) ? 1 : 0;    // W (0x0001)
    keyboard->Remote_Key_S = (rc_ctrl->key.v & 0x0002) ? 1 : 0;    // S (0x0002)
    keyboard->Remote_Key_A = (rc_ctrl->key.v & 0x0004) ? 1 : 0;    // A (0x0004)
    keyboard->Remote_Key_D = (rc_ctrl->key.v & 0x0008) ? 1 : 0;    // D (0x0008)
    
    // 功能键
    keyboard->Remote_Key_Shift = (rc_ctrl->key.v & 0x0010) ? 1 : 0;// Shift (0x0010) 加速
    keyboard->Remote_Key_Ctrl = (rc_ctrl->key.v & 0x0020) ? 1 : 0; // Ctrl (0x0020)
    keyboard->Remote_Key_Q = (rc_ctrl->key.v & 0x0040) ? 1 : 0;    // Q (0x0040)
    keyboard->Remote_Key_E = (rc_ctrl->key.v & 0x0080) ? 1 : 0;    // E (0x0080)
    keyboard->Remote_Key_R = (rc_ctrl->key.v & 0x0100) ? 1 : 0;    // R (0x0100)
    keyboard->Remote_Key_F = (rc_ctrl->key.v & 0x0200) ? 1 : 0;    // F (0x0200)
    keyboard->Remote_Key_G = (rc_ctrl->key.v & 0x0400) ? 1 : 0;    // G (0x0400)
    keyboard->Remote_Key_Z = (rc_ctrl->key.v & 0x0800) ? 1 : 0;    // Z (0x0800)
    keyboard->Remote_Key_X = (rc_ctrl->key.v & 0x1000) ? 1 : 0;    // X (0x1000)
    keyboard->Remote_Key_C = (rc_ctrl->key.v & 0x2000) ? 1 : 0;    // C (0x2000)
    keyboard->Remote_Key_V = (rc_ctrl->key.v & 0x4000) ? 1 : 0;    // V (0x4000)
    keyboard->Remote_Key_B = (rc_ctrl->key.v & 0x8000) ? 1 : 0;    // B (0x8000) 跳跃
    
    // -------------------------- 2. 鼠标数据赋值 --------------------------
    keyboard->Remote_Mouse_RL = rc_ctrl->mouse.x;          // 鼠标左右（右正左负）
    keyboard->Remote_Mouse_DU = -rc_ctrl->mouse.y;         // 鼠标上下（后正前负，取反匹配定义）
    keyboard->Remote_Mouse_Wheel = rc_ctrl->mouse.z;       // 鼠标滚轮（前正后负）
    keyboard->Remote_Mouse_KeyL = rc_ctrl->mouse.press_l;  // 鼠标左键（按下为1）
    keyboard->Remote_Mouse_KeyR = rc_ctrl->mouse.press_r;  // 鼠标右键（按下为1）
    
    // -------------------------- 3. 按键状态记录与触发 --------------------------
    // 1) 保存上一次按键状态
    keyboard->Remote_KeyLast_Q = keyboard->Remote_Key_Q;
    keyboard->Remote_KeyLast_G = keyboard->Remote_Key_G;
    keyboard->Remote_KeyLast_Z = keyboard->Remote_Key_Z;
    keyboard->Remote_KeyLast_B = keyboard->Remote_Key_B;
    keyboard->Remote_KeyLast_Ctrl = keyboard->Remote_Key_Ctrl;
    
    // 2) 检测按键"按下触发"（仅在按键从0→1时切换状态）
    // Q键触发：上一次未按，当前按下 → 切换状态
    if (keyboard->Remote_KeyLast_Q == 0 && keyboard->Remote_Key_Q == 1) {
        keyboard->Remote_KeyPush_Q = !keyboard->Remote_KeyPush_Q;
    }
    // G键触发
    if (keyboard->Remote_KeyLast_G == 0 && keyboard->Remote_Key_G == 1) {
        keyboard->Remote_KeyPush_G = !keyboard->Remote_KeyPush_G;
    }
    // Z键触发
    if (keyboard->Remote_KeyLast_Z == 0 && keyboard->Remote_Key_Z == 1) {
        keyboard->Remote_KeyPush_Z = !keyboard->Remote_KeyPush_Z;
    }
    // B键触发
    if (keyboard->Remote_KeyLast_B == 0 && keyboard->Remote_Key_B == 1) {
        keyboard->Remote_KeyPush_B = !keyboard->Remote_KeyPush_B;
    }
    // Ctrl键触发
    if (keyboard->Remote_KeyLast_Ctrl == 0 && keyboard->Remote_Key_Ctrl == 1) {
        keyboard->Remote_KeyPush_Ctrl = !keyboard->Remote_KeyPush_Ctrl;
    }

	
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
