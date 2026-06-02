#include "remote_control.h"
#include "main.h"
#include "string.h"
#include "usart.h"
//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700
// 遥控器掉线时间阈值
#define RC_LOST_TIME 100  // ms
static uint32_t receive_count = 0;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
//取正函数
static int16_t RC_abs(int16_t value);
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
Keyboard_Data keyboard_data;

RC_ctrl_t rc_ctrl;
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
// 上一次接收数据的时间
static uint32_t last_receive_time = 0;

void remote_control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

//判断遥控器数据是否出错，
uint8_t RC_data_is_error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}

void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
// clang-format on
// 记录接收数据的次数
#define COUNT_RECEIVED                            \
    if (now - last_receive_time > RC_LOST_TIME) { \
        receive_count = 0;                        \
    }                                             \
    receive_count++;
//串口中断
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);
			        
				uint32_t now = HAL_GetTick();


        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {   
								//处理遥控器数据
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
                
                COUNT_RECEIVED
                
                //记录数据接收时间
                last_receive_time = HAL_GetTick();
							
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);

            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
			
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);


            }
        }
    }

}

static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

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
  static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
		
	
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL
    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;

    Keyboard_DataProcess(&keyboard_data,rc_ctrl);
    
}
/**
  * @brief          获取遥控器是否离线。
  * @retval         true:离线，false:在线
  */
inline bool GetRcOffline(void)
{

    return !((receive_count > 5) && (HAL_GetTick() - last_receive_time < RC_LOST_TIME));
}

