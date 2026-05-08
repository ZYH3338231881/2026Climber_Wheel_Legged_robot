/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "struct_typedef.h"
#include "bsp_rc.h"
#include "stdbool.h"

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __packed struct
        {
                uint16_t v;
								
        } key;
				bool offline : 1;

} RC_ctrl_t;
typedef struct
{
	int16_t Remote_Mouse_RL;//鼠标X轴-鼠标左右速度,范围-32768~32767,向右为正,向左为负,静止值为0
	int16_t Remote_Mouse_DU;//鼠标Y轴-鼠标前后速度,范围-32768~32767,向后为正,向前为负,静止值为0
	int16_t Remote_Mouse_Wheel;//鼠标Z轴-鼠标滚轮速度,范围-32768~32767,向前为正,向后为负,静止值为0
	uint8_t Remote_Mouse_KeyL;//鼠标左键,按下为1,未按下为0
	uint8_t Remote_Mouse_KeyR;//鼠标右键,按下为1,未按下为0
	
	uint8_t Remote_Key_W;//键盘W键,按下为1,未按下为0
	uint8_t Remote_Key_S;//键盘S键,按下为1,未按下为0
	uint8_t Remote_Key_A;//键盘A键,按下为1,未按下为0
	uint8_t Remote_Key_D;//键盘D键,按下为1,未按下为0
	uint8_t Remote_Key_Q;//键盘Q键,按下为1,未按下为0
	uint8_t Remote_Key_E;//键盘E键,按下为1,未按下为0
	uint8_t Remote_Key_R;
	uint8_t Remote_Key_F;
	uint8_t Remote_Key_G;
	uint8_t Remote_Key_Z;
	uint8_t Remote_Key_X;
	uint8_t Remote_Key_C;
	uint8_t Remote_Key_V;
	uint8_t Remote_Key_B;
	uint8_t Remote_Key_Shift;//键盘Shift键,按下为1,未按下为0
	uint8_t Remote_Key_Ctrl;//键盘Ctrl键,按下为1,未按下为0
	
	uint8_t Remote_KeyLast_Q;//上一次键盘Q键
	uint8_t Remote_KeyLast_G;//上一次键盘E键
	uint8_t Remote_KeyLast_Z;//上一次键盘E键
	uint8_t Remote_KeyLast_B;//上一次键盘E键
	uint8_t Remote_KeyLast_Ctrl;//上一次键盘Ctrl键
	uint8_t Remote_KeyPush_Q;//按下键盘Q键,按下时0,1切换
	uint8_t Remote_KeyPush_G;//按下键盘E键,按下时0,1切换
	uint8_t Remote_KeyPush_Z;//按下键盘E键,按下时0,1切换
	uint8_t Remote_KeyPush_B;//按下键盘E键,按下时0,1切换
	uint8_t Remote_KeyPush_Ctrl;//按下键盘Ctrl键,按下时0,1切换
}Keyboard_Data;//键鼠接收结构体
extern Keyboard_Data keyboard_data;

/* ----------------------- Internal Data ----------------------------------- */

extern void remote_control_init(void);
extern const RC_ctrl_t *get_remote_control_point(void);
extern uint8_t RC_data_is_error(void);
extern void slove_RC_lost(void);
extern void slove_data_error(void);
extern void sbus_to_usart1(uint8_t *sbus);
extern inline bool GetRcOffline(void);
extern void Keyboard_DataProcess(Keyboard_Data *keyboard,RC_ctrl_t *rc_ctrl);

#endif
