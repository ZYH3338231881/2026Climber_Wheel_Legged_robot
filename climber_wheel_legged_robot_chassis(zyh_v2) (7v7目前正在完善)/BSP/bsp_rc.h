#ifndef __UART_BSP_H__
#define __UART_BSP_H__
#include "struct_typedef.h"
#define BUFF_SIZE	18

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
	uint8_t Remote_KeyLast_E;//上一次键盘E键
	uint8_t Remote_KeyLast_F;//上一次键盘F键
	
	uint8_t Remote_KeyLast_G;//上一次键盘E键
	uint8_t Remote_KeyLast_Z;//上一次键盘E键
	uint8_t Remote_KeyLast_B;//上一次键盘E键
	uint8_t Remote_KeyLast_Ctrl;//上一次键盘Ctrl键
	
	uint8_t Remote_KeyPush_Q;//按下键盘Q键,按下时0,1切换
	uint8_t Remote_KeyPush_E;//按下键盘E键,按下时0,1切换
	uint8_t Remote_KeyPush_F;//按下键盘F键,按下时0,1切换
	uint8_t Remote_KeyPush_G;//按下键盘E键,按下时0,1切换
	uint8_t Remote_KeyPush_Z;//按下键盘E键,按下时0,1切换
	uint8_t Remote_KeyPush_B;//按下键盘E键,按下时0,1切换

	uint8_t Remote_KeyPush_Ctrl;//按下键盘Ctrl键,按下时0,1切换
}Keyboard_Data;//键鼠接收结构体



typedef __packed struct
{
  int16_t ch0;   //右摇杆水平  右正左负 （-660  660） 
  int16_t ch1;   //右摇杆竖直  上正下负 （-660  660） 
  int16_t ch2;   //左摇杆水平  右正左负 （-660  660） 
  int16_t ch3;   //右摇杆竖直  上正下负 （-660  660） 
  int16_t ch4;  //左上滑轮    下正上负
  uint16_t sw1;   //左拨杆 下 中 上 2 3 1
  uint16_t sw2;   //右拨杆 下 中 上 2 3 1
	
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

	
} RC_ctrl_t;
 
#define rc_Init   \
{                 \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
}
extern RC_ctrl_t rc_ctrl;

extern const RC_ctrl_t *get_remote_control_point(void);



#endif /*__UART_BSP_H__ */

