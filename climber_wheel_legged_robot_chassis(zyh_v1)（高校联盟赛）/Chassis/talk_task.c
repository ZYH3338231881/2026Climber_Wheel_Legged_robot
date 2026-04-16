#include "talk_task.h"
#include "cmsis_os.h"
#include "bsp_delay.h"
#include "computer_rec.h"
#include "CAN_cmd_damiao.h"
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern  Keyboard_Data keyboard_data;
extern  JudgementDataTypedef JudgementData;
extern Chassis_s CHASSIS;
extern void keyboard_sendControl(uint8_t can, uint16_t std_id,int16_t mouse_x,int16_t mouse_y,int16_t yaw_control,int16_t pitch_control);
extern void keyboard_sendControl2(uint8_t can, uint16_t std_id,uint8_t mouse_press_l,uint8_t mouse_press_r,fp32 yaw_speed,uint8_t fric_flag,uint8_t trigger_flag);


static CanCtrlData_s CAN_CTRL_DATA = {
    .tx_header.IdType = FDCAN_STANDARD_ID,  //标准帧
    .tx_header.TxFrameType = FDCAN_DATA_FRAME,//数据帧
    .tx_header.DataLength = 8,  //数据长度
};
uint8_t FRIC_FLAG=0,TRIGGER_FLAG=0;
void Talk_Task_start(void const * argument)
{
  /* USER CODE BEGIN Talk_Task_start */
  /* Infinite loop */
  for(;;)
  {
		MToC_sendControl(3,0x555,JudgementData.power_heat_data_t.shooter_17_heat1,JudgementData.shoot_data_t.initial_speed,CHASSIS.rc->key.v,CHASSIS.mode); 
		osDelay(1);
		keyboard_sendControl(3,0x556,keyboard_data.Remote_Mouse_RL,keyboard_data.Remote_Mouse_DU,CHASSIS.rc->ch0,CHASSIS.rc->ch2);
		osDelay(1);
		if(CHASSIS.rc->ch3>=300)
		{
			FRIC_FLAG=1;
		}
		else
		{
			FRIC_FLAG=0;
		}
		
		if(CHASSIS.rc->ch2>=300)
		{
			TRIGGER_FLAG=1;
		}
		else
		{
			TRIGGER_FLAG=0;
		}
		keyboard_sendControl2(3,0x557,keyboard_data.Remote_Mouse_KeyL,keyboard_data.Remote_Mouse_KeyR,CHASSIS.fdb.body.yaw_dot,FRIC_FLAG,TRIGGER_FLAG);
		osDelay(1);

  }
  /* USER CODE END Talk_Task_start */
}

void keyboard_sendControl(uint8_t can, uint16_t std_id,int16_t mouse_x,int16_t mouse_y,int16_t yaw_control,int16_t pitch_control)
{
    hcan_t * hcan = NULL;
    if (can == 1)
        hcan = &hfdcan1;
    else if (can == 2)
        hcan = &hfdcan2;
		else if (can == 3)
        hcan = &hfdcan3;
    if (hcan == NULL) return;
                                                
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.Identifier = std_id;

    CAN_CTRL_DATA.tx_data[0] = mouse_x>>8;
    CAN_CTRL_DATA.tx_data[1] = mouse_x;
    CAN_CTRL_DATA.tx_data[2] = mouse_y>>8;
    CAN_CTRL_DATA.tx_data[3] = mouse_y;
    CAN_CTRL_DATA.tx_data[4] = yaw_control>>8;
    CAN_CTRL_DATA.tx_data[5] = yaw_control;
    CAN_CTRL_DATA.tx_data[6] = pitch_control>>8;
    CAN_CTRL_DATA.tx_data[7] = pitch_control;
    canx_send_data(CAN_CTRL_DATA.hcan,CAN_CTRL_DATA.tx_header.Identifier,CAN_CTRL_DATA.tx_data,8);
}
void keyboard_sendControl2(uint8_t can, uint16_t std_id,uint8_t mouse_press_l,uint8_t mouse_press_r,fp32 yaw_speed,uint8_t fric_flag,uint8_t trigger_flag)
{
    hcan_t * hcan = NULL;
    if (can == 1)
        hcan = &hfdcan1;
    else if (can == 2)
        hcan = &hfdcan2;
		else if (can == 3)
        hcan = &hfdcan3;
    if (hcan == NULL) return;
   int16_t yaw_speed_int16=(int16_t)((yaw_speed) * 1000);
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.Identifier = std_id;

    CAN_CTRL_DATA.tx_data[0] =mouse_press_l ;
    CAN_CTRL_DATA.tx_data[1] =mouse_press_r;
    CAN_CTRL_DATA.tx_data[2] = yaw_speed_int16>>8;
    CAN_CTRL_DATA.tx_data[3] = yaw_speed_int16;
    CAN_CTRL_DATA.tx_data[4] = fric_flag;
    CAN_CTRL_DATA.tx_data[5] = trigger_flag;
    CAN_CTRL_DATA.tx_data[6] = 0;
    CAN_CTRL_DATA.tx_data[7] = 0;
    canx_send_data(CAN_CTRL_DATA.hcan,CAN_CTRL_DATA.tx_header.Identifier,CAN_CTRL_DATA.tx_data,8);
}
