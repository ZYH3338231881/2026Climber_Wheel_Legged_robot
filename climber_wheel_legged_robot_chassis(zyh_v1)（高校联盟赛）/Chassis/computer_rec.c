#include "computer_rec.h"
#include "usart.h"
#include "chassis_task.h"
#include "crc_check.h"
extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_uart7_rx;

  uint8_t JudgeDataBuffer[JudgeBufferLength];
  SelfDefineInfo_t SelfDefineInfo;
  JudgementDataTypedef JudgementData;

uint8_t judge_re[400];

void Computer_rec_init(void)
{
	__HAL_UART_ENABLE_IT(&huart7,UART_IT_IDLE);   //使能串口空闲中断  两次消息间隔会触发
	HAL_UART_Receive_DMA(&huart7,judge_re,400);  //打开串口DMA接收  声明缓存数组  缓存数组长度
}

void judgeCalculate(uint8_t JudgeDataBuffer[JudgeBufferLength])//裁判系统解算
{	
	static uint16_t start_pos=0,next_start_pos=0;
	while(1)
	{
		memcpy(&JudgementData.frameHeader, &JudgeDataBuffer[start_pos],FrameHeader_Len);
		/*先校验头帧0xA5 然后crc8校验帧头 再crc16位校验整包*/
		if((JudgementData.frameHeader.SOF==(uint16_t)JudgeFrameHeader)\
    &&(1==Verify_CRC8_Check_Sum(&JudgeDataBuffer[start_pos],FrameHeader_Len))\
    &&(1==Verify_CRC16_Check_Sum(&JudgeDataBuffer[start_pos], JudgementData.frameHeader.DataLenth+FrameHeader_Len+4)))//数据位长度+帧头长度+命令码长度+校验码长度
		{
			memcpy(&JudgementData.rxCmdId, (&JudgeDataBuffer[start_pos]+5), sizeof(JudgementData.rxCmdId));
			JudgeDataBuffer[start_pos]++;//每处理完一次就在帧头加一防止再次处理这帧数据
			next_start_pos=start_pos+9+JudgementData.frameHeader.DataLenth;//9为 5位帧头 2位数据长度 2校验位
			
			switch(JudgementData.rxCmdId)
			{
				case ID_game_state://读取比赛状态数据 0x0001
				{
					memcpy(&JudgementData.GameState,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				case ID_game_result://读取比赛结果 0x0002
				{
					memcpy(&JudgementData.GameResult,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				case ID_game_robot_HP://读取机器人血量数0x0003
				{
					memcpy(&JudgementData.GameRobotHP,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				case ID_event_data://读取场地事件 0x0101
				{
					memcpy(&JudgementData.EventData,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				case ID_supply_projectile_action://读取补给站数据 0x0102
				{
					memcpy(&JudgementData.SupplyProjectileAction,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				case ID_referee_warning://0x0104,//裁判警告
				{
					memcpy(&JudgementData.RefereeWarn,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;				
				case ID_dart_remaining_time://0x0105,//飞镖发射相关数据
				{
					memcpy(&JudgementData.DartRemain,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				
				case Robot_Status_ID://0x0201,//机器人状态  等级
				{
					memcpy(&JudgementData.robot_status_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				case power_heat_data_ID://0x0202,//枪口热量 底盘功率
				{	
					memcpy(&JudgementData.power_heat_data_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
			  }					
				case robot_pos_ID://0x0203,//机器人位置数据
				{
					memcpy(&JudgementData.robot_pos_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去
				}
				break;
				case robot_buff_ID: //0x0204,//buff
        {
					memcpy(&JudgementData.buff_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}
				break;
				case robot_hurt_ID://=0x0206,//伤害类型
				{
					memcpy(&JudgementData.hurt_data_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}		
				break;
				case shoot_data_ID://0x0207,//射频射速
				{
					memcpy(&JudgementData.shoot_data_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}		
				break;
				case projectile_allowance_ID: //0x0208,//允许发弹量	
        {
					memcpy(&JudgementData.projectile_allowance_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}
				break;
				case rfid_status_ID: //=0x0209,//机器人 RFID 模块状态
        {
					memcpy(&JudgementData.rfid_status_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}
				break;
				case dart_client_cmd_ID:// =0x020A,//飞镖选手端指令数据
				{
					memcpy(&JudgementData.dart_client_cmd_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}		
				break;
				case ground_robot_position_ID: //	=0x020B,//地面机器人位置数据
        {
					memcpy(&JudgementData.ground_robot_position_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}
				break;
				case radar_mark_data_ID://=0x020C,//雷达标记进度数据
				{
					memcpy(&JudgementData.radar_mark_data_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}		
				break;
				case sentry_info_ID://  =0x020D,//哨兵自主决策信息同步
				{
					memcpy(&JudgementData.sentry_info_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}		
				break;				
				case radar_info_t_ID://=0x020E,//雷达自主决策信息同步
				{
					memcpy(&JudgementData.radar_info_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}		
				break;						
				
				case student_interactive_header_ID:
        {
					memcpy(&JudgementData.userinfo,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}
				break;
//				case controller_header_ID: //自定义控制器
//        {
//					memcpy(&JudgementData.controller_data_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
//				}
//				break;
				case key_inpict_ID: //图传串口传键鼠
        {
				memcpy(&JudgementData.robot_key_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//把数组中的数据复制到对应的结构体中去			
				}
				break;
				
			}
			start_pos=next_start_pos;
		}
		else
		{
			start_pos=0;
			break;				
		}		
		/**如果头指针越界了退出循环**/
		if(start_pos>JudgeBufferLength)
		{
			start_pos=0;
			break;
		}
	}
}

//利用串口接收不定长数据

void Usart7Receive_IDLE(void)
{
	HAL_UART_DMAStop(&huart7);//要停止DMA的接收来处理数据
  judgeCalculate(judge_re);
	memset(judge_re,0,400);
	HAL_UART_Receive_DMA(&huart7,judge_re,400);
}


void UART7_IRQHandler(void)
{
	if(RESET!=__HAL_UART_GET_FLAG(&huart7,UART_FLAG_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart7);
		Usart7Receive_IDLE();
	}
	HAL_UART_IRQHandler(&huart7);
}	
