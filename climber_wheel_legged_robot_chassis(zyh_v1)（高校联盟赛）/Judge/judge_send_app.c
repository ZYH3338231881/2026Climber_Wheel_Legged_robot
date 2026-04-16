/**
  ****************************(C) COPYRIGHT 2024 HRBUST_AIR****************************
  * @file    Judge_Data.c
	* @brief   DJI裁判系统数据发送函数
  * @author  JackyJuu , HRBUST_AIR_TEAM , website:www.airclub.tech
	* @Note 	 所有设计裁判系统发送接收函数如下所示
  * @version V2.0.0
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     12-2-2020      JackyJuu            Done
  *  V1.2.0     4-2-2021       JackyJuu            Done
  *  V1.6.1     3-3-2024       LEE          		   Done
  ****************************(C) COPYRIGHT 2021 HRBUST_AIR****************************
	* @describe DJI裁判系统串口数据
*/
/************************ 2024年3月Climber李川裁判系统解析  ***********************/
/*********************************************************************************/


#include "judge_send_app.h"
#include "computer_rec.h"
#include <string.h>
#include "crc_check.h"
#include "usart.h"


void make_line(robot_interaction_figure_t *custom_grapic_draw, 
                uint8_t stun, uint8_t name[3], uint8_t color, uint8_t width,uint8_t line_type,uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey,uint8_t type,uint8_t layer)
{
//	custom_grapic_draw->data_cmd_id = 0x0101;									   // 绘制一个图
//	custom_grapic_draw->sender_ID = JudgeStructure.ext_game_robot_status.robot_id; // 发送者ID，机器人对应ID
//	custom_grapic_draw->receiver_ID = (JudgeStructure.ext_game_robot_status.robot_id + 0x100);
	custom_grapic_draw->interaction_figure_t[stun].figure_name[0] = name[0];
	custom_grapic_draw->interaction_figure_t[stun].figure_name[1] = name[1];
	custom_grapic_draw->interaction_figure_t[stun].figure_name[2] = name[2]; // 图形名
	// 上面三个字节代表的是图形名，用于图形索引，可自行定义
	custom_grapic_draw->interaction_figure_t[stun].operate_tpye = type; // 图形操作，0：空操作；1：增加；2：修改；3：删除；
	custom_grapic_draw->interaction_figure_t[stun].figure_tpye = line_type; // 图形类型，0为直线，1矩形2圆4圆弧  其他的查看用户手册
	custom_grapic_draw->interaction_figure_t[stun].layer = layer;		   // 图层数
	custom_grapic_draw->interaction_figure_t[stun].color = color;	   // 颜色
	custom_grapic_draw->interaction_figure_t[stun].width = width;
	custom_grapic_draw->interaction_figure_t[0].details_a = 0;
  custom_grapic_draw->interaction_figure_t[0].details_b = 0;
	custom_grapic_draw->interaction_figure_t[stun].start_x = sx;
	custom_grapic_draw->interaction_figure_t[stun].start_y = sy;
	custom_grapic_draw->interaction_figure_t[stun].details_d = ex;
	custom_grapic_draw->interaction_figure_t[stun].details_e = ey;
}

void make_cir(robot_interaction_figure_t *custom_grapic_draw,uint8_t stun,uint8_t name[3],
			 uint8_t color, uint8_t type,uint8_t cir_type,uint8_t layer,uint16_t width, uint16_t x,uint16_t y,
			uint32_t start_angle,uint32_t end_angle,uint32_t r,uint32_t x_len,uint32_t y_len)
{
	custom_grapic_draw->interaction_figure_t[stun].figure_name[0] = name[0];
	custom_grapic_draw->interaction_figure_t[stun].figure_name[1] = name[1];
	custom_grapic_draw->interaction_figure_t[stun].figure_name[2] = name[2]; // 图形名
	// 上面三个字节代表的是图形名，用于图形索引，可自行定义
	custom_grapic_draw->interaction_figure_t[stun].operate_tpye = type; // 图形操作，0：空操作；1：增加；2：修改；3：删除；
	custom_grapic_draw->interaction_figure_t[stun].figure_tpye = cir_type; // 图形类型，0为直线，1矩形2圆4圆弧  其他的查看用户手册
	custom_grapic_draw->interaction_figure_t[stun].layer = layer;		   // 图层数
	custom_grapic_draw->interaction_figure_t[stun].color = color;	   // 颜色
	custom_grapic_draw->interaction_figure_t[stun].width = width;
	custom_grapic_draw->interaction_figure_t[stun].details_a = start_angle;//起始角度
  custom_grapic_draw->interaction_figure_t[stun].details_b = end_angle;//终止角度
	custom_grapic_draw->interaction_figure_t[stun].start_x = x;
	custom_grapic_draw->interaction_figure_t[stun].start_y = y;
	custom_grapic_draw->interaction_figure_t[stun].details_c = r;
	custom_grapic_draw->interaction_figure_t[stun].details_d = x_len;
	custom_grapic_draw->interaction_figure_t[stun].details_e = y_len;
}


void make_word(robot_interaction_word_t*custom_grapic_draw,uint8_t name[3],uint8_t stun,
	uint8_t color,uint32_t size,uint8_t width,uint16_t sx,uint16_t sy,char *str,uint8_t word_type,uint8_t type,uint8_t layer)
{
  {
    custom_grapic_draw->interaction_word_t[stun].figure_name[0] = name[0];
    custom_grapic_draw->interaction_word_t[stun].figure_name[1] = name[1];
    custom_grapic_draw->interaction_word_t[stun].figure_name[2] = name[2];//图形名
    //上面三个字节代表的是图形名，用于图形索引，可自行定义
    custom_grapic_draw->interaction_word_t[stun].operate_tpye=type;//图形操作，0：空操作；1：增加；2：修改；3：删除；
		  /*0：直线1：矩形 2： 正圆3：椭圆
  4：圆弧5：浮点数6：整型数7：字符*/
    custom_grapic_draw->interaction_word_t[stun].figure_tpye=word_type;//图形类型，0为直线，其他的查看用户手册
		
    custom_grapic_draw->interaction_word_t[stun].layer=layer;//图层数
		  /*0：红/蓝（己方颜色） 1：黄色 2：绿色 3：橙色
  4：紫红色 5：粉色 6：青色 7：黑色 8：白色*/
    custom_grapic_draw->interaction_word_t[stun].color=color;//颜色
    custom_grapic_draw->interaction_word_t[stun].details_a = size;
    custom_grapic_draw->interaction_word_t[stun].details_b = sizeof(str);
    custom_grapic_draw->interaction_word_t[stun].width=width;
    custom_grapic_draw->interaction_word_t[stun].start_x=sx ;
    custom_grapic_draw->interaction_word_t[stun].start_y=sy ;
    custom_grapic_draw->interaction_word_t[stun].details_c = 0;
    custom_grapic_draw->interaction_word_t[stun].details_d = 0;
    custom_grapic_draw->interaction_word_t[stun].details_e = 0;
  }
  //存入文字
  memset(custom_grapic_draw->data,0,30);  //存储数据的数组清零
  memcpy(custom_grapic_draw->data,str,strlen(str));

}


void make_num(robot_interaction_figure_t *custom_grapic_draw,uint8_t name[3],uint8_t stun,
	uint8_t color,uint32_t size,uint8_t width,uint16_t sx,uint16_t sy,int32_t exfloat,uint8_t word_type,uint8_t type,uint8_t layer)
{
  {
    custom_grapic_draw->interaction_figure_t[stun].figure_name[0] = name[0];
    custom_grapic_draw->interaction_figure_t[stun].figure_name[1] = name[1];
    custom_grapic_draw->interaction_figure_t[stun].figure_name[2] = name[2];//图形名
    //上面三个字节代表的是图形名，用于图形索引，可自行定义
    custom_grapic_draw->interaction_figure_t[stun].operate_tpye=type;//图形操作，0：空操作；1：增加；2：修改；3：删除；
		  /*0：直线1：矩形 2： 正圆3：椭圆
  4：圆弧5：浮点数6：整型数7：字符*/
    custom_grapic_draw->interaction_figure_t[stun].figure_tpye=word_type;//图形类型，0为直线，其他的查看用户手册
		
    custom_grapic_draw->interaction_figure_t[stun].layer=layer;//图层数
		  /*0：红/蓝（己方颜色） 1：黄色 2：绿色 3：橙色
  4：紫红色 5：粉色 6：青色 7：黑色 8：白色*/
    custom_grapic_draw->interaction_figure_t[stun].color=color;//颜色
    custom_grapic_draw->interaction_figure_t[stun].details_a = size;
    custom_grapic_draw->interaction_figure_t[stun].width= width;
    custom_grapic_draw->interaction_figure_t[stun].start_x= sx ;
    custom_grapic_draw->interaction_figure_t[stun].start_y= sy ;
    custom_grapic_draw->interaction_figure_t[stun].details_c = (exfloat << 22) >> 22;;
    custom_grapic_draw->interaction_figure_t[stun].details_d = (exfloat << 11) >> 21;
    custom_grapic_draw->interaction_figure_t[stun].details_e = exfloat >> 21;
	}
	
}





uint8_t seq=0;

void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t leng)
{
  unsigned char i=i;
  uint8_t tx_buff[MAX_SIZE];
  uint16_t frame_length = frameheader_len + cmd_len + leng + crc_len;   //数据帧长度

  memset(tx_buff,0,frame_length);  //存储数据的数组清零

  /*****帧头打包*****/
  tx_buff[0] = sof;//数据帧起始字节
  memcpy(&tx_buff[1],(uint8_t*)&leng, sizeof(leng));//数据帧中data的长度
  tx_buff[3] = seq;//包序号
  Append_CRC8_Check_Sum(tx_buff,frameheader_len);  //帧头校验CRC8

  /*****命令码打包*****/
  memcpy(&tx_buff[frameheader_len],(uint8_t*)&cmd_id, cmd_len);

  /*****数据打包*****/
  memcpy(&tx_buff[frameheader_len+cmd_len], p_data, leng);
  Append_CRC16_Check_Sum(tx_buff,frame_length);  //一帧数据校验CRC16

  if (seq == 0xff) seq=0;
  else seq++;

  /*****数据上传*****/
// 清除传输完成标志位
  __HAL_UART_CLEAR_FLAG(&huart7, UART_FLAG_TC);
 HAL_UART_Transmit_DMA(&huart7, tx_buff, frame_length);
    // 处理错误
      // 等待传输完成
 while (__HAL_UART_GET_FLAG(&huart7, UART_FLAG_TC) == RESET);

}
