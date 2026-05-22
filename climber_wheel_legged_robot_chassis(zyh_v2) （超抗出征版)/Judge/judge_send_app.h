#ifndef __JUDGE_DATA_H__
#define __JUDGE_DATA_H__

#include "main.h"

/************************ 2024年3月Climber李川裁判系统解析  ***********************/
/*********************************************************************************/


#define MAX_SIZE          228    //上传数据最大的长度
#define frameheader_len  5       //帧头长度
#define cmd_len          2       //命令码长度
#define crc_len          2       //CRC16校验

//屏幕分辨率1920x1080
#define SCREEN_WIDTH 1080
#define SCREEN_LENGTH 1920

//图形参数绘制
typedef __packed struct
{
uint8_t figure_name[3]; 
uint32_t operate_tpye:3; 
uint32_t figure_tpye:3; 
uint32_t layer:4; 
uint32_t color:4; 
uint32_t details_a:9;
uint32_t details_b:9;
uint32_t width:10; 
uint32_t start_x:11; 
uint32_t start_y:11; 
uint32_t details_c:10; 
uint32_t details_d:11; 
uint32_t details_e:11; 
}interaction_figure_t;

//客户端绘制文字
typedef __packed struct 
{ 
uint16_t data_cmd_id;
uint16_t sender_id;
uint16_t receiver_id;
interaction_figure_t  interaction_word_t[1]; 
uint8_t data[30]; 
} robot_interaction_word_t; 

//客户端绘制七个图形
typedef __packed struct
{
uint16_t data_cmd_id;
uint16_t sender_id;
uint16_t receiver_id;
interaction_figure_t interaction_figure_t[7];	//自定义七个图形数据
}robot_interaction_figure_t;

void make_cir(robot_interaction_figure_t *custom_grapic_draw,uint8_t stun,uint8_t name[3],
			 uint8_t color, uint8_t type,uint8_t cir_type,uint8_t layer,uint16_t width, uint16_t x,uint16_t y,
			uint32_t start_angle,uint32_t end_angle,uint32_t r,uint32_t x_len,uint32_t y_len);
void make_line(robot_interaction_figure_t *custom_grapic_draw, 
uint8_t stun, uint8_t name[3], uint8_t color, uint8_t width
,uint8_t line_type,uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey,uint8_t type,uint8_t layer);
void make_word(robot_interaction_word_t*custom_grapic_draw,uint8_t name[3],uint8_t stun,
	uint8_t color,uint32_t size,uint8_t width,uint16_t sx,uint16_t sy,char *str,uint8_t word_type,uint8_t type,uint8_t layer);

void make_num(robot_interaction_figure_t *custom_grapic_draw,uint8_t name[3],uint8_t stun,
	uint8_t color,uint32_t size,uint8_t width,uint16_t sx,uint16_t sy,int32_t exfloat,uint8_t word_type,uint8_t type,uint8_t layer);

void make_7_grab_graph(robot_interaction_figure_t*custom_grapic_draw, uint8_t type);   //画瞄准线
void make2_7_grab_graph(robot_interaction_figure_t*custom_grapic_draw, uint8_t type);   
void change_7_grab_graph(robot_interaction_figure_t*custom_grapic_draw, uint8_t type);


void make_mode_word(robot_interaction_word_t*custom_grapic_draw,char *str,int type);
void make2_mode_word(robot_interaction_word_t*custom_grapic_draw,char *str,int type);
void make3_mode_word(robot_interaction_word_t*custom_grapic_draw,int type);
void make4_mode_word(robot_interaction_word_t*custom_grapic_draw,int type);

void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len);




void judge_painting_init(void);
void judge_painting(void);

#endif 
