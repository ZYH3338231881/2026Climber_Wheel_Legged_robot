#ifndef _COMPUTER_REC_H
#define _COMPUTER_REC_H
#include "chassis_task.h"
/************************ 2024年3月Climber李川裁判系统解析  ***********************/
/*********************************************************************************/
/************************ 2017年的裁判系统协议结构体 Start ***********************/

#ifdef  __DRIVER_GLOBALS
#define __DRIVER_EXT
#else
#define __DRIVER_EXT extern
#endif

#define JudgeBufferLength_PJ        29 //0X0004
#define JudgeBufferLength_Position  25 //0X0008
#define JudgeFrameLength_PJPosi_SH      54 //之所以是54是因为大疆将04 与 08两段数据安排到一帧里发送，但是校验却是其中一个包的校验
#define JudgeFrameLength_1_SH      54 

#define JudgeFrameLength_2_SH      11
#define JudgeFrameLength_3_SH      24

/************************ 2017年的裁判系统协议结构体 End ***********************/




/************************ 2024年3月Climber李川裁判系统解析  ***********************/
/*********************    包含所有接收信息   ************************************/
///******************  2024年的裁判系统协议结构体 Start  ***********************/
#define JudgeBufferLength       400
#define JudgeFrameHeader        0xA5        //帧头 
#define FrameHeader_Len         5 

#define CMD_LEN      2    //cmdid bytes
#define CRC_LEN      2    //crc16 bytes
#define HEADER_LEN   sizeof(tFrameHeader)  //frame length

typedef __packed struct
{
  uint8_t   SOF;//0xA5
  uint16_t  DataLenth;
	uint8_t   Seq;
  uint8_t   CRC8;
}tFrameHeader;

typedef enum 
{
	GameInfo = 0x0001,     
	RealBloodChangedData,  
	RealShootData,            
	SelfDefinedData =0x0100, 
	Wrong = 0x1301
}tCmdID; 


/***************命令码ID********************/

/* 

	ID: 0x0001  Byte:  3    比赛状态数据       			发送频率 1Hz      
	ID: 0x0002  Byte:  1    比赛结果数据         		比赛结束后发送      
	ID: 0x0003  Byte:  2    比赛机器人存活数据   		1Hz发送  
	ID: 0x0101  Byte:  4    场地事件数据   				事件改变后发送
	ID: 0x0102  Byte:  3    场地补给站动作标识数据    	动作改变后发送 
	ID: 0X0103  Byte:  2    场地补给站预约子弹数据      参赛队发送，10Hz 
	ID: 0X0201  Byte: 15    机器人状态数据        		10Hz
	ID: 0X0202  Byte: 14    实时功率热量数据   			50Hz       
	ID: 0x0203  Byte: 16    机器人位置数据           	10Hz
	ID: 0x0204  Byte:  1    机器人增益数据           	增益状态改变后发送
	ID: 0x0205  Byte:  3    空中机器人能量状态数据      10Hz
	ID: 0x0206  Byte:  1    伤害状态数据           		伤害发生后发送
	ID: 0x0207  Byte:  6    实时射击数据           		子弹发射后发送
	ID: 0x0301  Byte:  n    机器人间交互数据           	发送方触发发送,10Hz
	
*/

typedef enum
{
	ID_game_state       					= 0x0001,//比赛状态数据
	ID_game_result 	   						= 0x0002,//比赛结果数据
	ID_game_robot_HP       				= 0x0003,//比赛机器人存活数据
	
	ID_event_data  								= 0x0101,//场地事件数据 
	ID_supply_projectile_action   = 0x0102,//场地补给站动作标识数据
	ID_referee_warning						=	0x0104,//裁判警告
	ID_dart_remaining_time				= 0x0105,//飞镖发射相关数据
	
	Robot_Status_ID               =0x0201,//机器人状态  等级
	power_heat_data_ID   	        =0x0202,//枪口热量 底盘功率
	robot_pos_ID   	 					    =0x0203,//机器人位置数据
	robot_buff_ID   	 					  =0x0204,//buff
	robot_hurt_ID                 =0x0206,//伤害类型
	shoot_data_ID	      					=0x0207,//射频射速
	projectile_allowance_ID	   		=0x0208,//允许发弹量	
	rfid_status_ID   	 					  =0x0209,//机器人 RFID 模块状态
	dart_client_cmd_ID            =0x020A,//飞镖选手端指令数据
	ground_robot_position_ID	   	=0x020B,//地面机器人位置数据	
	radar_mark_data_ID            =0x020C,//雷达标记进度数据
	sentry_info_ID   	      	 	  =0x020D,//哨兵自主决策信息同步
	radar_info_t_ID   	 					=0x020E,//雷达自主决策信息同步

	student_interactive_header_ID     =0x0301,
	controller_header_ID     =0x0302,
	key_inpict_ID     =0x0304,
	
} Judege_Cmd_ID;
//帧头




//比赛状态数据 命令码ID 0x0001  发送频率：1Hz
typedef __packed struct
{
	uint8_t game_type:4;     //0-3bit:比赛类型 1：大师赛 2:单项赛 3：人工智能挑战赛 4；3V3 5:1V1
	uint8_t game_progress:4; //4-7bit:比赛阶段 0：未开始比赛 1：准备阶段 2：自检阶段 3:5s倒计时 4：对战中 5：比赛结算中
	uint16_t stage_remain_time; //当前阶段剩余时间 /s
	
	uint64_t SyncTimeStamp;
}ext_game_status_t;

//比赛结果数据 命令码ID 0x0002  发送频率：比赛结束后发送
typedef __packed struct
{
	uint8_t winner;  //0:平局 1:红方胜利 2：蓝方胜利
}ext_game_result_t;

//机器人血量数据 命令码ID 0x0003  发送频率：1Hz
typedef __packed struct
{
	uint16_t red_1_robot_HP; //红1英雄机器人血量  未上场以及罚下血量为0
	uint16_t red_2_robot_HP; //红2工程机器人血量 
	uint16_t red_3_robot_HP; //红3步兵机器人血量 
	uint16_t red_4_robot_HP; //红4步兵机器人血量 
	uint16_t red_5_robot_HP; //红5步兵机器人血量 
	uint16_t red_7_robot_HP; //红5哨兵机器人血量 
	
	uint16_t red_outpost_HP; //红方前哨站
	uint16_t red_base_HP; //红方基地
	
	uint16_t blue_1_robot_HP; //蓝1英雄机器人血量  未上场以及罚下血量为0
	uint16_t blue_2_robot_HP; //蓝2工程机器人血量 
	uint16_t blue_3_robot_HP; //蓝3步兵机器人血量 
	uint16_t blue_4_robot_HP; //蓝4步兵机器人血量 
	uint16_t blue_5_robot_HP; //蓝5步兵机器人血量 
	uint16_t blue_7_robot_HP; //蓝5哨兵机器人血量 
	
	uint16_t blue_outpost_HP; //蓝方前哨站
	uint16_t blue_base_HP; //蓝方基地
}ext_game_robot_HP_t;

//场地信息      命令码ID 0x0101
typedef __packed struct
{
	uint32_t event_type;
}ext_event_data_t;

//补给站动作标识 命令码ID 0x0102  发送频率：动作改变后发送 发送范围：己方机器人
typedef __packed struct
{
 uint8_t reserved;
 uint8_t supply_robot_id;
 uint8_t supply_projectile_step;
 uint8_t supply_projectile_num;
}ext_supply_projectile_action_t;

//裁判警告信息 命令码ID 0x0104  发送频率：发生警告后发送 
typedef __packed struct
{
 uint8_t level;
 uint8_t offending_robot_id;
 uint8_t count;
}ext_referee_warning_t;


//飞镖发射口倒计时 命令码ID 0x0105  发送频率：1Hz  发送范围：己方机器人
typedef __packed struct
{
 uint8_t dart_remaining_time;
 uint16_t dart_info;
}ext_dart_remaining_time_t;


/*比赛机器人状态   命令码ID 0x0201 */
typedef __packed struct
{	
uint8_t robot_id;
uint8_t robot_level;
uint16_t current_HP; 
uint16_t maximum_HP;
uint16_t shooter_barrel_cooling_value;
uint16_t shooter_barrel_heat_limit;
uint16_t chassis_power_limit; 
uint8_t power_management_gimbal_output : 1;
uint8_t power_management_chassis_output : 1; 
uint8_t power_management_shooter_output : 1;

} ext_game_robot_state_t;

/*底盘功率及枪口热量 命令码ID 0x0202 */
typedef __packed struct
{
	uint16_t chassis_volt;//底盘输出电压  单位 毫伏
	uint16_t chassis_current;//底盘输出电流  单位毫安
	float    chassis_power;//底盘输出功率  单位 W
	uint16_t buffer_energy;//底盘功率缓冲  单位J
	uint16_t shooter_17_heat1;//17mm枪口热量
	uint16_t shooter_17_heat2;//17mm枪口热量
	uint16_t shooter_42_heat3;//42mm枪口热量
} ext_power_heat_data_t;

/*底盘功率及枪口热量 命令码ID 0x0203 */
typedef __packed struct
{
 float x;
 float y;
 float angle;
}ext_robot_pos_t;

//机器人增益 命令码ID 0x0204  发送频率：1Hz 
typedef __packed struct
{
 uint8_t recovery_buff;
 uint8_t cooling_buff;
 uint8_t defence_buff;
 uint8_t vulnerability_buff;
 uint16_t attack_buff;
}ext_buff_t;

//空中支援时间数据 命令码ID 0x0205  发送频率：1Hz 
typedef __packed struct
{
 uint8_t airforce_status;
 uint8_t time_remain;
}ext_air_support_data_t;

/*伤害类型   命令码ID 0x0206 */
typedef __packed struct
{
uint8_t armor_id : 4;
uint8_t HP_deduction_reason : 4;
} ext_robot_hurt_t;

/*实时射击数据  命令码ID 0x0207*/
typedef __packed struct
{
 uint8_t bullet_type;
 uint8_t shooter_number;
 uint8_t launching_frequency;
 float initial_speed;
}ext_shoot_data_t;

/*允许发弹量  命令码ID 0x0208*/
typedef __packed struct
{
 uint16_t projectile_allowance_17mm;
 uint16_t projectile_allowance_42mm;
 uint16_t remaining_gold_coin;
}ext_projectile_allowance_t;

/*机器人 RFID 模块状态 命令码ID 0x0209*/
typedef __packed struct
{
 uint32_t rfid_status;
}ext_rfid_status_t;

/*飞镖选手端指令数据  命令码ID 0x020A*/
typedef __packed struct
{
 uint8_t dart_launch_opening_status;
 uint8_t reserved;
 uint16_t target_change_time;
 uint16_t latest_launch_cmd_time;
}ext_dart_client_cmd_t;

/*地面机器人位置数据  命令码ID 0x020B*/
typedef __packed struct
{
 float hero_x;
 float hero_y;
 float engineer_x;
 float engineer_y;
 float standard_3_x;
 float standard_3_y;
 float standard_4_x;
 float standard_4_y;
 float standard_5_x;
 float standard_5_y;
}ext_ground_robot_position_t;

/*雷达标记进度数据  命令码ID 0x020C*/
typedef __packed struct
{
 uint8_t mark_hero_progress;
 uint8_t mark_engineer_progress;
 uint8_t mark_standard_3_progress;
 uint8_t mark_standard_4_progress;
 uint8_t mark_standard_5_progress;
 uint8_t mark_sentry_progress;
}ext_radar_mark_data_t;

/*哨兵自主决策信息同步  命令码ID 0x020D*/
typedef __packed struct
{
 uint32_t sentry_info;
}ext_sentry_info_t;

/*雷达自主决策信息同步  命令码ID 0x020E*/
typedef __packed struct
{
 uint8_t radar_info;
}ext_radar_info_t;





/*用户自定义部分可以画UI显示在屏幕上  命令码ID 0x0301*/
/*0-1 数据内容ID 0xD180
	2-3 发送者的ID
*/

typedef __packed struct
{
	uint16_t rxCmdId;
	uint16_t data_id;
	uint16_t send_id;
	uint16_t receive_id;
} id_data_t;

typedef __packed struct
{
	tFrameHeader Header;
	id_data_t    id;
	float data1;
	float data2;
	float data3;
	uint8_t masks;
	uint16_t crc_16;
} ext_client_custom_data_t;

//机器人之间相互通信  子内容ID:0x0200~0x02FF       注意，其他UI结构体定义在judge_send_app.h中了
typedef __packed struct
{
	uint8_t data[112];
} robot_interactive_data_t;

//哨兵自主决策指令  子内容ID:0x0120
typedef __packed struct
{
uint32_t sentry_cmd; 
} ext_sentry_cmd_t;

//雷达自主决策指令  子内容ID:0x0121
typedef __packed struct
{
uint32_t sentry_cmd; 
}ext_radar_cmd_t;

//自定义控制器x0302
typedef __packed struct
{
	uint8_t data[30];
} ext_robot_controller_data_t;


/*选手端小地图交互数据  命令码ID 0x0303*/
typedef __packed struct
{
float target_position_x;
float target_position_y;
uint8_t cmd_keyboard;
uint8_t target_robot_id;
uint8_t cmd_source;
}ext_map_command_t;

/*图传键鼠详细见图传键鼠串口文件  命令码ID 0x0304*/

/*选手端小地图接收雷达数据  命令码ID 0x0305*/
typedef __packed struct
{
uint16_t target_robot_id;
float target_position_x;
float target_position_y;
}ext_map_robot_data_t;

/*选手端小地图接收哨兵数据  命令码ID 0x0307*/
typedef __packed struct
{
uint8_t intention;
uint16_t start_position_x;
uint16_t start_position_y;
int8_t delta_x[49];
int8_t delta_y[49];
uint16_t sender_id;
}ext_map_data_t;

/*选手端小地图接收机器人数据  命令码ID 0x0308*/
typedef __packed struct
{ 
uint16_t sender_id;
uint16_t receiver_id;
uint8_t user_data[30];
} ext_custom_info_t;

typedef struct
{
int16_t mouse_x;
int16_t mouse_y;
int16_t mouse_z;
int8_t left_button_down;
int8_t right_button_down;
uint16_t keyboard_value;
uint16_t reserved;
} ext_robot_command_t;


typedef  struct
{
  tFrameHeader          frameHeader;
  uint16_t              rxCmdId;
	
	ext_game_status_t     GameState;				        	//0x0001
	ext_game_result_t     GameResult;			        	  //0x0002
	ext_game_robot_HP_t   GameRobotHP;			          //0x0003
	
	ext_event_data_t        			   EventData;					        //0x0101		
	ext_supply_projectile_action_t	 SupplyProjectileAction;		//0x0102
	ext_referee_warning_t            RefereeWarn;               //0x0104
	ext_dart_remaining_time_t        DartRemain;                //0x0105

	
	ext_game_robot_state_t  robot_status_t;			 //0x0201
  ext_power_heat_data_t   power_heat_data_t;	 //0x0202
	ext_robot_pos_t					robot_pos_t;				 //0x0203
	ext_buff_t              buff_t;			   			 //0x0204
	ext_robot_hurt_t        hurt_data_t; 			   //0x0206
  ext_shoot_data_t		    shoot_data_t;		     //0x0207
  ext_projectile_allowance_t projectile_allowance_t; //0x0208
	ext_rfid_status_t				rfid_status_t;			 //0x0209
  ext_dart_client_cmd_t   dart_client_cmd_t;   //0x020A
  ext_ground_robot_position_t ground_robot_position_t; //0x020B
  ext_radar_mark_data_t   radar_mark_data_t;   //0x020C
	ext_sentry_info_t 			sentry_info_t;       //0x020D
	ext_radar_info_t 				radar_info_t;   	   //0x020E
	
	
	ext_robot_command_t     robot_key_t;
	ext_robot_controller_data_t controller_data_t;
	ext_client_custom_data_t robot_data_t;
	ext_client_custom_data_t userinfo;
}JudgementDataTypedef;

///******************  2024年的裁判系统协议结构体 end  ***********************/




typedef __packed struct
{
	float data1;
	float data2;
	float data3;
	uint8_t mask;
}tSelfDefineInfo;  

typedef  __packed struct
{
	float data1;
	float data2;
	float data3;
	uint8_t mask;
}SelfDefineInfo_t;   //学生上传自定义数据 (0x0005)  

typedef  struct
{
  tFrameHeader          frameHeader;
  uint16_t              rxCmdId;
		__packed union 
	{
  ext_power_heat_data_t power_heat_data_t;
  ext_shoot_data_t		  shoot_data_t;
	ext_robot_hurt_t      robot_hurt_t;
	ext_game_robot_state_t game_robot_state_t;
  robot_interactive_data_t robot_data_t;
	ext_client_custom_data_t userinfo;
		}Data;
	uint16_t        CRC16;
	uint16_t        CRC16_2 ;
}FRAME;

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	tSelfDefineInfo SelfDefineInfo; 
	uint16_t        CRC16;         
}tFrame;  



void Send_FrameData(uint16_t cmdid, uint8_t * pchMessage,uint16_t dwLength);
void client_send(uint8_t * data);
void client_init(uint8_t * data1,uint8_t * data2);
void get_chassis_power_and_buffer(float *power, float *buffer, uint16_t *powmax);
void judgeCalculate(uint8_t JudgeDataBuffer[JudgeBufferLength]);//裁判系统解算
void Computer_rec_init(void);
void Usart7Receive_IDLE(void);

#endif