#include "computer_rec.h"
#include "judge_paint_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "judge_send_app.h"
#include "math.h"
#include "chassis_task.h"
#include <string.h>
#include <stdio.h>       // 用于 sprintf 格式化角度数字
#include "bsp_rc.h"      // 包含遥控器头文件
extern Keyboard_Data keyboard_data;

extern RC_ctrl_t rc_ctrl; // 声明遥控器全局结构体

// ==========================================
// 外部核心数据引用
// ==========================================
extern JudgementDataTypedef JudgementData;
extern Chassis_s CHASSIS;               
extern CTOM_message_t ctom_message;     

// ==========================================
// 1. 图元名称分配 (3字节 ID，绝对不能重复)
// ==========================================
/* --- 静态背景 Pack 1 (全息火控准星) --- */
uint8_t bg_aim_c[3]      = {0, 0, 1}; 
uint8_t bg_aim_in[3]     = {0, 0, 2}; 
uint8_t bg_aim_out[3]    = {0, 0, 3}; 
uint8_t bg_aim_l[3]      = {0, 0, 4}; 
uint8_t bg_aim_r[3]      = {0, 0, 5}; 
uint8_t bg_aim_t[3]      = {0, 0, 6}; 
uint8_t bg_aim_b[3]      = {0, 0, 7}; 

/* --- 静态背景 Pack 2 (标尺与外框) --- */
uint8_t bg_scale_1[3]    = {0, 1, 1}; 
uint8_t bg_scale_2[3]    = {0, 1, 2}; 
uint8_t bg_scale_3[3]    = {0, 1, 3}; 
uint8_t bg_compass_gim[3]= {0, 1, 4}; 
uint8_t bg_status_box[3] = {0, 1, 5}; 
uint8_t bg_cap_box[3]    = {0, 1, 6}; 
uint8_t bg_leg_l_line[3] = {0, 1, 7}; // 左腿标注辅助线

/* --- 静态背景 Pack 3 (【巨型紫色大框】、车身外八字线) --- */
uint8_t bg_leg_r_line[3] = {0, 2, 1}; // 右腿标注辅助线
uint8_t bg_body_l[3]     = {0, 2, 2}; 
uint8_t bg_body_r[3]     = {0, 2, 3}; 
uint8_t bg_main_box[3]   = {0, 2, 4}; // 巨型紫色战术显示框

/* --- 侧视图机甲拟真图元 Pack 4 --- */
uint8_t dyn_chassis[3]   = {0, 3, 1}; 
uint8_t dyn_thigh_L[3]   = {0, 3, 2}; 
uint8_t dyn_calf_L[3]    = {0, 3, 3}; 
uint8_t dyn_thigh_R[3]   = {0, 3, 4}; 
uint8_t dyn_calf_R[3]    = {0, 3, 5}; 
uint8_t dyn_horiz_L[3]   = {0, 3, 6}; 
uint8_t dyn_horiz_R[3]   = {0, 3, 7}; 

/* --- 旋转巨型底盘图元 Pack 5 --- */
uint8_t dyn_chas_l1[3]   = {0, 4, 1}; 
uint8_t dyn_chas_l2[3]   = {0, 4, 2}; 
uint8_t dyn_chas_l3[3]   = {0, 4, 3}; 
uint8_t dyn_chas_l4[3]   = {0, 4, 4}; 
uint8_t dyn_chas_dir[3]  = {0, 4, 5}; 

/* --- 其他动态图元 Pack 6 --- */
uint8_t lit_mode[3]      = {0, 5, 1}; 
uint8_t lit_aim[3]       = {0, 5, 2}; 
uint8_t lit_fric[3]      = {0, 5, 3}; 
uint8_t dyn_slip_box[3]  = {0, 5, 4}; 
uint8_t dyn_cap_fill[3]  = {0, 5, 5}; 
uint8_t dyn_compass[3]   = {0, 5, 6}; 
uint8_t dyn_compass_tip[3]={0, 5, 7}; 

/* --- 文字图元 Pack 7 静态发送 (防丢失) --- */
uint8_t txt_mode[3]      = {0, 6, 1}; 
uint8_t txt_aim[3]       = {0, 6, 2}; 
uint8_t txt_fric[3]      = {0, 6, 3}; 
uint8_t txt_slip[3]      = {0, 6, 4}; 
uint8_t txt_gim_f[3]     = {0, 6, 5}; 
uint8_t txt_leg_L_leg[3] = {0, 6, 6}; 
uint8_t txt_leg_R_leg[3] = {0, 6, 7}; 

/* --- 静态文字 Pack 8 (【超级电容描述】移入) --- */
uint8_t txt_cap_name_1[3]= {0, 7, 1}; // SUPER
uint8_t txt_cap_name_2[3]= {0, 7, 2}; // CAP
uint8_t txt_cap_per[3]   = {0, 7, 3}; // 单位%

/* --- 模块专用动态文字 Pack 10 --- */
uint8_t txt_gim_yaw[3]   = {0, 8, 8}; // 黄色代码1
uint8_t txt_gim_pitch[3] = {0, 8, 9}; // 黄色代码1

// ==========================================
// 边沿触发记忆变量与重发计数器
// ==========================================
uint8_t last_ui_mode = 255;
uint8_t last_ui_aim  = 255;
uint8_t last_ui_fric = 255;
uint8_t last_ui_motor_offline = 255;

int16_t last_ui_yaw   = 9999;
int16_t last_ui_pitch = 9999;

uint8_t send_cnt_mode = 0;
uint8_t send_cnt_aim  = 0;
uint8_t send_cnt_fric = 0;
uint8_t send_cnt_motor_offline = 0;

// ==========================================
// ?? 参数调节区
// ==========================================
#define BODY_X       1550    
#define BODY_Y       850     
#define BODY_LEN     120     
#define LEG_SCALE    800.0f  

#define THIGH_PX     165.0f  
#define CALF_PX      200.0f  
#define THETA_DIR    1.0f    
#define KNEE_DIR     1.0f    

#define COMPASS_X    960     
//#define COMPASS_Y    200
#define COMPASS_R    60      

#define YAW_DIR      -1.0f   

robot_interaction_figure_t pack_figs;
robot_interaction_word_t   pack_word;

// ==========================================
// 2. 静态 UI 初始化
// ==========================================
void judge_painting_init(void)  
{
    uint16_t sender = JudgementData.robot_status_t.robot_id;
    uint16_t receiver = 0x100 + sender;
    
    last_ui_mode = 255; last_ui_aim = 255; last_ui_fric = 255; 
    last_ui_yaw = 9999; last_ui_pitch = 9999; 
    send_cnt_mode = 3; send_cnt_aim = 3; send_cnt_fric = 3;

    /* --- 第 1 包：全息复合式火控准星 --- */
    memset(&pack_figs, 0, sizeof(pack_figs));
//    make_cir(&pack_figs, 0, bg_aim_c,   3, 4, 2, 1, 3, 960, 440, 0, 360, 4, 0, 0); 
//    make_cir(&pack_figs, 1, bg_aim_in,  6, 2, 2, 1, 2, 960, 440, 0, 360, 40, 0, 0); 
//    make_cir(&pack_figs, 2, bg_aim_out, 8, 1, 2, 1, 1, 960, 440, 0, 360, 120, 0, 0); 
    
    make_line(&pack_figs, 3, bg_aim_l,  4, 3, 0, 950, 470, 910, 470, 1, 1); 
    make_line(&pack_figs, 4, bg_aim_r,  4, 3, 0, 970,470, 1010,470, 1, 1); 
    make_line(&pack_figs, 5, bg_aim_t,  4, 3, 0, 960, 480, 960, 520, 1, 1); 
    make_line(&pack_figs, 6, bg_aim_b,  4, 3, 0, 960, 460, 960, 420, 1, 1); 
    
    pack_figs.data_cmd_id = 0x0104; pack_figs.sender_id = sender; pack_figs.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_figs, sizeof(pack_figs));
    vTaskDelay(30); 

    /* --- 第 2 包：标尺与外框、罗盘参照线、腿部辅助线 --- */
    memset(&pack_figs, 0, sizeof(pack_figs));
//    make_line(&pack_figs, 0, bg_scale_1,    4, 2, 0, 945, 420, 975, 420, 1, 1);
//    make_line(&pack_figs, 1, bg_scale_2,    4, 2, 0, 935, 360, 985, 360, 1, 1);
//    make_line(&pack_figs, 2, bg_scale_3,    4, 2, 0, 920, 300, 1000,300, 1, 1);
    
    make_line(&pack_figs, 3, bg_compass_gim,2, 4, 0, COMPASS_X, COMPASS_Y, COMPASS_X, COMPASS_Y + COMPASS_R + 10, 1, 1); 
    make_line(&pack_figs, 4, bg_status_box, 1, 2, 1, 50, 620, 400, 880, 1, 1); 
    make_line(&pack_figs, 5, bg_cap_box,    8, 2, 1, 1740,300, 1780,700, 1, 1);
    make_line(&pack_figs, 6, bg_leg_l_line, 6, 2, 0, BODY_X - 120, BODY_Y - 160, BODY_X - 40,  BODY_Y - 160, 1, 1);
    
    pack_figs.data_cmd_id = 0x0104; pack_figs.sender_id = sender; pack_figs.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_figs, sizeof(pack_figs));
    vTaskDelay(30); 

    /* --- 第 3 包：【巨型紫色大框】与右腿辅助线 --- */
    memset(&pack_figs, 0, sizeof(pack_figs));
    make_line(&pack_figs, 0, bg_leg_r_line, 1, 2, 0, BODY_X + 30,  BODY_Y - 160, BODY_X + 110, BODY_Y - 160, 1, 1);
    make_line(&pack_figs, 1, bg_body_l,     6, 8, 0, 480, 0, 730, 300, 1, 1); 
    make_line(&pack_figs, 2, bg_body_r,     6, 8, 0, 1440, 0, 1190, 300, 1, 1); 
    make_line(&pack_figs, 3, bg_main_box,   2, 3, 1, 960-500, 480-150, 960+500, 480+300, 1, 1);
    
    pack_figs.interaction_figure_t[4].operate_tpye = 0;
    pack_figs.interaction_figure_t[5].operate_tpye = 0;
    pack_figs.interaction_figure_t[6].operate_tpye = 0;
    pack_figs.data_cmd_id = 0x0104; pack_figs.sender_id = sender; pack_figs.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_figs, sizeof(pack_figs));
    vTaskDelay(30); 

    /* --- 第 4 包：侧视图机甲 + 战机姿态仪占位 --- */
    memset(&pack_figs, 0, sizeof(pack_figs));
    make_line(&pack_figs, 0, dyn_chassis, 8, 8, 0, BODY_X - BODY_LEN, BODY_Y, BODY_X + BODY_LEN, BODY_Y, 1, 2); 
    make_line(&pack_figs, 1, dyn_thigh_L, 6, 6, 0, BODY_X, BODY_Y, BODY_X, BODY_Y-100, 1, 2); 
    make_line(&pack_figs, 2, dyn_calf_L,  6, 4, 0, BODY_X, BODY_Y-100, BODY_X, BODY_Y-250, 1, 2); 
    make_line(&pack_figs, 3, dyn_thigh_R, 1, 6, 0, BODY_X, BODY_Y, BODY_X, BODY_Y-100, 1, 2); 
    make_line(&pack_figs, 4, dyn_calf_R,  1, 4, 0, BODY_X, BODY_Y-100, BODY_X, BODY_Y-250, 1, 2); 
    
    make_line(&pack_figs, 5, dyn_horiz_L, 2, 3, 0, 960-200, 540, 960-80, 540, 1, 2); 
    make_line(&pack_figs, 6, dyn_horiz_R, 2, 3, 0, 960+80,  540, 960+200, 540, 1, 2); 
    
    pack_figs.data_cmd_id = 0x0104; pack_figs.sender_id = sender; pack_figs.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_figs, sizeof(pack_figs));
    vTaskDelay(30); 

    /* --- 第 5 包：旋转底盘占位 --- */
    memset(&pack_figs, 0, sizeof(pack_figs));
    make_line(&pack_figs, 0, dyn_chas_l1,   6, 3, 0, COMPASS_X, COMPASS_Y, COMPASS_X, COMPASS_Y, 1, 2);
    make_line(&pack_figs, 1, dyn_chas_l2,   6, 2, 0, COMPASS_X, COMPASS_Y, COMPASS_X, COMPASS_Y, 1, 2);
    make_line(&pack_figs, 2, dyn_chas_l3,   6, 2, 0, COMPASS_X, COMPASS_Y, COMPASS_X, COMPASS_Y, 1, 2);
    make_line(&pack_figs, 3, dyn_chas_l4,   6, 2, 0, COMPASS_X, COMPASS_Y, COMPASS_X, COMPASS_Y, 1, 2);
    make_line(&pack_figs, 4, dyn_chas_dir,  6, 4, 0, COMPASS_X, COMPASS_Y, COMPASS_X, COMPASS_Y, 1, 2);
    pack_figs.interaction_figure_t[5].operate_tpye = 0; 
    pack_figs.interaction_figure_t[6].operate_tpye = 0; 
    pack_figs.data_cmd_id = 0x0104; pack_figs.sender_id = sender; pack_figs.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_figs, sizeof(pack_figs));
    vTaskDelay(30); 

    /* --- 第 6 包：指示灯与剩余占位 --- */
    memset(&pack_figs, 0, sizeof(pack_figs));
    make_line(&pack_figs, 0, lit_mode,        8, 15, 0, 100, 820, 100, 835, 1, 2);
    make_line(&pack_figs, 1, lit_aim,         8, 15, 0, 100, 750, 100, 765, 1, 2);
    make_line(&pack_figs, 2, lit_fric,        8, 15, 0, 100, 680, 100, 695, 1, 2);
    
    // 【修复】：初始化时，把打滑提示框直接塞进原点 (0,0) 完美隐藏
    make_line(&pack_figs, 3, dyn_slip_box,    3, 0,  1, 0, 0, 0, 0, 1, 2);  
    
    make_line(&pack_figs, 4, dyn_cap_fill,    2, 36, 0, 1760, 302, 1760, 698, 1, 2); 
    make_line(&pack_figs, 5, dyn_compass,     3, 6, 0, COMPASS_X, COMPASS_Y, COMPASS_X, COMPASS_Y+COMPASS_R, 1, 2);
    make_cir (&pack_figs, 6, dyn_compass_tip, 3, 1, 2, 1, 10, COMPASS_X, COMPASS_Y+COMPASS_R, 0, 360, 6, 0, 0);
    
    pack_figs.data_cmd_id = 0x0104; pack_figs.sender_id = sender; pack_figs.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_figs, sizeof(pack_figs));
    vTaskDelay(30); 

    /* --- 第 7 包文字：静态发送主UI描述 --- */
    memset(&pack_word, 0, sizeof(pack_word));
    make_word(&pack_word, txt_mode, 0, 8, 24, 3, 140, 820, "MD: WAIT", 7, 1, 2);
    pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));  vTaskDelay(80); 

    memset(&pack_word, 0, sizeof(pack_word));
    make_word(&pack_word, txt_aim,  0, 8, 24, 3, 140, 750, "AIM: WAIT", 7, 1, 2);
    pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));  vTaskDelay(80);

    memset(&pack_word, 0, sizeof(pack_word));
    make_word(&pack_word, txt_fric, 0, 8, 24, 3, 140, 680, "FRIC: WAIT", 7, 1, 2);
    pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));  vTaskDelay(80);

    memset(&pack_word, 0, sizeof(pack_word));
    make_word(&pack_word, txt_leg_L_leg, 0, 6, 16, 2, BODY_X - 120, BODY_Y - 180, "L(CYAN)", 7, 1, 1); 
    pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));  vTaskDelay(80);

    memset(&pack_word, 0, sizeof(pack_word));
    make_word(&pack_word, txt_leg_R_leg, 0, 1, 16, 2, BODY_X + 30, BODY_Y - 180, "R(YELLOW)", 7, 1, 1); 
    pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));  vTaskDelay(80);

    memset(&pack_word, 0, sizeof(pack_word));
    make_word(&pack_word, txt_gim_f, 0, 2, 14, 2, COMPASS_X-30, COMPASS_Y+COMPASS_R+28, "GIM_FWD", 7, 1, 1);
    pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));  vTaskDelay(80);

    // 【修复】：初始化时，直接填入纯空格来完美隐藏文字！
    memset(&pack_word, 0, sizeof(pack_word));
    make_word(&pack_word, txt_slip, 0, 3, 28, 4, 820, 700, "           ", 7, 1, 3); 
    pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));  vTaskDelay(80);

    /* --- 第 8 包文字：静态发送【超级电容描述】 --- */
    memset(&pack_word, 0, sizeof(pack_word));
    make_word(&pack_word, txt_cap_name_1, 0, 8, 20, 2, 1610, 500+60, "SUPER", 7, 1, 1);
    pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));  vTaskDelay(80); 

    memset(&pack_word, 0, sizeof(pack_word));
    make_word(&pack_word, txt_cap_name_2, 0, 8, 20, 2, 1610, 500+20, "CAP", 7, 1, 1);
    pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));  vTaskDelay(80);
    
    memset(&pack_word, 0, sizeof(pack_word));
    make_word(&pack_word, txt_cap_per,    0, 2, 36, 4, 1800, 500-20, "%", 7, 1, 3);
    pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));  vTaskDelay(80);

    /* --- 第 10 包：模块专用【Yaw/Pitch 静态文字初始化】 --- */
    memset(&pack_word, 0, sizeof(pack_word));
    make_word(&pack_word, txt_gim_yaw,   0, 1, 20, 2, 1140, 560, "YAW: 0", 7, 1, 2);
    pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));  vTaskDelay(80);

    memset(&pack_word, 0, sizeof(pack_word));
    make_word(&pack_word, txt_gim_pitch, 0, 1, 20, 2, 1140, 520, "PIT: 0", 7, 1, 2);
    pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));  vTaskDelay(80);
}


// ==========================================
// 3. 全息动态更新 (高频调用)
// ==========================================
void judge_painting_update(void)
{
    uint16_t sender = JudgementData.robot_status_t.robot_id;
    uint16_t receiver = 0x100 + sender;

    // --------------------------------------------------------
    // 【模块 1】：文字防丢包连发更新
    // --------------------------------------------------------
    if (CHASSIS.mode != last_ui_mode) {
        last_ui_mode = CHASSIS.mode;
        send_cnt_mode = 3; 
    }
    if (send_cnt_mode > 0) {
        uint8_t color; char str[15];
        if (last_ui_mode == CHASSIS_FREE)        { color = 2; strcpy(str, "MD: FREE"); }
        else if (last_ui_mode == CHASSIS_SAFE)   { color = 8; strcpy(str, "MD: SAFE"); }
//        else if (last_ui_mode == CHASSIS_OFF_HOOK) { color = 3; strcpy(str, "MD: HOOK"); }
//        else if (last_ui_mode == CHASSIS_CUSTOM) { color = 2; strcpy(str, "MD: CLIMB"); }
        else                                     { color = 8; strcpy(str, "MD: OTHER"); }
        
        memset(&pack_word, 0, sizeof(pack_word));
        make_word(&pack_word, txt_mode, 0, color, 24, 3, 140, 820, str, 7, 2, 2); 
        pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
        referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));
        vTaskDelay(5); 
        send_cnt_mode--; 
    }

//    if (ctom_message.auto_aim_active != last_ui_aim) {
//        last_ui_aim = ctom_message.auto_aim_active;
//        send_cnt_aim = 3;
//    }
    if (send_cnt_aim > 0) {
        uint8_t color = last_ui_aim ? 3 : 8; 
        char *str = last_ui_aim ? "AIM: ON " : "AIM: OFF";
        memset(&pack_word, 0, sizeof(pack_word));
        make_word(&pack_word, txt_aim, 0, color, 24, 3, 140, 750, str, 7, 2, 2);
        pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
        referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));
        vTaskDelay(5);
        send_cnt_aim--;
    }

//    if (ctom_message.fric_bool != last_ui_fric) {
//        last_ui_fric = ctom_message.fric_bool;
//        send_cnt_fric = 3;
//    }
    if (send_cnt_fric > 0) {
        uint8_t color = last_ui_fric ? 2 : 8; 
        char *str = last_ui_fric ? "FRIC: ON " : "FRIC: OFF";
        memset(&pack_word, 0, sizeof(pack_word));
        make_word(&pack_word, txt_fric, 0, color, 24, 3, 140, 680, str, 7, 2, 2);
        pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
        referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));
        vTaskDelay(5);
        send_cnt_fric--;
    }


    uint8_t is_motor_offline=0;
    if(ctom_message.gimbal_motor_offline
        ||CHASSIS.joint_motor[0].fdb.state==0
        ||CHASSIS.joint_motor[1].fdb.state==0
        ||CHASSIS.joint_motor[2].fdb.state==0
        ||CHASSIS.joint_motor[3].fdb.state==0)
    {
        is_motor_offline=1;        
    }

    if (is_motor_offline != last_ui_motor_offline) {
        last_ui_motor_offline = is_motor_offline;
        send_cnt_motor_offline = 3;
    }
    if(send_cnt_motor_offline>0)
    {
        uint8_t color = last_ui_motor_offline ? 3 : 8; 
        char *str = last_ui_motor_offline ? " MTOR_OFFLINE! " : "              "; 
         memset(&pack_word, 0, sizeof(pack_word));
        make_word(&pack_word, txt_slip, 0, color, 28, 4, 820, 700, str, 7, 2, 3);
        pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
        referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));
    }

//    // --------------------------------------------------------
//    // 【模块 4】：Gyro Reminder
//    // --------------------------------------------------------
//    static uint8_t last_gyro_remind = 0;
//    uint8_t current_gyro_remind = (fabsf(CHASSIS.x_dot_obv) < 0.5f) ? 1 : 0;
//    static uint8_t gyro_send_cnt = 0;
//    
//    if (current_gyro_remind != last_gyro_remind) {
//        last_gyro_remind = current_gyro_remind;
//        gyro_send_cnt = 3; // 状态变化时连发3次
//    }
//    
//    if (gyro_send_cnt > 0) {
//        uint8_t color = current_gyro_remind ? 3 : 8;
//        char *str = current_gyro_remind ? "Open Gyro" : "          ";
//        memset(&pack_word, 0, sizeof(pack_word));
//        make_word(&pack_word, txt_slip, 0, color, 28, 4, 820, 700, str, 7, 2, 3);
//        pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
//        referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));
//        gyro_send_cnt--;
//        vTaskDelay(5); // 间隔发送
//    }



    // --------------------------------------------------------
    // 【模块 2】：动态数据包图元 8.5 (Yaw/Pitch)
    // --------------------------------------------------------
    int16_t current_yaw   = (int16_t)ctom_message.gimbal_imu_yaw; 
    int16_t current_pitch = (int16_t)ctom_message.gimbal_imu_pitch; 
    
    if (current_yaw != last_ui_yaw) {
        last_ui_yaw = current_yaw;
        char str[15];
        sprintf(str, "YAW: %d", current_yaw);
        memset(&pack_word, 0, sizeof(pack_word));
        make_word(&pack_word, txt_gim_yaw, 0, 1, 20, 2, 1140, 560, str, 7, 2, 2); 
        pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
        referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));
        vTaskDelay(5);
    }
    
    if (current_pitch != last_ui_pitch) {
        last_ui_pitch = current_pitch;
        char str[15];
        sprintf(str, "PIT: %d", current_pitch);
        memset(&pack_word, 0, sizeof(pack_word));
        make_word(&pack_word, txt_gim_pitch, 0, 1, 20, 2, 1140, 520, str, 7, 2, 2); 
        pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
        referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));
        vTaskDelay(5);
    }

    // --------------------------------------------------------
    // 【模块 3】：侧视图机甲 + 战机姿态仪同步
    // --------------------------------------------------------
    memset(&pack_figs, 0, sizeof(pack_figs));
    pack_figs.data_cmd_id = 0x0104; 
    pack_figs.sender_id = sender;
    pack_figs.receiver_id = receiver;

    float roll = CHASSIS.fdb.body.pitch; 
    int16_t body_dx = (int16_t)(BODY_LEN * cosf(roll));
    int16_t body_dy = (int16_t)(BODY_LEN * sinf(roll)); 
    uint8_t pitch_color = (fabsf(roll) > 0.35f) ? 3 : 8; 
    make_line(&pack_figs, 0, dyn_chassis, pitch_color, 8, 0, 
              BODY_X - body_dx, BODY_Y - body_dy, BODY_X + body_dx, BODY_Y + body_dy, 2, 2);

    for (int i = 0; i < 2; i++) {
        float L0 = CHASSIS.fdb.leg[i].rod.L0;
        float theta = CHASSIS.fdb.leg_state[i].theta * THETA_DIR; 
        float d = L0 * LEG_SCALE; 
        
        float cos_alpha = (THIGH_PX*THIGH_PX + d*d - CALF_PX*CALF_PX) / (2.0f * THIGH_PX * d);
        if(cos_alpha > 1.0f) cos_alpha = 1.0f;   
        if(cos_alpha < -1.0f) cos_alpha = -1.0f;
        float alpha = acosf(cos_alpha);
        
        float theta_thigh = theta + KNEE_DIR * alpha;
        int16_t knee_x = BODY_X + (int16_t)(THIGH_PX * sinf(theta_thigh));
        int16_t knee_y = BODY_Y - (int16_t)(THIGH_PX * cosf(theta_thigh));
        
        int16_t foot_x = BODY_X + (int16_t)(d * sinf(theta));
        int16_t foot_y = BODY_Y - (int16_t)(d * cosf(theta));

        uint8_t color = (i == 0) ? 6 : 1; 
        if (CHASSIS.fdb.leg[i].is_take_off) color = 4; 
        
        make_line(&pack_figs, 1+i*2, (i==0 ? dyn_thigh_L : dyn_thigh_R), color, 6, 0, 
                  BODY_X, BODY_Y, knee_x, knee_y, 2, 2);
        make_line(&pack_figs, 2+i*2, (i==0 ? dyn_calf_L : dyn_calf_R), color, 4, 0, 
                  knee_x, knee_y, foot_x, foot_y, 2, 2);
    }
    
    float cos_r = cosf(roll);
    float sin_r = sinf(roll);
    make_line(&pack_figs, 5, dyn_horiz_L, pitch_color, 3, 0, 
              960 - (int16_t)(200*cos_r), 540 + (int16_t)(200*sin_r), 
              960 - (int16_t)(80*cos_r),  540 + (int16_t)(80*sin_r), 2, 2);
              
    make_line(&pack_figs, 6, dyn_horiz_R, pitch_color, 3, 0, 
              960 + (int16_t)(80*cos_r),  540 - (int16_t)(80*sin_r), 
              960 + (int16_t)(200*cos_r), 540 - (int16_t)(200*sin_r), 2, 2);
              
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_figs, sizeof(pack_figs));
    vTaskDelay(5); 

   // --------------------------------------------------------
		// 【模块 4】：旋转的巨型底盘矩形（已修正）
		// --------------------------------------------------------
		memset(&pack_figs, 0, sizeof(pack_figs));
		pack_figs.data_cmd_id = 0x0104; 
		pack_figs.sender_id = sender;
		pack_figs.receiver_id = receiver;

		// 1. 计算偏航误差（保持原逻辑不变）
		float yaw_error = (GIMBAL_DIRECT_YAW_MID*57.3 - ctom_message.gimbal_yaw_6020*53.7) * YAW_DIR;
		while (yaw_error > 180.0f)  yaw_error -= 360.0f;
		while (yaw_error < -180.0f) yaw_error += 360.0f;
		float yaw_rad = yaw_error * (float)M_PI / 180.0f;

		// 2. 修正：底盘坐标系旋转（前向为y轴正方向，右向为x轴正方向）
		float vf_x = -sinf(yaw_rad);  // 前方向量X（修正符号）
		float vf_y = cosf(yaw_rad);   // 前方向量Y
		float vr_x = cosf(yaw_rad);   // 右方向量X
		float vr_y = sinf(yaw_rad);   // 右方向量Y（修正符号）

		int16_t L = 90;  // 底盘半长（前后方向）
		int16_t W = 64;  // 底盘半宽（左右方向）
		uint8_t chas_color = (fabsf(yaw_error) > 60.0f) ? 3 : 6; 

		// 3. 修正：矩形四个顶点坐标（基于底盘中心 COMPASS_X, COMPASS_Y）
		int16_t fl_x = COMPASS_X + (int16_t)( L * vf_x + W * vr_x ); // 前左
		int16_t fl_y = COMPASS_Y + (int16_t)( L * vf_y + W * vr_y );
		int16_t fr_x = COMPASS_X + (int16_t)( L * vf_x - W * vr_x ); // 前右
		int16_t fr_y = COMPASS_Y + (int16_t)( L * vf_y - W * vr_y );
		int16_t bl_x = COMPASS_X + (int16_t)(-L * vf_x + W * vr_x ); // 后左
		int16_t bl_y = COMPASS_Y + (int16_t)(-L * vf_y + W * vr_y );
		int16_t br_x = COMPASS_X + (int16_t)(-L * vf_x - W * vr_x ); // 后右
		int16_t br_y = COMPASS_Y + (int16_t)(-L * vf_y - W * vr_y );

		// 4. 绘制矩形四条边（统一线宽）
		make_line(&pack_figs, 0, dyn_chas_l1, chas_color, 2, 0, fl_x, fl_y, fr_x, fr_y, 2, 2); // 前
		make_line(&pack_figs, 1, dyn_chas_l2, chas_color, 2, 0, fr_x, fr_y, br_x, br_y, 2, 2); // 右
		make_line(&pack_figs, 2, dyn_chas_l3, chas_color, 2, 0, br_x, br_y, bl_x, bl_y, 2, 2); // 后
		make_line(&pack_figs, 3, dyn_chas_l4, chas_color, 2, 0, bl_x, bl_y, fl_x, fl_y, 2, 2); // 左

		// 5. 修正：方向指示线（指向正前方）
		int16_t front_mid_x = COMPASS_X + (int16_t)(L * vf_x);
		int16_t front_mid_y = COMPASS_Y + (int16_t)(L * vf_y);
		make_line(&pack_figs, 4, dyn_chas_dir, chas_color, 3, 0, COMPASS_X, COMPASS_Y, front_mid_x, front_mid_y, 2, 2);

		// 6. 隐藏多余图元
		pack_figs.interaction_figure_t[5].operate_tpye = 0; 
		pack_figs.interaction_figure_t[6].operate_tpye = 0; 

		referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_figs, sizeof(pack_figs));
		vTaskDelay(5); 

    // --------------------------------------------------------
    // 【模块 5】：指示灯与电容条填充
    // --------------------------------------------------------
    memset(&pack_figs, 0, sizeof(pack_figs));
    pack_figs.data_cmd_id = 0x0104; 
    pack_figs.sender_id = sender;
    pack_figs.receiver_id = receiver;

    uint8_t c_free = (CHASSIS.mode == CHASSIS_FREE) ? 2 : 8; 
//    uint8_t c_aim  = (ctom_message.auto_aim_active) ? 3 : 8; 
//    uint8_t c_fric = (ctom_message.fric_bool) ? 2 : 8;       
    
    make_line(&pack_figs, 0, lit_mode, c_free, 15, 0, 100, 820, 100, 835, 2, 2);
//    make_line(&pack_figs, 1, lit_aim,  c_aim,  15, 0, 100, 750, 100, 765, 2, 2);
//    make_line(&pack_figs, 2, lit_fric, c_fric, 15, 0, 100, 680, 100, 695, 2, 2);

//    if (CHASSIS.fdb.body.is_slipping) {
//        make_line(&pack_figs, 3, dyn_slip_box, 3, 20, 1, 800, 680, 1120, 740, 2, 2); 
//    } 
//		else {
//        // 【修复】：用原点坐标直接把图形揉进黑洞，绝对看不见！
//        make_line(&pack_figs, 3, dyn_slip_box, 3, 0,  1, 0, 0, 0, 0, 2, 2); 
//    }

    // 电容条填充：填充高度396像素。
    float cap_limit = (JudgementData.power_heat_data_t.buffer_energy > 0.0f) ? JudgementData.power_heat_data_t.buffer_energy : 0.0f;
    cap_limit = (cap_limit > 60.0f) ? 60.0f : cap_limit; 
    int16_t cap_h = (int16_t)(cap_limit / 60.0f * 396.0f); 
    uint8_t cap_color = (cap_limit > 40.0f) ? 2 : ((cap_limit > 15.0f) ? 1 : 3); 

    make_line(&pack_figs, 4, dyn_cap_fill, cap_color, 36, 0, 1760, 302, 1760, 302 + cap_h, 2, 2);
    
    // 罗盘底层指针
    int16_t compass_end_x = COMPASS_X + (int16_t)(COMPASS_R * sinf(yaw_rad));
    int16_t compass_end_y = COMPASS_Y + (int16_t)(COMPASS_R * cosf(yaw_rad)); 
    make_line(&pack_figs, 5, dyn_compass, chas_color, 6, 0, COMPASS_X, COMPASS_Y, compass_end_x, compass_end_y, 2, 2);
    make_cir (&pack_figs, 6, dyn_compass_tip, chas_color, 2, 2, 2, 10, compass_end_x, compass_end_y, 0, 360, 6, 0, 0);

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_figs, sizeof(pack_figs)); 
    
    // --------------------------------------------------------
    // 【模块 6】：电容动态百分比文字
    // --------------------------------------------------------
    char str_cap[10];
    uint8_t cap_per = (uint8_t)(cap_limit / 60.0f * 100.0f);
    sprintf(str_cap, "%d", cap_per);
    memset(&pack_word, 0, sizeof(pack_word));
    make_word(&pack_word, txt_cap_per, 0, cap_color, 36, 4, 1800, 500-20, str_cap, 7, 2, 3);
    pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));
}

// ==========================================
// 4. FreeRTOS 主任务调用入口
// ==========================================
void UI_Task(void const * argument)
{

    osDelay(3000); 
    judge_painting_init(); 
    osDelay(2000); 

    while(1)
    {
        if (keyboard_data.Remote_Key_V==1)
        {
            judge_painting_init(); 
            osDelay(5000); 
        }

//        if(CHASSIS.mode != CHASSIS_SAFE && CHASSIS.mode != CHASSIS_OFF) {
            judge_painting_update();
//        }
        
        osDelay(30); 
    }
}