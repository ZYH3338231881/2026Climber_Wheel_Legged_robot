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
#include "bsp_can.h"
extern Keyboard_Data keyboard_data;

extern RC_ctrl_t rc_ctrl; // 声明遥控器全局结构体

// ==========================================
// 外部核心数据引用
// ==========================================
extern JudgementDataTypedef JudgementData;
extern Chassis_s CHASSIS;               
extern CTOM_message_t ctom_message;     
extern SuperCap_rx_t SuperCap_rx;

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

    make_cir(&pack_figs, 2, bg_aim_out, 4, 1, 2, 1, 5, 930, 480, 0, 360, 30, 0, 0); //紫红色准星
    
    pack_figs.data_cmd_id = 0x0104; pack_figs.sender_id = sender; pack_figs.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_figs, sizeof(pack_figs));
    vTaskDelay(10); 

    /* --- 第 2 包：标尺与外框、罗盘参照线、腿部辅助线 --- */
    memset(&pack_figs, 0, sizeof(pack_figs));
//    make_line(&pack_figs, 0, bg_scale_1,    4, 2, 0, 945, 420, 975, 420, 1, 1);
//    make_line(&pack_figs, 1, bg_scale_2,    4, 2, 0, 935, 360, 985, 360, 1, 1);
//    make_line(&pack_figs, 2, bg_scale_3,    4, 2, 0, 920, 300, 1000,300, 1, 1);
    
    // make_line(&pack_figs, 3, bg_compass_gim,2, 4, 0, COMPASS_X, COMPASS_Y, COMPASS_X, COMPASS_Y + COMPASS_R + 10, 1, 1); 
    // make_line(&pack_figs, 4, bg_status_box, 1, 2, 1, 50, 620, 400, 880, 1, 1); 
    // make_line(&pack_figs, 5, bg_cap_box,    8, 2, 1, 1740,300, 1780,700, 1, 1);
//    make_line(&pack_figs, 6, bg_leg_l_line, 6, 2, 0, BODY_X - 120, BODY_Y - 160, BODY_X - 40,  BODY_Y - 160, 1, 1);
    
    pack_figs.data_cmd_id = 0x0104; pack_figs.sender_id = sender; pack_figs.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_figs, sizeof(pack_figs));
    vTaskDelay(10); 

    /* --- 第 3 包：【巨型紫色大框】与右腿辅助线 --- */
    memset(&pack_figs, 0, sizeof(pack_figs));
//    make_line(&pack_figs, 0, bg_leg_r_line, 1, 2, 0, BODY_X + 30,  BODY_Y - 160, BODY_X + 110, BODY_Y - 160, 1, 1);
    // make_line(&pack_figs, 1, bg_body_l,     6, 8, 0, 480, 0, 730, 300, 1, 1); 
    // make_line(&pack_figs, 2, bg_body_r,     6, 8, 0, 1440, 0, 1190, 300, 1, 1); 
    // make_line(&pack_figs, 3, bg_main_box,   2, 3, 1, 960-500, 480-150, 960+500, 480+300, 1, 1);
    
    pack_figs.interaction_figure_t[4].operate_tpye = 0;
    pack_figs.interaction_figure_t[5].operate_tpye = 0;
    pack_figs.interaction_figure_t[6].operate_tpye = 0;
    pack_figs.data_cmd_id = 0x0104; pack_figs.sender_id = sender; pack_figs.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_figs, sizeof(pack_figs));
    vTaskDelay(10); 
		
		
    /* --- 第 4 包：侧视动态腿部变化展示 水平车体roll变化展示*/
    // 底盘中心线
    make_line(&pack_figs, 0, dyn_chassis, 8, 8, 0, BODY_X - BODY_LEN, BODY_Y, BODY_X + BODY_LEN, BODY_Y, 1, 2); 
    // 左腿模型
    make_line(&pack_figs, 1, dyn_thigh_L, 6, 6, 0, BODY_X, BODY_Y, BODY_X, BODY_Y-100, 1, 2); 
    make_line(&pack_figs, 2, dyn_calf_L,  6, 4, 0, BODY_X, BODY_Y-100, BODY_X, BODY_Y-250, 1, 2); 
    // 右腿模型
    make_line(&pack_figs, 3, dyn_thigh_R, 1, 6, 0, BODY_X, BODY_Y, BODY_X, BODY_Y-100, 1, 2); 
    make_line(&pack_figs, 4, dyn_calf_R,  1, 4, 0, BODY_X, BODY_Y-100, BODY_X, BODY_Y-250, 1, 2); 
    // 水平辅助线
    make_line(&pack_figs, 5, dyn_horiz_L, 2, 3, 0, 960-200, 540, 960-80, 540, 1, 2); 
    make_line(&pack_figs, 6, dyn_horiz_R, 2, 3, 0, 960+80,  540, 960+200, 540, 1, 2); 
    
    pack_figs.data_cmd_id = 0x0104; pack_figs.sender_id = sender; pack_figs.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_figs, sizeof(pack_figs));
    vTaskDelay(10); 

    /* --- 第 5 包：旋转底盘占位 --- */
    memset(&pack_figs, 0, sizeof(pack_figs));
    // 初始化圆弧占位（六分之一个圆弧）
    // 绘制底盘朝向指示器（345到15度，对应-15到15度）
    make_cir(&pack_figs, 0, dyn_chas_l1, 0, 1, 4, 2, 5, SCREEN_X_CENTER, SCREEN_Y_CENTER, 345, 15, 200, 200, 200); 

    // 隐藏其他图元
    
    pack_figs.interaction_figure_t[1].operate_tpye = 0;
    pack_figs.interaction_figure_t[2].operate_tpye = 0;
    pack_figs.interaction_figure_t[3].operate_tpye = 0;
    pack_figs.interaction_figure_t[4].operate_tpye = 0;
    pack_figs.interaction_figure_t[5].operate_tpye = 0; 
    pack_figs.interaction_figure_t[6].operate_tpye = 0; 
    pack_figs.data_cmd_id = 0x0104; pack_figs.sender_id = sender; pack_figs.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_figs, sizeof(pack_figs));
    vTaskDelay(10); 

    /* --- 第 6 包：指示灯与剩余占位 --- */
    memset(&pack_figs, 0, sizeof(pack_figs));
    make_line(&pack_figs, 0, lit_mode,        8, 15, 0, 100, 820, 100, 835, 1, 2);
    make_line(&pack_figs, 1, lit_aim,         8, 15, 0, 100, 750, 100, 765, 1, 2);
    make_line(&pack_figs, 2, lit_fric,        8, 15, 0, 100, 680, 100, 695, 1, 2);
    
    // 【修复】：初始化时，把打滑提示框直接塞进原点 (0,0) 完美隐藏
    //  make_line(&pack_figs, 3, dyn_slip_box,    3, 0,  1, 0, 0, 0, 0, 1, 2);  
    
     make_line(&pack_figs, 4, dyn_cap_fill, 2, 36, 0, 1760, 302, 1760, 698, 1, 2); 
    //  make_line(&pack_figs, 5, dyn_compass,     3, 6, 0, COMPASS_X, COMPASS_Y, COMPASS_X, COMPASS_Y+COMPASS_R, 1, 2);
    //  make_cir (&pack_figs, 6, dyn_compass_tip, 3, 1, 2, 1, 10, COMPASS_X, COMPASS_Y+COMPASS_R, 0, 360, 6, 0, 0);
    
    pack_figs.data_cmd_id = 0x0104; pack_figs.sender_id = sender; pack_figs.receiver_id = receiver;
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_figs, sizeof(pack_figs));
    vTaskDelay(10); 

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

//    memset(&pack_word, 0, sizeof(pack_word));
//    make_word(&pack_word, txt_leg_L_leg, 0, 6, 16, 2, BODY_X - 120, BODY_Y - 180, "L(CYAN)", 7, 1, 1); 
//    pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
//    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));  vTaskDelay(80);

//    memset(&pack_word, 0, sizeof(pack_word));
//    make_word(&pack_word, txt_leg_R_leg, 0, 1, 16, 2, BODY_X + 30, BODY_Y - 180, "R(YELLOW)", 7, 1, 1); 
//    pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
//    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));  vTaskDelay(80);

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

    // 1 接收到自瞄  0未接收到自瞄 
    if (ctom_message.aim_live != last_ui_aim) {
        last_ui_aim = ctom_message.aim_live;
        send_cnt_aim = 5;
    }
    if (send_cnt_aim > 0) {
        uint8_t color = last_ui_aim ? 2 : 4; 
        char *str = last_ui_aim ? "AIM: ON " : "AIM: OFF";
        memset(&pack_word, 0, sizeof(pack_word));
        make_word(&pack_word, txt_aim, 0, color, 24, 2, 140, 750, str, 7, 2, 2);
        pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
        referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));
        vTaskDelay(10);
        send_cnt_aim--;
    }




            memset(&pack_figs, 0, sizeof(pack_figs));
            // 当aim_shoot=2时，准星变为绿色；否则为紫色
            uint8_t aim_color = (ctom_message.aim_shoot == 2) ? 2 : 4; // 2表示绿色，4表示紫色
            make_cir(&pack_figs, 0, bg_aim_out, aim_color, 2, 2, 1, 5, 930, 480, 0, 360, 30, 0, 0);

            pack_figs.data_cmd_id = 0x0104; 
            pack_figs.sender_id = sender; 
            pack_figs.receiver_id = receiver;
            referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_figs, sizeof(pack_figs));
            vTaskDelay(10);



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
//    int16_t current_yaw   = (int16_t)ctom_message.gimbal_imu_yaw; 
//    int16_t current_pitch = (int16_t)ctom_message.gimbal_imu_pitch; 
//    
//    if (current_yaw != last_ui_yaw) {
//        last_ui_yaw = current_yaw;
//        char str[15];
//        sprintf(str, "YAW: %d", current_yaw);
//        memset(&pack_word, 0, sizeof(pack_word));
//        make_word(&pack_word, txt_gim_yaw, 0, 1, 20, 2, 1140, 560, str, 7, 2, 2); 
//        pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
//        referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));
//        vTaskDelay(5);
//    }
//    
//    if (current_pitch != last_ui_pitch) {
//        last_ui_pitch = current_pitch;
//        char str[15];
//        sprintf(str, "PIT: %d", current_pitch);
//        memset(&pack_word, 0, sizeof(pack_word));
//        make_word(&pack_word, txt_gim_pitch, 0, 1, 20, 2, 1140, 520, str, 7, 2, 2); 
//        pack_word.data_cmd_id = 0x0110; pack_word.sender_id = sender; pack_word.receiver_id = receiver;
//        referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_word, sizeof(pack_word));
//        vTaskDelay(10);
//    }

    // --------------------------------------------------------
    // 【模块 3】：腿部运动姿态  动态展示
    // --------------------------------------------------------
    memset(&pack_figs, 0, sizeof(pack_figs));
    pack_figs.data_cmd_id = 0x0104; 
    pack_figs.sender_id = sender;
    pack_figs.receiver_id = receiver;

    float roll = CHASSIS.fdb.body.pitch; 
		float pitch= CHASSIS.fdb.body.roll;
		
    int16_t body_dx = (int16_t)(BODY_LEN * cosf(roll));
    int16_t body_dy = (int16_t)(BODY_LEN * sinf(roll)); 
		
		int16_t chassis_dx = (int16_t)(BODY_LEN * cosf(pitch));
    int16_t chassis_dy = (int16_t)(BODY_LEN * sinf(pitch)); 
		
    uint8_t pitch_color = (fabsf(pitch) > 0.15f) ? 3 : 8; 
	  uint8_t roll_color = (fabsf(roll) > 0.15f) ? 3 : 8; 

    make_line(&pack_figs, 0, dyn_chassis, pitch_color, 8, 0, BODY_X - chassis_dx, BODY_Y - chassis_dy, BODY_X + chassis_dx, BODY_Y + chassis_dy, 2, 2);

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
    make_line(&pack_figs, 5, dyn_horiz_L, roll_color, 3, 0, 
              960 - (int16_t)(200*cos_r), 540 + (int16_t)(200*sin_r), 
              960 - (int16_t)(80*cos_r),  540 + (int16_t)(80*sin_r), 2, 2);
              
    make_line(&pack_figs, 6, dyn_horiz_R, roll_color, 3, 0, 
              960 + (int16_t)(80*cos_r),  540 - (int16_t)(80*sin_r), 
              960 + (int16_t)(200*cos_r), 540 - (int16_t)(200*sin_r), 2, 2);
              
    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_figs, sizeof(pack_figs));
    vTaskDelay(10); 

    // --------------------------------------------------------
    // 【模块 4】：小陀螺圆弧显示
	// --------------------------------------------------------
		memset(&pack_figs, 0, sizeof(pack_figs));
		pack_figs.data_cmd_id = 0x0104; 
		pack_figs.sender_id = sender;
		pack_figs.receiver_id = receiver;

		// 1. 计算偏航误差（保持原逻辑不变）
		float yaw_error = (GIMBAL_DIRECT_YAW_MID*57.3 - ctom_message.gimbal_yaw_6020*57.3) * YAW_DIR;
		while (yaw_error > 180.0f)  yaw_error -= 360.0f;
		while (yaw_error < -180.0f) yaw_error += 360.0f;
		float yaw_rad = yaw_error * (float)M_PI / 180.0f;

		// 2. 小陀螺显示：六分之一个圆弧模拟底盘中值
	uint8_t chas_color = (fabsf(yaw_error) > 60.0f) ? 3 : 6; 
	float arc_radius = 200; // 圆弧半径
	
	// 将弧度转换为角度，并确保在0-360度范围内
	float yaw_deg = yaw_error; // yaw_error已经是-180到180度的角度
	if (yaw_deg < 0) yaw_deg += 360.0f; // 转换为0-360度
	
	// 计算圆弧起始和结束角度（60度范围）
	float arc_start_deg = yaw_deg - 30.0f;
	float arc_end_deg = yaw_deg + 30.0f;
	
	// 确保角度在0-360度范围内
	if (arc_start_deg < 0) arc_start_deg += 360.0f;
	if (arc_end_deg > 360.0f) arc_end_deg -= 360.0f;
	
	// 绘制六分之一个圆弧（60度）
	// 使用更新操作刷新已创建的图元
    // 确保所有参数与创建操作完全一致（除了角度和颜色）
    make_cir(&pack_figs, 0, dyn_chas_l1, chas_color, 2, 4, 2, 5, SCREEN_X_CENTER, SCREEN_Y_CENTER, arc_start_deg, arc_end_deg, arc_radius, 200, 200);

		// 3. 隐藏多余图元
		pack_figs.interaction_figure_t[1].operate_tpye = 0; 
		pack_figs.interaction_figure_t[2].operate_tpye = 0; 
		pack_figs.interaction_figure_t[3].operate_tpye = 0; 
		pack_figs.interaction_figure_t[4].operate_tpye = 0; 
		pack_figs.interaction_figure_t[5].operate_tpye = 0; 
		pack_figs.interaction_figure_t[6].operate_tpye = 0; 

		referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_figs, sizeof(pack_figs));
		vTaskDelay(10); 

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

    int16_t cap_h = (int16_t)(SuperCap_rx.capEnergy  / 255.0f * 396.0f); 
    uint8_t cap_color = (SuperCap_rx.capEnergy  > 150.0f) ? 1 : ((SuperCap_rx.capEnergy > 50.0f) ? 1 : 3); 

    make_line(&pack_figs, 4, dyn_cap_fill, cap_color, 36, 0, 1760, 302, 1760, 302 + cap_h, 2, 2);
//    
    // 罗盘底层指针
    // int16_t compass_end_x = COMPASS_X + (int16_t)(COMPASS_R * sinf(yaw_rad));
    // int16_t compass_end_y = COMPASS_Y + (int16_t)(COMPASS_R * cosf(yaw_rad)); 
    // make_line(&pack_figs, 5, dyn_compass, chas_color, 6, 0, COMPASS_X, COMPASS_Y, compass_end_x, compass_end_y, 2, 2);
    // make_cir (&pack_figs, 6, dyn_compass_tip, chas_color, 2, 2, 2, 10, compass_end_x, compass_end_y, 0, 360, 6, 0, 0);

    referee_data_pack_handle(0xA5, 0x0301, (uint8_t *)&pack_figs, sizeof(pack_figs)); 

    // --------------------------------------------------------
    // 【模块 6】：电容动态百分比文字
    // --------------------------------------------------------
    char str_cap[10];
    uint8_t cap_per = (uint8_t)(SuperCap_rx.capEnergy / 255.0f * 100.0f);
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

    osDelay(2000); 
    judge_painting_init(); 
    osDelay(1000); 

    while(1)
    {
        if (keyboard_data.Remote_Key_V==1)
        {
            judge_painting_init(); 
            osDelay(1000); 
        }

//        if(CHASSIS.mode != CHASSIS_SAFE && CHASSIS.mode != CHASSIS_OFF) {
            judge_painting_update();
//        }
        
        osDelay(10); 
    }
}