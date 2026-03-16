//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_frame1_H
#define UI_frame1_H

#include "ui_interface.h"

extern ui_interface_rect_t *ui_frame1_Ungroup_Aim_range;
extern ui_interface_line_t *ui_frame1_Ungroup_small_leg;
extern ui_interface_line_t *ui_frame1_Ungroup_big_leg;
extern ui_interface_line_t *ui_frame1_Ungroup_Body_pitch;
extern ui_interface_rect_t *ui_frame1_Ungroup_chassis;
extern ui_interface_line_t *ui_frame1_Ungroup_gimbal;
extern ui_interface_number_t *ui_frame1_Ungroup_vx_float;
extern ui_interface_number_t *ui_frame1_Ungroup_gyro_float;
extern ui_interface_number_t *ui_frame1_Ungroup_pitch_float;
extern ui_interface_number_t *ui_frame1_Ungroup_yaw_float;
extern ui_interface_number_t *ui_frame1_Ungroup_leg_float;
extern ui_interface_line_t *ui_frame1_Ungroup_left_car;
extern ui_interface_line_t *ui_frame1_Ungroup_right_car;
extern ui_interface_line_t *ui_frame1_Ungroup_left_big_leg;
extern ui_interface_line_t *ui_frame1_Ungroup_left_small_leg;
extern ui_interface_ellipse_t *ui_frame1_Ungroup_NewEllipse;
extern ui_interface_string_t *ui_frame1_Ungroup_Aim_Range_word;
extern ui_interface_string_t *ui_frame1_Ungroup_MODE;
extern ui_interface_string_t *ui_frame1_Ungroup_loss_control;
extern ui_interface_string_t *ui_frame1_Ungroup_yaw_offest;
extern ui_interface_string_t *ui_frame1_Ungroup_Vel;
extern ui_interface_string_t *ui_frame1_Ungroup_GORY;
extern ui_interface_string_t *ui_frame1_Ungroup_PITCH;
extern ui_interface_string_t *ui_frame1_Ungroup_leg_mode;
extern ui_interface_string_t *ui_frame1_Ungroup_VISION;
extern ui_interface_string_t *ui_frame1_Ungroup_mode_text;
extern ui_interface_string_t *ui_frame1_Ungroup_vision_text;
extern ui_interface_string_t *ui_frame1_Ungroup_loss_control_text;
extern ui_interface_string_t *ui_frame1_Ungroup_left_leg;
extern ui_interface_string_t *ui_frame1_Ungroup_CAP;

void ui_init_frame1_Ungroup();
void ui_update_frame1_Ungroup();
void ui_remove_frame1_Ungroup();


#endif // UI_frame1_H
