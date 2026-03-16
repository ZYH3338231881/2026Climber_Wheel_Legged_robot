//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_7_frame_t ui_frame1_Ungroup_0;

ui_interface_rect_t *ui_frame1_Ungroup_Aim_range = (ui_interface_rect_t*)&(ui_frame1_Ungroup_0.data[0]);
ui_interface_line_t *ui_frame1_Ungroup_small_leg = (ui_interface_line_t*)&(ui_frame1_Ungroup_0.data[1]);
ui_interface_line_t *ui_frame1_Ungroup_big_leg = (ui_interface_line_t*)&(ui_frame1_Ungroup_0.data[2]);
ui_interface_line_t *ui_frame1_Ungroup_Body_pitch = (ui_interface_line_t*)&(ui_frame1_Ungroup_0.data[3]);
ui_interface_rect_t *ui_frame1_Ungroup_chassis = (ui_interface_rect_t*)&(ui_frame1_Ungroup_0.data[4]);
ui_interface_line_t *ui_frame1_Ungroup_gimbal = (ui_interface_line_t*)&(ui_frame1_Ungroup_0.data[5]);
ui_interface_number_t *ui_frame1_Ungroup_vx_float = (ui_interface_number_t*)&(ui_frame1_Ungroup_0.data[6]);

void _ui_init_frame1_Ungroup_0() {
    for (int i = 0; i < 7; i++) {
        ui_frame1_Ungroup_0.data[i].figure_name[0] = 1;
        ui_frame1_Ungroup_0.data[i].figure_name[1] = 0;
        ui_frame1_Ungroup_0.data[i].figure_name[2] = i + 0;
        ui_frame1_Ungroup_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_frame1_Ungroup_0.data[i].operate_type = 0;
    }

    ui_frame1_Ungroup_Aim_range->figure_type = 1;
    ui_frame1_Ungroup_Aim_range->operate_type = 1;
    ui_frame1_Ungroup_Aim_range->layer = 0;
    ui_frame1_Ungroup_Aim_range->color = 4;
    ui_frame1_Ungroup_Aim_range->start_x = 550;
    ui_frame1_Ungroup_Aim_range->start_y = 268;
    ui_frame1_Ungroup_Aim_range->width = 2;
    ui_frame1_Ungroup_Aim_range->end_x = 1356;
    ui_frame1_Ungroup_Aim_range->end_y = 881;

    ui_frame1_Ungroup_small_leg->figure_type = 0;
    ui_frame1_Ungroup_small_leg->operate_type = 1;
    ui_frame1_Ungroup_small_leg->layer = 0;
    ui_frame1_Ungroup_small_leg->color = 5;
    ui_frame1_Ungroup_small_leg->start_x = 1592;
    ui_frame1_Ungroup_small_leg->start_y = 345;
    ui_frame1_Ungroup_small_leg->width = 4;
    ui_frame1_Ungroup_small_leg->end_x = 1649;
    ui_frame1_Ungroup_small_leg->end_y = 427;

    ui_frame1_Ungroup_big_leg->figure_type = 0;
    ui_frame1_Ungroup_big_leg->operate_type = 1;
    ui_frame1_Ungroup_big_leg->layer = 0;
    ui_frame1_Ungroup_big_leg->color = 5;
    ui_frame1_Ungroup_big_leg->start_x = 1648;
    ui_frame1_Ungroup_big_leg->start_y = 425;
    ui_frame1_Ungroup_big_leg->width = 4;
    ui_frame1_Ungroup_big_leg->end_x = 1598;
    ui_frame1_Ungroup_big_leg->end_y = 493;

    ui_frame1_Ungroup_Body_pitch->figure_type = 0;
    ui_frame1_Ungroup_Body_pitch->operate_type = 1;
    ui_frame1_Ungroup_Body_pitch->layer = 0;
    ui_frame1_Ungroup_Body_pitch->color = 8;
    ui_frame1_Ungroup_Body_pitch->start_x = 1495;
    ui_frame1_Ungroup_Body_pitch->start_y = 495;
    ui_frame1_Ungroup_Body_pitch->width = 5;
    ui_frame1_Ungroup_Body_pitch->end_x = 1728;
    ui_frame1_Ungroup_Body_pitch->end_y = 497;

    ui_frame1_Ungroup_chassis->figure_type = 1;
    ui_frame1_Ungroup_chassis->operate_type = 1;
    ui_frame1_Ungroup_chassis->layer = 0;
    ui_frame1_Ungroup_chassis->color = 7;
    ui_frame1_Ungroup_chassis->start_x = 904;
    ui_frame1_Ungroup_chassis->start_y = 41;
    ui_frame1_Ungroup_chassis->width = 3;
    ui_frame1_Ungroup_chassis->end_x = 1047;
    ui_frame1_Ungroup_chassis->end_y = 229;

    ui_frame1_Ungroup_gimbal->figure_type = 0;
    ui_frame1_Ungroup_gimbal->operate_type = 1;
    ui_frame1_Ungroup_gimbal->layer = 0;
    ui_frame1_Ungroup_gimbal->color = 0;
    ui_frame1_Ungroup_gimbal->start_x = 974;
    ui_frame1_Ungroup_gimbal->start_y = 137;
    ui_frame1_Ungroup_gimbal->width = 8;
    ui_frame1_Ungroup_gimbal->end_x = 972;
    ui_frame1_Ungroup_gimbal->end_y = 225;

    ui_frame1_Ungroup_vx_float->figure_type = 5;
    ui_frame1_Ungroup_vx_float->operate_type = 1;
    ui_frame1_Ungroup_vx_float->layer = 0;
    ui_frame1_Ungroup_vx_float->color = 3;
    ui_frame1_Ungroup_vx_float->start_x = 111;
    ui_frame1_Ungroup_vx_float->start_y = 630;
    ui_frame1_Ungroup_vx_float->width = 2;
    ui_frame1_Ungroup_vx_float->font_size = 15;
    ui_frame1_Ungroup_vx_float->number = 12345;


    ui_proc_7_frame(&ui_frame1_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_0, sizeof(ui_frame1_Ungroup_0));
}

void _ui_update_frame1_Ungroup_0() {
    for (int i = 0; i < 7; i++) {
        ui_frame1_Ungroup_0.data[i].operate_type = 2;
    }
    
    ui_frame1_Ungroup_vx_float->number = 12345;

    ui_proc_7_frame(&ui_frame1_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_0, sizeof(ui_frame1_Ungroup_0));
}

void _ui_remove_frame1_Ungroup_0() {
    for (int i = 0; i < 7; i++) {
        ui_frame1_Ungroup_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_frame1_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_0, sizeof(ui_frame1_Ungroup_0));
}
ui_7_frame_t ui_frame1_Ungroup_1;

ui_interface_number_t *ui_frame1_Ungroup_gyro_float = (ui_interface_number_t*)&(ui_frame1_Ungroup_1.data[0]);
ui_interface_number_t *ui_frame1_Ungroup_pitch_float = (ui_interface_number_t*)&(ui_frame1_Ungroup_1.data[1]);
ui_interface_number_t *ui_frame1_Ungroup_yaw_float = (ui_interface_number_t*)&(ui_frame1_Ungroup_1.data[2]);
ui_interface_number_t *ui_frame1_Ungroup_leg_float = (ui_interface_number_t*)&(ui_frame1_Ungroup_1.data[3]);
ui_interface_line_t *ui_frame1_Ungroup_left_car = (ui_interface_line_t*)&(ui_frame1_Ungroup_1.data[4]);
ui_interface_line_t *ui_frame1_Ungroup_right_car = (ui_interface_line_t*)&(ui_frame1_Ungroup_1.data[5]);
ui_interface_line_t *ui_frame1_Ungroup_left_big_leg = (ui_interface_line_t*)&(ui_frame1_Ungroup_1.data[6]);

void _ui_init_frame1_Ungroup_1() {
    for (int i = 0; i < 7; i++) {
        ui_frame1_Ungroup_1.data[i].figure_name[0] = 1;
        ui_frame1_Ungroup_1.data[i].figure_name[1] = 0;
        ui_frame1_Ungroup_1.data[i].figure_name[2] = i + 7;
        ui_frame1_Ungroup_1.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_frame1_Ungroup_1.data[i].operate_type = 0;
    }

    ui_frame1_Ungroup_gyro_float->figure_type = 5;
    ui_frame1_Ungroup_gyro_float->operate_type = 1;
    ui_frame1_Ungroup_gyro_float->layer = 0;
    ui_frame1_Ungroup_gyro_float->color = 3;
    ui_frame1_Ungroup_gyro_float->start_x = 110;
    ui_frame1_Ungroup_gyro_float->start_y = 578;
    ui_frame1_Ungroup_gyro_float->width = 2;
    ui_frame1_Ungroup_gyro_float->font_size = 15;
    ui_frame1_Ungroup_gyro_float->number = 12345;

    ui_frame1_Ungroup_pitch_float->figure_type = 5;
    ui_frame1_Ungroup_pitch_float->operate_type = 1;
    ui_frame1_Ungroup_pitch_float->layer = 0;
    ui_frame1_Ungroup_pitch_float->color = 3;
    ui_frame1_Ungroup_pitch_float->start_x = 1704;
    ui_frame1_Ungroup_pitch_float->start_y = 776;
    ui_frame1_Ungroup_pitch_float->width = 2;
    ui_frame1_Ungroup_pitch_float->font_size = 15;
    ui_frame1_Ungroup_pitch_float->number = 12345;

    ui_frame1_Ungroup_yaw_float->figure_type = 5;
    ui_frame1_Ungroup_yaw_float->operate_type = 1;
    ui_frame1_Ungroup_yaw_float->layer = 0;
    ui_frame1_Ungroup_yaw_float->color = 3;
    ui_frame1_Ungroup_yaw_float->start_x = 1702;
    ui_frame1_Ungroup_yaw_float->start_y = 720;
    ui_frame1_Ungroup_yaw_float->width = 2;
    ui_frame1_Ungroup_yaw_float->font_size = 15;
    ui_frame1_Ungroup_yaw_float->number = 12345;

    ui_frame1_Ungroup_leg_float->figure_type = 5;
    ui_frame1_Ungroup_leg_float->operate_type = 1;
    ui_frame1_Ungroup_leg_float->layer = 0;
    ui_frame1_Ungroup_leg_float->color = 3;
    ui_frame1_Ungroup_leg_float->start_x = 1641;
    ui_frame1_Ungroup_leg_float->start_y = 663;
    ui_frame1_Ungroup_leg_float->width = 2;
    ui_frame1_Ungroup_leg_float->font_size = 15;
    ui_frame1_Ungroup_leg_float->number = 12345;

    ui_frame1_Ungroup_left_car->figure_type = 0;
    ui_frame1_Ungroup_left_car->operate_type = 1;
    ui_frame1_Ungroup_left_car->layer = 0;
    ui_frame1_Ungroup_left_car->color = 1;
    ui_frame1_Ungroup_left_car->start_x = 523;
    ui_frame1_Ungroup_left_car->start_y = 16;
    ui_frame1_Ungroup_left_car->width = 3;
    ui_frame1_Ungroup_left_car->end_x = 759;
    ui_frame1_Ungroup_left_car->end_y = 397;

    ui_frame1_Ungroup_right_car->figure_type = 0;
    ui_frame1_Ungroup_right_car->operate_type = 1;
    ui_frame1_Ungroup_right_car->layer = 0;
    ui_frame1_Ungroup_right_car->color = 1;
    ui_frame1_Ungroup_right_car->start_x = 1433;
    ui_frame1_Ungroup_right_car->start_y = 12;
    ui_frame1_Ungroup_right_car->width = 3;
    ui_frame1_Ungroup_right_car->end_x = 1172;
    ui_frame1_Ungroup_right_car->end_y = 388;

    ui_frame1_Ungroup_left_big_leg->figure_type = 0;
    ui_frame1_Ungroup_left_big_leg->operate_type = 1;
    ui_frame1_Ungroup_left_big_leg->layer = 0;
    ui_frame1_Ungroup_left_big_leg->color = 2;
    ui_frame1_Ungroup_left_big_leg->start_x = 1648;
    ui_frame1_Ungroup_left_big_leg->start_y = 425;
    ui_frame1_Ungroup_left_big_leg->width = 4;
    ui_frame1_Ungroup_left_big_leg->end_x = 1598;
    ui_frame1_Ungroup_left_big_leg->end_y = 493;


    ui_proc_7_frame(&ui_frame1_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_1, sizeof(ui_frame1_Ungroup_1));
}

void _ui_update_frame1_Ungroup_1() {
    for (int i = 0; i < 7; i++) {
        ui_frame1_Ungroup_1.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_frame1_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_1, sizeof(ui_frame1_Ungroup_1));
}

void _ui_remove_frame1_Ungroup_1() {
    for (int i = 0; i < 7; i++) {
        ui_frame1_Ungroup_1.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_frame1_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_1, sizeof(ui_frame1_Ungroup_1));
}
ui_2_frame_t ui_frame1_Ungroup_2;

ui_interface_line_t *ui_frame1_Ungroup_left_small_leg = (ui_interface_line_t*)&(ui_frame1_Ungroup_2.data[0]);
ui_interface_ellipse_t *ui_frame1_Ungroup_NewEllipse = (ui_interface_ellipse_t*)&(ui_frame1_Ungroup_2.data[1]);

void _ui_init_frame1_Ungroup_2() {
    for (int i = 0; i < 2; i++) {
        ui_frame1_Ungroup_2.data[i].figure_name[0] = 1;
        ui_frame1_Ungroup_2.data[i].figure_name[1] = 0;
        ui_frame1_Ungroup_2.data[i].figure_name[2] = i + 14;
        ui_frame1_Ungroup_2.data[i].operate_type = 1;
    }
    for (int i = 2; i < 2; i++) {
        ui_frame1_Ungroup_2.data[i].operate_type = 0;
    }

    ui_frame1_Ungroup_left_small_leg->figure_type = 0;
    ui_frame1_Ungroup_left_small_leg->operate_type = 1;
    ui_frame1_Ungroup_left_small_leg->layer = 0;
    ui_frame1_Ungroup_left_small_leg->color = 2;
    ui_frame1_Ungroup_left_small_leg->start_x = 1592;
    ui_frame1_Ungroup_left_small_leg->start_y = 345;
    ui_frame1_Ungroup_left_small_leg->width = 4;
    ui_frame1_Ungroup_left_small_leg->end_x = 1649;
    ui_frame1_Ungroup_left_small_leg->end_y = 427;

    ui_frame1_Ungroup_NewEllipse->figure_type = 3;
    ui_frame1_Ungroup_NewEllipse->operate_type = 1;
    ui_frame1_Ungroup_NewEllipse->layer = 0;
    ui_frame1_Ungroup_NewEllipse->color = 3;
    ui_frame1_Ungroup_NewEllipse->start_x = 958;
    ui_frame1_Ungroup_NewEllipse->start_y = 458;
    ui_frame1_Ungroup_NewEllipse->width = 5;
    ui_frame1_Ungroup_NewEllipse->rx = 57;
    ui_frame1_Ungroup_NewEllipse->ry = 57;


    ui_proc_2_frame(&ui_frame1_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_2, sizeof(ui_frame1_Ungroup_2));
}

void _ui_update_frame1_Ungroup_2() {
    for (int i = 0; i < 2; i++) {
        ui_frame1_Ungroup_2.data[i].operate_type = 2;
    }

    ui_proc_2_frame(&ui_frame1_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_2, sizeof(ui_frame1_Ungroup_2));
}

void _ui_remove_frame1_Ungroup_2() {
    for (int i = 0; i < 2; i++) {
        ui_frame1_Ungroup_2.data[i].operate_type = 3;
    }

    ui_proc_2_frame(&ui_frame1_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_2, sizeof(ui_frame1_Ungroup_2));
}

ui_string_frame_t ui_frame1_Ungroup_3;
ui_interface_string_t* ui_frame1_Ungroup_Aim_Range_word = &(ui_frame1_Ungroup_3.option);

void _ui_init_frame1_Ungroup_3() {
    ui_frame1_Ungroup_3.option.figure_name[0] = 1;
    ui_frame1_Ungroup_3.option.figure_name[1] = 0;
    ui_frame1_Ungroup_3.option.figure_name[2] = 16;
    ui_frame1_Ungroup_3.option.operate_type = 1;

    ui_frame1_Ungroup_Aim_Range_word->figure_type = 7;
    ui_frame1_Ungroup_Aim_Range_word->operate_type = 1;
    ui_frame1_Ungroup_Aim_Range_word->layer = 0;
    ui_frame1_Ungroup_Aim_Range_word->color = 0;
    ui_frame1_Ungroup_Aim_Range_word->start_x = 558;
    ui_frame1_Ungroup_Aim_Range_word->start_y = 871;
    ui_frame1_Ungroup_Aim_Range_word->width = 2;
    ui_frame1_Ungroup_Aim_Range_word->font_size = 20;
    ui_frame1_Ungroup_Aim_Range_word->str_length = 9;
    strcpy(ui_frame1_Ungroup_Aim_Range_word->string, "Aim Range");


    ui_proc_string_frame(&ui_frame1_Ungroup_3);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_3, sizeof(ui_frame1_Ungroup_3));
}

void _ui_update_frame1_Ungroup_3() {
    ui_frame1_Ungroup_3.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_Ungroup_3);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_3, sizeof(ui_frame1_Ungroup_3));
}

void _ui_remove_frame1_Ungroup_3() {
    ui_frame1_Ungroup_3.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_Ungroup_3);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_3, sizeof(ui_frame1_Ungroup_3));
}
ui_string_frame_t ui_frame1_Ungroup_4;
ui_interface_string_t* ui_frame1_Ungroup_MODE = &(ui_frame1_Ungroup_4.option);

void _ui_init_frame1_Ungroup_4() {
    ui_frame1_Ungroup_4.option.figure_name[0] = 1;
    ui_frame1_Ungroup_4.option.figure_name[1] = 0;
    ui_frame1_Ungroup_4.option.figure_name[2] = 17;
    ui_frame1_Ungroup_4.option.operate_type = 1;

    ui_frame1_Ungroup_MODE->figure_type = 7;
    ui_frame1_Ungroup_MODE->operate_type = 1;
    ui_frame1_Ungroup_MODE->layer = 0;
    ui_frame1_Ungroup_MODE->color = 2;
    ui_frame1_Ungroup_MODE->start_x = 35;
    ui_frame1_Ungroup_MODE->start_y = 737;
    ui_frame1_Ungroup_MODE->width = 1;
    ui_frame1_Ungroup_MODE->font_size = 12;
    ui_frame1_Ungroup_MODE->str_length = 4;
    strcpy(ui_frame1_Ungroup_MODE->string, "MODE");


    ui_proc_string_frame(&ui_frame1_Ungroup_4);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_4, sizeof(ui_frame1_Ungroup_4));
}

void _ui_update_frame1_Ungroup_4() {
    ui_frame1_Ungroup_4.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_Ungroup_4);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_4, sizeof(ui_frame1_Ungroup_4));
}

void _ui_remove_frame1_Ungroup_4() {
    ui_frame1_Ungroup_4.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_Ungroup_4);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_4, sizeof(ui_frame1_Ungroup_4));
}
ui_string_frame_t ui_frame1_Ungroup_5;
ui_interface_string_t* ui_frame1_Ungroup_loss_control = &(ui_frame1_Ungroup_5.option);

void _ui_init_frame1_Ungroup_5() {
    ui_frame1_Ungroup_5.option.figure_name[0] = 1;
    ui_frame1_Ungroup_5.option.figure_name[1] = 0;
    ui_frame1_Ungroup_5.option.figure_name[2] = 18;
    ui_frame1_Ungroup_5.option.operate_type = 1;

    ui_frame1_Ungroup_loss_control->figure_type = 7;
    ui_frame1_Ungroup_loss_control->operate_type = 1;
    ui_frame1_Ungroup_loss_control->layer = 0;
    ui_frame1_Ungroup_loss_control->color = 0;
    ui_frame1_Ungroup_loss_control->start_x = 26;
    ui_frame1_Ungroup_loss_control->start_y = 681;
    ui_frame1_Ungroup_loss_control->width = 2;
    ui_frame1_Ungroup_loss_control->font_size = 15;
    ui_frame1_Ungroup_loss_control->str_length = 12;
    strcpy(ui_frame1_Ungroup_loss_control->string, "loss_control");


    ui_proc_string_frame(&ui_frame1_Ungroup_5);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_5, sizeof(ui_frame1_Ungroup_5));
}

void _ui_update_frame1_Ungroup_5() {
    ui_frame1_Ungroup_5.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_Ungroup_5);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_5, sizeof(ui_frame1_Ungroup_5));
}

void _ui_remove_frame1_Ungroup_5() {
    ui_frame1_Ungroup_5.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_Ungroup_5);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_5, sizeof(ui_frame1_Ungroup_5));
}
ui_string_frame_t ui_frame1_Ungroup_6;
ui_interface_string_t* ui_frame1_Ungroup_yaw_offest = &(ui_frame1_Ungroup_6.option);

void _ui_init_frame1_Ungroup_6() {
    ui_frame1_Ungroup_6.option.figure_name[0] = 1;
    ui_frame1_Ungroup_6.option.figure_name[1] = 0;
    ui_frame1_Ungroup_6.option.figure_name[2] = 19;
    ui_frame1_Ungroup_6.option.operate_type = 1;

    ui_frame1_Ungroup_yaw_offest->figure_type = 7;
    ui_frame1_Ungroup_yaw_offest->operate_type = 1;
    ui_frame1_Ungroup_yaw_offest->layer = 0;
    ui_frame1_Ungroup_yaw_offest->color = 8;
    ui_frame1_Ungroup_yaw_offest->start_x = 1540;
    ui_frame1_Ungroup_yaw_offest->start_y = 723;
    ui_frame1_Ungroup_yaw_offest->width = 2;
    ui_frame1_Ungroup_yaw_offest->font_size = 20;
    ui_frame1_Ungroup_yaw_offest->str_length = 3;
    strcpy(ui_frame1_Ungroup_yaw_offest->string, "yaw");


    ui_proc_string_frame(&ui_frame1_Ungroup_6);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_6, sizeof(ui_frame1_Ungroup_6));
}

void _ui_update_frame1_Ungroup_6() {
    ui_frame1_Ungroup_6.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_Ungroup_6);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_6, sizeof(ui_frame1_Ungroup_6));
}

void _ui_remove_frame1_Ungroup_6() {
    ui_frame1_Ungroup_6.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_Ungroup_6);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_6, sizeof(ui_frame1_Ungroup_6));
}
ui_string_frame_t ui_frame1_Ungroup_7;
ui_interface_string_t* ui_frame1_Ungroup_Vel = &(ui_frame1_Ungroup_7.option);

void _ui_init_frame1_Ungroup_7() {
    ui_frame1_Ungroup_7.option.figure_name[0] = 1;
    ui_frame1_Ungroup_7.option.figure_name[1] = 0;
    ui_frame1_Ungroup_7.option.figure_name[2] = 20;
    ui_frame1_Ungroup_7.option.operate_type = 1;

    ui_frame1_Ungroup_Vel->figure_type = 7;
    ui_frame1_Ungroup_Vel->operate_type = 1;
    ui_frame1_Ungroup_Vel->layer = 0;
    ui_frame1_Ungroup_Vel->color = 2;
    ui_frame1_Ungroup_Vel->start_x = 36;
    ui_frame1_Ungroup_Vel->start_y = 631;
    ui_frame1_Ungroup_Vel->width = 2;
    ui_frame1_Ungroup_Vel->font_size = 15;
    ui_frame1_Ungroup_Vel->str_length = 3;
    strcpy(ui_frame1_Ungroup_Vel->string, "Vel");


    ui_proc_string_frame(&ui_frame1_Ungroup_7);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_7, sizeof(ui_frame1_Ungroup_7));
}

void _ui_update_frame1_Ungroup_7() {
    ui_frame1_Ungroup_7.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_Ungroup_7);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_7, sizeof(ui_frame1_Ungroup_7));
}

void _ui_remove_frame1_Ungroup_7() {
    ui_frame1_Ungroup_7.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_Ungroup_7);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_7, sizeof(ui_frame1_Ungroup_7));
}
ui_string_frame_t ui_frame1_Ungroup_8;
ui_interface_string_t* ui_frame1_Ungroup_GORY = &(ui_frame1_Ungroup_8.option);

void _ui_init_frame1_Ungroup_8() {
    ui_frame1_Ungroup_8.option.figure_name[0] = 1;
    ui_frame1_Ungroup_8.option.figure_name[1] = 0;
    ui_frame1_Ungroup_8.option.figure_name[2] = 21;
    ui_frame1_Ungroup_8.option.operate_type = 1;

    ui_frame1_Ungroup_GORY->figure_type = 7;
    ui_frame1_Ungroup_GORY->operate_type = 1;
    ui_frame1_Ungroup_GORY->layer = 0;
    ui_frame1_Ungroup_GORY->color = 2;
    ui_frame1_Ungroup_GORY->start_x = 31;
    ui_frame1_Ungroup_GORY->start_y = 577;
    ui_frame1_Ungroup_GORY->width = 2;
    ui_frame1_Ungroup_GORY->font_size = 15;
    ui_frame1_Ungroup_GORY->str_length = 4;
    strcpy(ui_frame1_Ungroup_GORY->string, "GORY");


    ui_proc_string_frame(&ui_frame1_Ungroup_8);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_8, sizeof(ui_frame1_Ungroup_8));
}

void _ui_update_frame1_Ungroup_8() {
    ui_frame1_Ungroup_8.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_Ungroup_8);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_8, sizeof(ui_frame1_Ungroup_8));
}

void _ui_remove_frame1_Ungroup_8() {
    ui_frame1_Ungroup_8.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_Ungroup_8);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_8, sizeof(ui_frame1_Ungroup_8));
}
ui_string_frame_t ui_frame1_Ungroup_9;
ui_interface_string_t* ui_frame1_Ungroup_PITCH = &(ui_frame1_Ungroup_9.option);

void _ui_init_frame1_Ungroup_9() {
    ui_frame1_Ungroup_9.option.figure_name[0] = 1;
    ui_frame1_Ungroup_9.option.figure_name[1] = 0;
    ui_frame1_Ungroup_9.option.figure_name[2] = 22;
    ui_frame1_Ungroup_9.option.operate_type = 1;

    ui_frame1_Ungroup_PITCH->figure_type = 7;
    ui_frame1_Ungroup_PITCH->operate_type = 1;
    ui_frame1_Ungroup_PITCH->layer = 0;
    ui_frame1_Ungroup_PITCH->color = 8;
    ui_frame1_Ungroup_PITCH->start_x = 1540;
    ui_frame1_Ungroup_PITCH->start_y = 781;
    ui_frame1_Ungroup_PITCH->width = 2;
    ui_frame1_Ungroup_PITCH->font_size = 20;
    ui_frame1_Ungroup_PITCH->str_length = 5;
    strcpy(ui_frame1_Ungroup_PITCH->string, "PITCH");


    ui_proc_string_frame(&ui_frame1_Ungroup_9);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_9, sizeof(ui_frame1_Ungroup_9));
}

void _ui_update_frame1_Ungroup_9() {
    ui_frame1_Ungroup_9.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_Ungroup_9);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_9, sizeof(ui_frame1_Ungroup_9));
}

void _ui_remove_frame1_Ungroup_9() {
    ui_frame1_Ungroup_9.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_Ungroup_9);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_9, sizeof(ui_frame1_Ungroup_9));
}
ui_string_frame_t ui_frame1_Ungroup_10;
ui_interface_string_t* ui_frame1_Ungroup_leg_mode = &(ui_frame1_Ungroup_10.option);

void _ui_init_frame1_Ungroup_10() {
    ui_frame1_Ungroup_10.option.figure_name[0] = 1;
    ui_frame1_Ungroup_10.option.figure_name[1] = 0;
    ui_frame1_Ungroup_10.option.figure_name[2] = 23;
    ui_frame1_Ungroup_10.option.operate_type = 1;

    ui_frame1_Ungroup_leg_mode->figure_type = 7;
    ui_frame1_Ungroup_leg_mode->operate_type = 1;
    ui_frame1_Ungroup_leg_mode->layer = 0;
    ui_frame1_Ungroup_leg_mode->color = 4;
    ui_frame1_Ungroup_leg_mode->start_x = 1775;
    ui_frame1_Ungroup_leg_mode->start_y = 668;
    ui_frame1_Ungroup_leg_mode->width = 2;
    ui_frame1_Ungroup_leg_mode->font_size = 20;
    ui_frame1_Ungroup_leg_mode->str_length = 4;
    strcpy(ui_frame1_Ungroup_leg_mode->string, "less");


    ui_proc_string_frame(&ui_frame1_Ungroup_10);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_10, sizeof(ui_frame1_Ungroup_10));
}

void _ui_update_frame1_Ungroup_10() {
    ui_frame1_Ungroup_10.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_Ungroup_10);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_10, sizeof(ui_frame1_Ungroup_10));
}

void _ui_remove_frame1_Ungroup_10() {
    ui_frame1_Ungroup_10.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_Ungroup_10);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_10, sizeof(ui_frame1_Ungroup_10));
}
ui_string_frame_t ui_frame1_Ungroup_11;
ui_interface_string_t* ui_frame1_Ungroup_VISION = &(ui_frame1_Ungroup_11.option);

void _ui_init_frame1_Ungroup_11() {
    ui_frame1_Ungroup_11.option.figure_name[0] = 1;
    ui_frame1_Ungroup_11.option.figure_name[1] = 0;
    ui_frame1_Ungroup_11.option.figure_name[2] = 24;
    ui_frame1_Ungroup_11.option.operate_type = 1;

    ui_frame1_Ungroup_VISION->figure_type = 7;
    ui_frame1_Ungroup_VISION->operate_type = 1;
    ui_frame1_Ungroup_VISION->layer = 0;
    ui_frame1_Ungroup_VISION->color = 2;
    ui_frame1_Ungroup_VISION->start_x = 31;
    ui_frame1_Ungroup_VISION->start_y = 820;
    ui_frame1_Ungroup_VISION->width = 1;
    ui_frame1_Ungroup_VISION->font_size = 12;
    ui_frame1_Ungroup_VISION->str_length = 6;
    strcpy(ui_frame1_Ungroup_VISION->string, "VISION");


    ui_proc_string_frame(&ui_frame1_Ungroup_11);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_11, sizeof(ui_frame1_Ungroup_11));
}

void _ui_update_frame1_Ungroup_11() {
    ui_frame1_Ungroup_11.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_Ungroup_11);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_11, sizeof(ui_frame1_Ungroup_11));
}

void _ui_remove_frame1_Ungroup_11() {
    ui_frame1_Ungroup_11.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_Ungroup_11);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_11, sizeof(ui_frame1_Ungroup_11));
}
ui_string_frame_t ui_frame1_Ungroup_12;
ui_interface_string_t* ui_frame1_Ungroup_mode_text = &(ui_frame1_Ungroup_12.option);

void _ui_init_frame1_Ungroup_12() {
    ui_frame1_Ungroup_12.option.figure_name[0] = 1;
    ui_frame1_Ungroup_12.option.figure_name[1] = 0;
    ui_frame1_Ungroup_12.option.figure_name[2] = 25;
    ui_frame1_Ungroup_12.option.operate_type = 1;

    ui_frame1_Ungroup_mode_text->figure_type = 7;
    ui_frame1_Ungroup_mode_text->operate_type = 1;
    ui_frame1_Ungroup_mode_text->layer = 0;
    ui_frame1_Ungroup_mode_text->color = 3;
    ui_frame1_Ungroup_mode_text->start_x = 132;
    ui_frame1_Ungroup_mode_text->start_y = 740;
    ui_frame1_Ungroup_mode_text->width = 1;
    ui_frame1_Ungroup_mode_text->font_size = 12;
    ui_frame1_Ungroup_mode_text->str_length = 4;
    strcpy(ui_frame1_Ungroup_mode_text->string, "zero");


    ui_proc_string_frame(&ui_frame1_Ungroup_12);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_12, sizeof(ui_frame1_Ungroup_12));
}

void _ui_update_frame1_Ungroup_12() {
    ui_frame1_Ungroup_12.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_Ungroup_12);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_12, sizeof(ui_frame1_Ungroup_12));
}

void _ui_remove_frame1_Ungroup_12() {
    ui_frame1_Ungroup_12.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_Ungroup_12);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_12, sizeof(ui_frame1_Ungroup_12));
}
ui_string_frame_t ui_frame1_Ungroup_13;
ui_interface_string_t* ui_frame1_Ungroup_vision_text = &(ui_frame1_Ungroup_13.option);

void _ui_init_frame1_Ungroup_13() {
    ui_frame1_Ungroup_13.option.figure_name[0] = 1;
    ui_frame1_Ungroup_13.option.figure_name[1] = 0;
    ui_frame1_Ungroup_13.option.figure_name[2] = 26;
    ui_frame1_Ungroup_13.option.operate_type = 1;

    ui_frame1_Ungroup_vision_text->figure_type = 7;
    ui_frame1_Ungroup_vision_text->operate_type = 1;
    ui_frame1_Ungroup_vision_text->layer = 0;
    ui_frame1_Ungroup_vision_text->color = 3;
    ui_frame1_Ungroup_vision_text->start_x = 140;
    ui_frame1_Ungroup_vision_text->start_y = 815;
    ui_frame1_Ungroup_vision_text->width = 1;
    ui_frame1_Ungroup_vision_text->font_size = 12;
    ui_frame1_Ungroup_vision_text->str_length = 2;
    strcpy(ui_frame1_Ungroup_vision_text->string, "ON");


    ui_proc_string_frame(&ui_frame1_Ungroup_13);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_13, sizeof(ui_frame1_Ungroup_13));
}

void _ui_update_frame1_Ungroup_13() {
    ui_frame1_Ungroup_13.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_Ungroup_13);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_13, sizeof(ui_frame1_Ungroup_13));
}

void _ui_remove_frame1_Ungroup_13() {
    ui_frame1_Ungroup_13.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_Ungroup_13);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_13, sizeof(ui_frame1_Ungroup_13));
}
ui_string_frame_t ui_frame1_Ungroup_14;
ui_interface_string_t* ui_frame1_Ungroup_loss_control_text = &(ui_frame1_Ungroup_14.option);

void _ui_init_frame1_Ungroup_14() {
    ui_frame1_Ungroup_14.option.figure_name[0] = 1;
    ui_frame1_Ungroup_14.option.figure_name[1] = 0;
    ui_frame1_Ungroup_14.option.figure_name[2] = 27;
    ui_frame1_Ungroup_14.option.operate_type = 1;

    ui_frame1_Ungroup_loss_control_text->figure_type = 7;
    ui_frame1_Ungroup_loss_control_text->operate_type = 1;
    ui_frame1_Ungroup_loss_control_text->layer = 0;
    ui_frame1_Ungroup_loss_control_text->color = 3;
    ui_frame1_Ungroup_loss_control_text->start_x = 221;
    ui_frame1_Ungroup_loss_control_text->start_y = 682;
    ui_frame1_Ungroup_loss_control_text->width = 2;
    ui_frame1_Ungroup_loss_control_text->font_size = 15;
    ui_frame1_Ungroup_loss_control_text->str_length = 6;
    strcpy(ui_frame1_Ungroup_loss_control_text->string, "normal");


    ui_proc_string_frame(&ui_frame1_Ungroup_14);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_14, sizeof(ui_frame1_Ungroup_14));
}

void _ui_update_frame1_Ungroup_14() {
    ui_frame1_Ungroup_14.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_Ungroup_14);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_14, sizeof(ui_frame1_Ungroup_14));
}

void _ui_remove_frame1_Ungroup_14() {
    ui_frame1_Ungroup_14.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_Ungroup_14);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_14, sizeof(ui_frame1_Ungroup_14));
}
ui_string_frame_t ui_frame1_Ungroup_15;
ui_interface_string_t* ui_frame1_Ungroup_left_leg = &(ui_frame1_Ungroup_15.option);

void _ui_init_frame1_Ungroup_15() {
    ui_frame1_Ungroup_15.option.figure_name[0] = 1;
    ui_frame1_Ungroup_15.option.figure_name[1] = 0;
    ui_frame1_Ungroup_15.option.figure_name[2] = 28;
    ui_frame1_Ungroup_15.option.operate_type = 1;

    ui_frame1_Ungroup_left_leg->figure_type = 7;
    ui_frame1_Ungroup_left_leg->operate_type = 1;
    ui_frame1_Ungroup_left_leg->layer = 0;
    ui_frame1_Ungroup_left_leg->color = 1;
    ui_frame1_Ungroup_left_leg->start_x = 1476;
    ui_frame1_Ungroup_left_leg->start_y = 667;
    ui_frame1_Ungroup_left_leg->width = 2;
    ui_frame1_Ungroup_left_leg->font_size = 15;
    ui_frame1_Ungroup_left_leg->str_length = 8;
    strcpy(ui_frame1_Ungroup_left_leg->string, "left_leg");


    ui_proc_string_frame(&ui_frame1_Ungroup_15);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_15, sizeof(ui_frame1_Ungroup_15));
}

void _ui_update_frame1_Ungroup_15() {
    ui_frame1_Ungroup_15.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_Ungroup_15);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_15, sizeof(ui_frame1_Ungroup_15));
}

void _ui_remove_frame1_Ungroup_15() {
    ui_frame1_Ungroup_15.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_Ungroup_15);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_15, sizeof(ui_frame1_Ungroup_15));
}
ui_string_frame_t ui_frame1_Ungroup_16;
ui_interface_string_t* ui_frame1_Ungroup_CAP = &(ui_frame1_Ungroup_16.option);

void _ui_init_frame1_Ungroup_16() {
    ui_frame1_Ungroup_16.option.figure_name[0] = 1;
    ui_frame1_Ungroup_16.option.figure_name[1] = 0;
    ui_frame1_Ungroup_16.option.figure_name[2] = 29;
    ui_frame1_Ungroup_16.option.operate_type = 1;

    ui_frame1_Ungroup_CAP->figure_type = 7;
    ui_frame1_Ungroup_CAP->operate_type = 1;
    ui_frame1_Ungroup_CAP->layer = 0;
    ui_frame1_Ungroup_CAP->color = 2;
    ui_frame1_Ungroup_CAP->start_x = 35;
    ui_frame1_Ungroup_CAP->start_y = 778;
    ui_frame1_Ungroup_CAP->width = 1;
    ui_frame1_Ungroup_CAP->font_size = 12;
    ui_frame1_Ungroup_CAP->str_length = 3;
    strcpy(ui_frame1_Ungroup_CAP->string, "CAP");


    ui_proc_string_frame(&ui_frame1_Ungroup_16);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_16, sizeof(ui_frame1_Ungroup_16));
}

void _ui_update_frame1_Ungroup_16() {
    ui_frame1_Ungroup_16.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_Ungroup_16);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_16, sizeof(ui_frame1_Ungroup_16));
}

void _ui_remove_frame1_Ungroup_16() {
    ui_frame1_Ungroup_16.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_Ungroup_16);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_16, sizeof(ui_frame1_Ungroup_16));
}

void ui_init_frame1_Ungroup() {
    _ui_init_frame1_Ungroup_0();
    _ui_init_frame1_Ungroup_1();
    _ui_init_frame1_Ungroup_2();
    _ui_init_frame1_Ungroup_3();
    _ui_init_frame1_Ungroup_4();
    _ui_init_frame1_Ungroup_5();
    _ui_init_frame1_Ungroup_6();
    _ui_init_frame1_Ungroup_7();
    _ui_init_frame1_Ungroup_8();
    _ui_init_frame1_Ungroup_9();
    _ui_init_frame1_Ungroup_10();
    _ui_init_frame1_Ungroup_11();
    _ui_init_frame1_Ungroup_12();
    _ui_init_frame1_Ungroup_13();
    _ui_init_frame1_Ungroup_14();
    _ui_init_frame1_Ungroup_15();
    _ui_init_frame1_Ungroup_16();
}

void ui_update_frame1_Ungroup() {
    _ui_update_frame1_Ungroup_0();
    _ui_update_frame1_Ungroup_1();
    _ui_update_frame1_Ungroup_2();
    _ui_update_frame1_Ungroup_3();
    _ui_update_frame1_Ungroup_4();
    _ui_update_frame1_Ungroup_5();
    _ui_update_frame1_Ungroup_6();
    _ui_update_frame1_Ungroup_7();
    _ui_update_frame1_Ungroup_8();
    _ui_update_frame1_Ungroup_9();
    _ui_update_frame1_Ungroup_10();
    _ui_update_frame1_Ungroup_11();
    _ui_update_frame1_Ungroup_12();
    _ui_update_frame1_Ungroup_13();
    _ui_update_frame1_Ungroup_14();
    _ui_update_frame1_Ungroup_15();
    _ui_update_frame1_Ungroup_16();
}

void ui_remove_frame1_Ungroup() {
    _ui_remove_frame1_Ungroup_0();
    _ui_remove_frame1_Ungroup_1();
    _ui_remove_frame1_Ungroup_2();
    _ui_remove_frame1_Ungroup_3();
    _ui_remove_frame1_Ungroup_4();
    _ui_remove_frame1_Ungroup_5();
    _ui_remove_frame1_Ungroup_6();
    _ui_remove_frame1_Ungroup_7();
    _ui_remove_frame1_Ungroup_8();
    _ui_remove_frame1_Ungroup_9();
    _ui_remove_frame1_Ungroup_10();
    _ui_remove_frame1_Ungroup_11();
    _ui_remove_frame1_Ungroup_12();
    _ui_remove_frame1_Ungroup_13();
    _ui_remove_frame1_Ungroup_14();
    _ui_remove_frame1_Ungroup_15();
    _ui_remove_frame1_Ungroup_16();
}

