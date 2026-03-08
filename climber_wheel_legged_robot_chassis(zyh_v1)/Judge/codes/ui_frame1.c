//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_7_frame_t ui_frame1_Ungroup_0;

ui_interface_rect_t *ui_frame1_Ungroup_Aim_range = (ui_interface_rect_t*)&(ui_frame1_Ungroup_0.data[0]);
ui_interface_line_t *ui_frame1_Ungroup_JUMP2 = (ui_interface_line_t*)&(ui_frame1_Ungroup_0.data[1]);
ui_interface_number_t *ui_frame1_Ungroup_velFloat = (ui_interface_number_t*)&(ui_frame1_Ungroup_0.data[2]);
ui_interface_number_t *ui_frame1_Ungroup_goryFloat = (ui_interface_number_t*)&(ui_frame1_Ungroup_0.data[3]);
ui_interface_line_t *ui_frame1_Ungroup_JUMP0 = (ui_interface_line_t*)&(ui_frame1_Ungroup_0.data[4]);
ui_interface_line_t *ui_frame1_Ungroup_JUMP1 = (ui_interface_line_t*)&(ui_frame1_Ungroup_0.data[5]);
ui_interface_ellipse_t *ui_frame1_Ungroup_NewEllipse = (ui_interface_ellipse_t*)&(ui_frame1_Ungroup_0.data[6]);

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
    ui_frame1_Ungroup_Aim_range->start_x = 551;
    ui_frame1_Ungroup_Aim_range->start_y = 267;
    ui_frame1_Ungroup_Aim_range->width = 2;
    ui_frame1_Ungroup_Aim_range->end_x = 1357;
    ui_frame1_Ungroup_Aim_range->end_y = 880;

    ui_frame1_Ungroup_JUMP2->figure_type = 0;
    ui_frame1_Ungroup_JUMP2->operate_type = 1;
    ui_frame1_Ungroup_JUMP2->layer = 0;
    ui_frame1_Ungroup_JUMP2->color = 1;
    ui_frame1_Ungroup_JUMP2->start_x = 1368;
    ui_frame1_Ungroup_JUMP2->start_y = 6;
    ui_frame1_Ungroup_JUMP2->width = 6;
    ui_frame1_Ungroup_JUMP2->end_x = 1201;
    ui_frame1_Ungroup_JUMP2->end_y = 404;

    ui_frame1_Ungroup_velFloat->figure_type = 5;
    ui_frame1_Ungroup_velFloat->operate_type = 1;
    ui_frame1_Ungroup_velFloat->layer = 0;
    ui_frame1_Ungroup_velFloat->color = 0;
    ui_frame1_Ungroup_velFloat->start_x = 118;
    ui_frame1_Ungroup_velFloat->start_y = 658;
    ui_frame1_Ungroup_velFloat->width = 2;
    ui_frame1_Ungroup_velFloat->font_size = 20;
    ui_frame1_Ungroup_velFloat->number = 12345;

    ui_frame1_Ungroup_goryFloat->figure_type = 5;
    ui_frame1_Ungroup_goryFloat->operate_type = 1;
    ui_frame1_Ungroup_goryFloat->layer = 0;
    ui_frame1_Ungroup_goryFloat->color = 0;
    ui_frame1_Ungroup_goryFloat->start_x = 118;
    ui_frame1_Ungroup_goryFloat->start_y = 622;
    ui_frame1_Ungroup_goryFloat->width = 2;
    ui_frame1_Ungroup_goryFloat->font_size = 20;
    ui_frame1_Ungroup_goryFloat->number = 12345;

    ui_frame1_Ungroup_JUMP0->figure_type = 0;
    ui_frame1_Ungroup_JUMP0->operate_type = 1;
    ui_frame1_Ungroup_JUMP0->layer = 0;
    ui_frame1_Ungroup_JUMP0->color = 1;
    ui_frame1_Ungroup_JUMP0->start_x = 562;
    ui_frame1_Ungroup_JUMP0->start_y = 0;
    ui_frame1_Ungroup_JUMP0->width = 6;
    ui_frame1_Ungroup_JUMP0->end_x = 716;
    ui_frame1_Ungroup_JUMP0->end_y = 405;

    ui_frame1_Ungroup_JUMP1->figure_type = 0;
    ui_frame1_Ungroup_JUMP1->operate_type = 1;
    ui_frame1_Ungroup_JUMP1->layer = 0;
    ui_frame1_Ungroup_JUMP1->color = 1;
    ui_frame1_Ungroup_JUMP1->start_x = 715;
    ui_frame1_Ungroup_JUMP1->start_y = 402;
    ui_frame1_Ungroup_JUMP1->width = 6;
    ui_frame1_Ungroup_JUMP1->end_x = 1204;
    ui_frame1_Ungroup_JUMP1->end_y = 402;

    ui_frame1_Ungroup_NewEllipse->figure_type = 3;
    ui_frame1_Ungroup_NewEllipse->operate_type = 1;
    ui_frame1_Ungroup_NewEllipse->layer = 0;
    ui_frame1_Ungroup_NewEllipse->color = 7;
    ui_frame1_Ungroup_NewEllipse->start_x = 961;
    ui_frame1_Ungroup_NewEllipse->start_y = 538;
    ui_frame1_Ungroup_NewEllipse->width = 5;
    ui_frame1_Ungroup_NewEllipse->rx = 76;
    ui_frame1_Ungroup_NewEllipse->ry = 76;


    ui_proc_7_frame(&ui_frame1_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_0, sizeof(ui_frame1_Ungroup_0));
}

void _ui_update_frame1_Ungroup_0() {
    for (int i = 0; i < 7; i++) {
        ui_frame1_Ungroup_0.data[i].operate_type = 2;
    }

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

ui_string_frame_t ui_frame1_Ungroup_1;
ui_interface_string_t* ui_frame1_Ungroup_Aim_Range_word = &(ui_frame1_Ungroup_1.option);

void _ui_init_frame1_Ungroup_1() {
    ui_frame1_Ungroup_1.option.figure_name[0] = 1;
    ui_frame1_Ungroup_1.option.figure_name[1] = 0;
    ui_frame1_Ungroup_1.option.figure_name[2] = 7;
    ui_frame1_Ungroup_1.option.operate_type = 1;

    ui_frame1_Ungroup_Aim_Range_word->figure_type = 7;
    ui_frame1_Ungroup_Aim_Range_word->operate_type = 1;
    ui_frame1_Ungroup_Aim_Range_word->layer = 0;
    ui_frame1_Ungroup_Aim_Range_word->color = 0;
    ui_frame1_Ungroup_Aim_Range_word->start_x = 558;
    ui_frame1_Ungroup_Aim_Range_word->start_y = 881;
    ui_frame1_Ungroup_Aim_Range_word->width = 2;
    ui_frame1_Ungroup_Aim_Range_word->font_size = 20;
    ui_frame1_Ungroup_Aim_Range_word->str_length = 9;
    strcpy(ui_frame1_Ungroup_Aim_Range_word->string, "Aim Range");


    ui_proc_string_frame(&ui_frame1_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_1, sizeof(ui_frame1_Ungroup_1));
}

void _ui_update_frame1_Ungroup_1() {
    ui_frame1_Ungroup_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_1, sizeof(ui_frame1_Ungroup_1));
}

void _ui_remove_frame1_Ungroup_1() {
    ui_frame1_Ungroup_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_1, sizeof(ui_frame1_Ungroup_1));
}
ui_string_frame_t ui_frame1_Ungroup_2;
ui_interface_string_t* ui_frame1_Ungroup_MODE = &(ui_frame1_Ungroup_2.option);

void _ui_init_frame1_Ungroup_2() {
    ui_frame1_Ungroup_2.option.figure_name[0] = 1;
    ui_frame1_Ungroup_2.option.figure_name[1] = 0;
    ui_frame1_Ungroup_2.option.figure_name[2] = 8;
    ui_frame1_Ungroup_2.option.operate_type = 1;

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


    ui_proc_string_frame(&ui_frame1_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_2, sizeof(ui_frame1_Ungroup_2));
}

void _ui_update_frame1_Ungroup_2() {
    ui_frame1_Ungroup_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_2, sizeof(ui_frame1_Ungroup_2));
}

void _ui_remove_frame1_Ungroup_2() {
    ui_frame1_Ungroup_2.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_frame1_Ungroup_2, sizeof(ui_frame1_Ungroup_2));
}
ui_string_frame_t ui_frame1_Ungroup_3;
ui_interface_string_t* ui_frame1_Ungroup_loss_control = &(ui_frame1_Ungroup_3.option);

void _ui_init_frame1_Ungroup_3() {
    ui_frame1_Ungroup_3.option.figure_name[0] = 1;
    ui_frame1_Ungroup_3.option.figure_name[1] = 0;
    ui_frame1_Ungroup_3.option.figure_name[2] = 9;
    ui_frame1_Ungroup_3.option.operate_type = 1;

    ui_frame1_Ungroup_loss_control->figure_type = 7;
    ui_frame1_Ungroup_loss_control->operate_type = 1;
    ui_frame1_Ungroup_loss_control->layer = 0;
    ui_frame1_Ungroup_loss_control->color = 0;
    ui_frame1_Ungroup_loss_control->start_x = 10;
    ui_frame1_Ungroup_loss_control->start_y = 697;
    ui_frame1_Ungroup_loss_control->width = 2;
    ui_frame1_Ungroup_loss_control->font_size = 15;
    ui_frame1_Ungroup_loss_control->str_length = 12;
    strcpy(ui_frame1_Ungroup_loss_control->string, "loss_control");


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
ui_interface_string_t* ui_frame1_Ungroup_yaw_offest = &(ui_frame1_Ungroup_4.option);

void _ui_init_frame1_Ungroup_4() {
    ui_frame1_Ungroup_4.option.figure_name[0] = 1;
    ui_frame1_Ungroup_4.option.figure_name[1] = 0;
    ui_frame1_Ungroup_4.option.figure_name[2] = 10;
    ui_frame1_Ungroup_4.option.operate_type = 1;

    ui_frame1_Ungroup_yaw_offest->figure_type = 7;
    ui_frame1_Ungroup_yaw_offest->operate_type = 1;
    ui_frame1_Ungroup_yaw_offest->layer = 0;
    ui_frame1_Ungroup_yaw_offest->color = 8;
    ui_frame1_Ungroup_yaw_offest->start_x = 1534;
    ui_frame1_Ungroup_yaw_offest->start_y = 726;
    ui_frame1_Ungroup_yaw_offest->width = 2;
    ui_frame1_Ungroup_yaw_offest->font_size = 20;
    ui_frame1_Ungroup_yaw_offest->str_length = 10;
    strcpy(ui_frame1_Ungroup_yaw_offest->string, "yaw_offest");


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
ui_interface_string_t* ui_frame1_Ungroup_Vel = &(ui_frame1_Ungroup_5.option);

void _ui_init_frame1_Ungroup_5() {
    ui_frame1_Ungroup_5.option.figure_name[0] = 1;
    ui_frame1_Ungroup_5.option.figure_name[1] = 0;
    ui_frame1_Ungroup_5.option.figure_name[2] = 11;
    ui_frame1_Ungroup_5.option.operate_type = 1;

    ui_frame1_Ungroup_Vel->figure_type = 7;
    ui_frame1_Ungroup_Vel->operate_type = 1;
    ui_frame1_Ungroup_Vel->layer = 0;
    ui_frame1_Ungroup_Vel->color = 6;
    ui_frame1_Ungroup_Vel->start_x = 29;
    ui_frame1_Ungroup_Vel->start_y = 650;
    ui_frame1_Ungroup_Vel->width = 2;
    ui_frame1_Ungroup_Vel->font_size = 15;
    ui_frame1_Ungroup_Vel->str_length = 3;
    strcpy(ui_frame1_Ungroup_Vel->string, "Vel");


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
ui_interface_string_t* ui_frame1_Ungroup_GORY = &(ui_frame1_Ungroup_6.option);

void _ui_init_frame1_Ungroup_6() {
    ui_frame1_Ungroup_6.option.figure_name[0] = 1;
    ui_frame1_Ungroup_6.option.figure_name[1] = 0;
    ui_frame1_Ungroup_6.option.figure_name[2] = 12;
    ui_frame1_Ungroup_6.option.operate_type = 1;

    ui_frame1_Ungroup_GORY->figure_type = 7;
    ui_frame1_Ungroup_GORY->operate_type = 1;
    ui_frame1_Ungroup_GORY->layer = 0;
    ui_frame1_Ungroup_GORY->color = 6;
    ui_frame1_Ungroup_GORY->start_x = 29;
    ui_frame1_Ungroup_GORY->start_y = 618;
    ui_frame1_Ungroup_GORY->width = 2;
    ui_frame1_Ungroup_GORY->font_size = 15;
    ui_frame1_Ungroup_GORY->str_length = 4;
    strcpy(ui_frame1_Ungroup_GORY->string, "GORY");


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
ui_interface_string_t* ui_frame1_Ungroup_PITCH = &(ui_frame1_Ungroup_7.option);

void _ui_init_frame1_Ungroup_7() {
    ui_frame1_Ungroup_7.option.figure_name[0] = 1;
    ui_frame1_Ungroup_7.option.figure_name[1] = 0;
    ui_frame1_Ungroup_7.option.figure_name[2] = 13;
    ui_frame1_Ungroup_7.option.operate_type = 1;

    ui_frame1_Ungroup_PITCH->figure_type = 7;
    ui_frame1_Ungroup_PITCH->operate_type = 1;
    ui_frame1_Ungroup_PITCH->layer = 0;
    ui_frame1_Ungroup_PITCH->color = 8;
    ui_frame1_Ungroup_PITCH->start_x = 1534;
    ui_frame1_Ungroup_PITCH->start_y = 821;
    ui_frame1_Ungroup_PITCH->width = 2;
    ui_frame1_Ungroup_PITCH->font_size = 20;
    ui_frame1_Ungroup_PITCH->str_length = 5;
    strcpy(ui_frame1_Ungroup_PITCH->string, "PITCH");


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
ui_interface_string_t* ui_frame1_Ungroup_VISION = &(ui_frame1_Ungroup_8.option);

void _ui_init_frame1_Ungroup_8() {
    ui_frame1_Ungroup_8.option.figure_name[0] = 1;
    ui_frame1_Ungroup_8.option.figure_name[1] = 0;
    ui_frame1_Ungroup_8.option.figure_name[2] = 14;
    ui_frame1_Ungroup_8.option.operate_type = 1;

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
ui_interface_string_t* ui_frame1_Ungroup_left_leg = &(ui_frame1_Ungroup_9.option);

void _ui_init_frame1_Ungroup_9() {
    ui_frame1_Ungroup_9.option.figure_name[0] = 1;
    ui_frame1_Ungroup_9.option.figure_name[1] = 0;
    ui_frame1_Ungroup_9.option.figure_name[2] = 15;
    ui_frame1_Ungroup_9.option.operate_type = 1;

    ui_frame1_Ungroup_left_leg->figure_type = 7;
    ui_frame1_Ungroup_left_leg->operate_type = 1;
    ui_frame1_Ungroup_left_leg->layer = 0;
    ui_frame1_Ungroup_left_leg->color = 5;
    ui_frame1_Ungroup_left_leg->start_x = 631;
    ui_frame1_Ungroup_left_leg->start_y = 101;
    ui_frame1_Ungroup_left_leg->width = 1;
    ui_frame1_Ungroup_left_leg->font_size = 10;
    ui_frame1_Ungroup_left_leg->str_length = 8;
    strcpy(ui_frame1_Ungroup_left_leg->string, "left_leg");


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
ui_interface_string_t* ui_frame1_Ungroup_right_leg = &(ui_frame1_Ungroup_10.option);

void _ui_init_frame1_Ungroup_10() {
    ui_frame1_Ungroup_10.option.figure_name[0] = 1;
    ui_frame1_Ungroup_10.option.figure_name[1] = 0;
    ui_frame1_Ungroup_10.option.figure_name[2] = 16;
    ui_frame1_Ungroup_10.option.operate_type = 1;

    ui_frame1_Ungroup_right_leg->figure_type = 7;
    ui_frame1_Ungroup_right_leg->operate_type = 1;
    ui_frame1_Ungroup_right_leg->layer = 0;
    ui_frame1_Ungroup_right_leg->color = 5;
    ui_frame1_Ungroup_right_leg->start_x = 1139;
    ui_frame1_Ungroup_right_leg->start_y = 94;
    ui_frame1_Ungroup_right_leg->width = 1;
    ui_frame1_Ungroup_right_leg->font_size = 10;
    ui_frame1_Ungroup_right_leg->str_length = 9;
    strcpy(ui_frame1_Ungroup_right_leg->string, "right_leg");


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
ui_interface_string_t* ui_frame1_Ungroup_CAP = &(ui_frame1_Ungroup_11.option);

void _ui_init_frame1_Ungroup_11() {
    ui_frame1_Ungroup_11.option.figure_name[0] = 1;
    ui_frame1_Ungroup_11.option.figure_name[1] = 0;
    ui_frame1_Ungroup_11.option.figure_name[2] = 17;
    ui_frame1_Ungroup_11.option.operate_type = 1;

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
}

