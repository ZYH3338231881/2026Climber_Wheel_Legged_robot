/**
 * @file bsp_BuzzerSongs.h
 * @brief 蜂鸣器歌曲 C语言版
 */

#ifndef BSP_BUZZER_SONGS_H
#define BSP_BUZZER_SONGS_H

#include "bsp_buzzer.h"
#include <stdint.h>

typedef struct
{
    float frequency;
    float beat;
} BuzzerSongNote_t;

void BuzzerSongs_Play(const BuzzerSongNote_t *song, uint32_t note_num, uint32_t unit_ms, float loudness);
void BuzzerSongs_Play_Gala_You(void);
void BuzzerSongs_Play_Godfather(void);
void BuzzerSongs_Play_See_You_Again(void);

#endif