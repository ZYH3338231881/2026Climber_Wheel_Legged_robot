/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       music_typedef.h
  * @brief      music部分的内部使用的相关定义
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-04-03      Penguin         1.初始化
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
**/
#ifndef MUSIC_TYPEDEF_H
#define MUSIC_TYPEDEF_H

#include "struct_typedef.h"

// 填充音符 NOTE:音符频率(Hz) LONE:音符时长(ms)
#define WRITE_NOTE(NOTE, LONG)                                     \
    {                                                              \
        MUSIC_INFO.notes[MUSIC_INFO.last_note_id + 1].note = NOTE; \
        MUSIC_INFO.notes[MUSIC_INFO.last_note_id + 1].Long = LONG; \
        MUSIC_INFO.notes[MUSIC_INFO.last_note_id + 1].end =        \
            MUSIC_INFO.notes[MUSIC_INFO.last_note_id].end + LONG;  \
        MUSIC_INFO.last_note_id++;                                 \
    }

// 空拍
#define SLEEP_NOTE(LONG) WRITE_NOTE(0, LONG)

typedef struct
{
    int note;
    float Long;
    uint32_t end;
} Note;

typedef struct
{
    Note * notes;           // 音符列表
    uint32_t last_note_id;  // 结尾音符的index
} MusicInfo_s;

#endif  // MUSIC_TYPEDEF_H
/*------------------------------ End of File ------------------------------*/