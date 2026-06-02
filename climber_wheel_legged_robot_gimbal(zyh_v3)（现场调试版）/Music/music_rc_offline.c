
#include "music_rc_offline.h"

#include "bsp_buzzer.h"
#include "music.h"
#include "stm32f4xx_hal.h"

#define NOTE_START 1000
#define NOTE_ON 400
#define NOTE_OFF 0

#define NOTE_LONE 250
#define SLEEP_LONE 70

#define NOTE_NUM 10
static Note Notes[NOTE_NUM];  // Array of notes
static MusicInfo_s MUSIC_INFO;

/*-------------------- User functions --------------------*/

MusicInfo_s MusicRcOfflineInit(void)
{
    MUSIC_INFO.notes = Notes;

    WRITE_NOTE(1200, 100);

    // ßÙ-
    WRITE_NOTE(NOTE_ON, 400);

    WRITE_NOTE(1200, 100);
    WRITE_NOTE(NOTE_OFF, 100);

    return MUSIC_INFO;
}
