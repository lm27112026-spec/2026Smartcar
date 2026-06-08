
#include "music_referee.h"

#include "bsp_buzzer.h"
//#include "music.h"
#include "stm32f4xx_hal.h"
#include "music_typedef.h"

// clang-format off
//定义低音  
#define A1  131
#define A2  147
#define A3  165
#define A4  175
#define A5  196
#define A6  220
#define A7  247
  
//定义中音  
#define B1  262
#define B2  296
#define B3  330
#define B4  349
#define B5  392
#define B6  440
#define B7  494
  
//定义高音  
#define C1  523
#define C2  587
#define C3  659
#define C4  698
#define C4p 741
#define C5  784
#define C6  880
#define C7  988
  
//定义高二度  
#define D1  1047
#define D2  1175
#define D3  1319
#define D4  1397
#define D5  1568
#define D6  1760
#define D7  1976
  
//定义节拍  
#define OneBeat   200//一拍子两个1beat 
#define HalfBeat  100
// clang-format on

static Note CONNECT_NOTES[10];
static Note DISCONNECT_NOTES[10];

static uint32_t last_note_id = 0;        // Index of the last note
static uint32_t write_id = 1;            // Index of the note to be written
static uint32_t play_id_connect = 1;     // Index of the note to be played
static uint32_t play_id_disconnect = 1;  // Index of the note to be played

static uint32_t start_time = 0;  // Start time of the music
static uint32_t now = 0;

/*-------------------- Private functions --------------------*/
static void WriteConnectNote(int note, float Long)
{
    CONNECT_NOTES[write_id].note = note;
    CONNECT_NOTES[write_id].Long = Long;
    CONNECT_NOTES[write_id].end = CONNECT_NOTES[write_id - 1].end + Long;
    write_id++;
}

static void WriteDisconnectNote(int note, float Long)
{
    DISCONNECT_NOTES[write_id].note = note;
    DISCONNECT_NOTES[write_id].Long = Long;
    DISCONNECT_NOTES[write_id].end = DISCONNECT_NOTES[write_id - 1].end + Long;
    write_id++;
}

/*-------------------- User functions --------------------*/

bool MusicRefereeConncetPlay()
{
    now = HAL_GetTick();

    bool end = false;
    if (now - start_time >= CONNECT_NOTES[play_id_connect].end) {
        play_id_connect++;
        if (play_id_connect > last_note_id) {
            end = true;
            play_id_connect = 1;
            start_time = now;
        }

        buzzer_note(CONNECT_NOTES[play_id_connect].note, 0.05);
    }
    return end;
}

bool MusicRefereeDisconnectPlay()
{
    now = HAL_GetTick();

    bool end = false;
    if (now - start_time >= DISCONNECT_NOTES[play_id_disconnect].end) {
        play_id_disconnect++;
        if (play_id_disconnect > last_note_id) {
            end = true;
            play_id_disconnect = 1;
            start_time = now;
        }

        buzzer_note(DISCONNECT_NOTES[play_id_disconnect].note, 0.05);
    }
    return end;
}

void MusicRefereeInit(void)
{
    // connect
    WriteConnectNote(0, 2);
    WriteConnectNote(C3, HalfBeat * 2);
    WriteConnectNote(0, 3);
    WriteConnectNote(B3, HalfBeat * 3);
    WriteConnectNote(0, 3);
    WriteConnectNote(B5, HalfBeat * 4);
    WriteConnectNote(0, 500);

    last_note_id = write_id - 1;
    write_id = 1;

    // disconnect
    WriteDisconnectNote(0, 2);
    WriteDisconnectNote(D5, HalfBeat * 2);
    WriteDisconnectNote(0, 3);
    WriteDisconnectNote(B3, HalfBeat * 4);
    WriteDisconnectNote(0, 500);
    last_note_id = write_id - 1;
    write_id = 1;
}
