#include "music_gong_xi_fa_cai.h"

#include "bsp_buzzer.h"
//#include "music.h"
#include "stm32f4xx_hal.h"

//http://www.jianpu.cn/pu/86/86325.htm

// clang-format off
#define Z_no 0
#define A_do 440
#define A_re 494
#define A_mi 554
#define A_fa 587
#define A_so 659
#define A_la 740
#define A_si 831
// clang-format on

#define ONE_FOURTH 250
#define ONE_FOURTH_HALF 125
#define ONE_FOURTH_HALF_HALF 63

#define NOTE_NUM 120
static Note Notes[NOTE_NUM];  // Array of notes
static MusicInfo_s MUSIC_INFO;

/*-------------------- User functions --------------------*/

MusicInfo_s MusicGongXiFaCaiInit(void)
{
    MUSIC_INFO.notes = Notes;

    // do re mi fa so la si
    // 1  2  3  4  5  6  7

    // 前奏
    // 2·_ 3__ 2_ 5_ 2_ 3_ 2
    WRITE_NOTE(A_re, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 2·_
    WRITE_NOTE(A_mi, ONE_FOURTH_HALF_HALF);                    // 3__
    WRITE_NOTE(A_re, ONE_FOURTH_HALF);                         // 2_
    WRITE_NOTE(A_so, ONE_FOURTH_HALF);                         // 5_
    WRITE_NOTE(A_re, ONE_FOURTH_HALF);                         // 2_
    WRITE_NOTE(A_mi, ONE_FOURTH_HALF);                         // 3_
    WRITE_NOTE(A_re, ONE_FOURTH);                              // 2

    // 2·_ 3__ 2_ 1_ 6._ 1_ 6.
    WRITE_NOTE(A_re, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 2·_
    WRITE_NOTE(A_mi, ONE_FOURTH_HALF_HALF);                    // 3__
    WRITE_NOTE(A_re, ONE_FOURTH_HALF);                         // 2_
    WRITE_NOTE(A_do, ONE_FOURTH_HALF);                         // 1_
    WRITE_NOTE(A_la / 2, ONE_FOURTH_HALF);                     // 6._
    WRITE_NOTE(A_do, ONE_FOURTH_HALF);                         // 1_
    WRITE_NOTE(A_la / 2, ONE_FOURTH);                          // 6.

    // 2·_ 3__ 2_ 5_ 2_ 3_ 2_ 1_
    WRITE_NOTE(A_re, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 2·_
    WRITE_NOTE(A_mi, ONE_FOURTH_HALF_HALF);                    // 3__
    WRITE_NOTE(A_re, ONE_FOURTH_HALF);                         // 2_
    WRITE_NOTE(A_so, ONE_FOURTH_HALF);                         // 5_
    WRITE_NOTE(A_re, ONE_FOURTH_HALF);                         // 2_
    WRITE_NOTE(A_mi, ONE_FOURTH_HALF);                         // 3_
    WRITE_NOTE(A_re, ONE_FOURTH_HALF);                         // 2_
    WRITE_NOTE(A_do, ONE_FOURTH_HALF);                         // 1_

    // 6.·_ 1__ 6._ 5._ 1_ 0_ 1_ 0_
    WRITE_NOTE(A_la / 2, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 6.·_
    WRITE_NOTE(A_do, ONE_FOURTH_HALF_HALF);                        // 1__
    WRITE_NOTE(A_la / 2, ONE_FOURTH_HALF);                         // 6._
    WRITE_NOTE(A_so, ONE_FOURTH_HALF);                             // 5._
    WRITE_NOTE(A_do, ONE_FOURTH_HALF);                             // 1_
    WRITE_NOTE(Z_no, ONE_FOURTH_HALF);                             // 0_
    WRITE_NOTE(A_do, ONE_FOURTH_HALF);                             // 1_
    WRITE_NOTE(Z_no, ONE_FOURTH_HALF);                             // 0_

    // 嘿咦耶
    // 2 3 5 3
    WRITE_NOTE(A_re, ONE_FOURTH);  // 2
    WRITE_NOTE(A_mi, ONE_FOURTH);  // 3
    WRITE_NOTE(A_so, ONE_FOURTH);  // 5
    WRITE_NOTE(A_mi, ONE_FOURTH);  // 3

    // 耶咦咦哈吼
    // 2·_ 3__ 2_ 1_ 6. -
    WRITE_NOTE(A_re, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 2·_
    WRITE_NOTE(A_mi, ONE_FOURTH_HALF_HALF);                    // 3__
    WRITE_NOTE(A_re, ONE_FOURTH_HALF);                         // 2_
    WRITE_NOTE(A_do, ONE_FOURTH_HALF);                         // 1_
    WRITE_NOTE(A_la / 2, ONE_FOURTH_HALF * 2);                 // 6.

    // 嘿咦耶
    // 2 3 5 3
    WRITE_NOTE(A_re, ONE_FOURTH);  // 2
    WRITE_NOTE(A_mi, ONE_FOURTH);  // 3
    WRITE_NOTE(A_so, ONE_FOURTH);  // 5
    WRITE_NOTE(A_mi, ONE_FOURTH);  // 3

    // 耶咦咦哈吼
    // 2·_ 3__ 2_ 6._ 1 -
    WRITE_NOTE(A_re, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 2·_
    WRITE_NOTE(A_mi, ONE_FOURTH_HALF_HALF);                    // 3__
    WRITE_NOTE(A_re, ONE_FOURTH_HALF);                         // 2_
    WRITE_NOTE(A_la / 2, ONE_FOURTH_HALF);                     // 6._
    WRITE_NOTE(A_do, ONE_FOURTH_HALF * 2);                     // 1

    // 6.·_ 1__ 3._ 5._ 6.·_ 1__ 3._ 5._
    WRITE_NOTE(A_la / 2, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 6.·_
    WRITE_NOTE(A_do, ONE_FOURTH_HALF_HALF);                        // 1__
    WRITE_NOTE(A_mi, ONE_FOURTH_HALF);                             // 3._
    WRITE_NOTE(A_so, ONE_FOURTH_HALF);                             // 5._
    WRITE_NOTE(A_la / 2, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 6.·_
    WRITE_NOTE(A_do, ONE_FOURTH_HALF_HALF);                        // 1__
    WRITE_NOTE(A_mi, ONE_FOURTH_HALF);                             // 3._
    WRITE_NOTE(A_so, ONE_FOURTH_HALF);                             // 5._

    // 6.·_ 1__ 3._ 5._ 6._ 5._ 6.
    WRITE_NOTE(A_la / 2, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 6.·_
    WRITE_NOTE(A_do, ONE_FOURTH_HALF_HALF);                        // 1__
    WRITE_NOTE(A_mi, ONE_FOURTH_HALF);                             // 3._
    WRITE_NOTE(A_so, ONE_FOURTH_HALF);                             // 5._
    WRITE_NOTE(A_la / 2, ONE_FOURTH_HALF);                         // 6._
    WRITE_NOTE(A_so, ONE_FOURTH_HALF);                             // 5._
    WRITE_NOTE(A_la / 2, ONE_FOURTH);                              // 6.

    // 6.·_ 1__ 3._ 5._ 6.·_ 1__ 3._ 5._
    WRITE_NOTE(A_la / 2, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 6.·_
    WRITE_NOTE(A_do, ONE_FOURTH_HALF_HALF);                        // 1__
    WRITE_NOTE(A_mi, ONE_FOURTH_HALF);                             // 3._
    WRITE_NOTE(A_so, ONE_FOURTH_HALF);                             // 5._
    WRITE_NOTE(A_la / 2, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 6.·_
    WRITE_NOTE(A_do, ONE_FOURTH_HALF_HALF);                        // 1__
    WRITE_NOTE(A_mi, ONE_FOURTH_HALF);                             // 3._
    WRITE_NOTE(A_so, ONE_FOURTH_HALF);                             // 6.

    // 6.·_ 1__ 3._ 5._ 6._ 5._ 6.
    WRITE_NOTE(A_la / 2, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 6.·_
    WRITE_NOTE(A_do, ONE_FOURTH_HALF_HALF);                        // 1__
    WRITE_NOTE(A_mi, ONE_FOURTH_HALF);                             // 3._
    WRITE_NOTE(A_so, ONE_FOURTH_HALF);                             // 5._
    WRITE_NOTE(A_la / 2, ONE_FOURTH_HALF);                         // 6._
    WRITE_NOTE(A_so, ONE_FOURTH_HALF);                             // 5._
    WRITE_NOTE(A_la / 2, ONE_FOURTH);                              // 6.

    // 恭喜你发财
    // 6 5 6 5_ 2_
    WRITE_NOTE(A_la, ONE_FOURTH);       // 6
    WRITE_NOTE(A_so, ONE_FOURTH);       // 5
    WRITE_NOTE(A_la, ONE_FOURTH);       // 6
    WRITE_NOTE(A_so, ONE_FOURTH_HALF);  // 5_
    WRITE_NOTE(A_re, ONE_FOURTH_HALF);  // 2_

    // 3 - - -
    WRITE_NOTE(A_mi, ONE_FOURTH * 4);  // 3

    // 恭喜你进财
    // 6 5 6 5_ 5_
    WRITE_NOTE(A_la, ONE_FOURTH);       // 6
    WRITE_NOTE(A_so, ONE_FOURTH);       // 5
    WRITE_NOTE(A_la, ONE_FOURTH);       // 6
    WRITE_NOTE(A_so, ONE_FOURTH_HALF);  // 5_
    WRITE_NOTE(A_so, ONE_FOURTH_HALF);  // 5_

    // 请
    // 6 - - 0_ 3_
    WRITE_NOTE(A_la, ONE_FOURTH * 3);   // 6
    WRITE_NOTE(Z_no, ONE_FOURTH_HALF);  // 0_
    WRITE_NOTE(A_mi, ONE_FOURTH_HALF);  // 3_

    // 好的经过来 不
    // 2·_ 3__ 2_ 1_ 6. 0_ 3_
    WRITE_NOTE(A_re, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 2·_
    WRITE_NOTE(A_mi, ONE_FOURTH_HALF_HALF);                    // 3__
    WRITE_NOTE(A_re, ONE_FOURTH_HALF);                         // 2_
    WRITE_NOTE(A_do, ONE_FOURTH_HALF);                         // 1_
    WRITE_NOTE(A_la / 2, ONE_FOURTH_HALF);                     // 6.
    WRITE_NOTE(Z_no, ONE_FOURTH_HALF);                         // 0_
    WRITE_NOTE(A_mi, ONE_FOURTH_HALF);                         // 3_

    // 好的请走开 而
    // 2·_ 3__ 2_ 1_ 2 0_ 2_
    WRITE_NOTE(A_re, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 2·_
    WRITE_NOTE(A_mi, ONE_FOURTH_HALF_HALF);                    // 3__
    WRITE_NOTE(A_re, ONE_FOURTH_HALF);                         // 2_
    WRITE_NOTE(A_do, ONE_FOURTH_HALF);                         // 1_
    WRITE_NOTE(A_re, ONE_FOURTH);                              // 2
    WRITE_NOTE(Z_no, ONE_FOURTH_HALF);                         // 0_
    WRITE_NOTE(A_re, ONE_FOURTH_HALF);                         // 2_

    // 礼多人不
    // 1 2 3 5
    WRITE_NOTE(A_do, ONE_FOURTH);  // 1
    WRITE_NOTE(A_re, ONE_FOURTH);  // 2
    WRITE_NOTE(A_mi, ONE_FOURTH);  // 3
    WRITE_NOTE(A_so, ONE_FOURTH);  // 5

    // 怪
    // 6 - - -
    WRITE_NOTE(A_la, ONE_FOURTH * 4);  // 6

    SLEEP_NOTE(1500);

    return MUSIC_INFO;
}
/*------------------------------ End of File ------------------------------*/
