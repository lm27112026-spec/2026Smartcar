#include "music_hao_yun_lai.h"

#include "bsp_buzzer.h"
//#include "music.h"
#include "stm32f4xx_hal.h"

// http://www.jianpu.cn/pu/41/41935.htm

// clang-format off
#define D_do 294
#define D_re 330
#define D_mi 370
#define D_fa 392
#define D_so 440
#define D_la 494
#define D_si 554
// clang-format on

#define ONE_FOURTH 250
#define ONE_FOURTH_HALF 125
#define ONE_FOURTH_HALF_HALF 63

#define NOTE_NUM 140
static Note Notes[NOTE_NUM];  // Array of notes
static MusicInfo_s MUSIC_INFO;

/*-------------------- User functions --------------------*/

MusicInfo_s MusicHaoYunLaiInit(void)
{
    MUSIC_INFO.notes = Notes;

    // do re mi fa so la si
    // 1  2  3  4  5  6  7

    // 好运来祝你
    // 6 3`_ 2`_ 2` 1`_ 6_
    WRITE_NOTE(D_la, ONE_FOURTH);                        // 6
    WRITE_NOTE(D_mi * 2, ONE_FOURTH_HALF);               // 3`_
    WRITE_NOTE(D_re * 2, ONE_FOURTH_HALF + ONE_FOURTH);  // 2`_ 2`
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF);               // 1`_
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);                   // 6_

    // 好运来，
    // 5 1`_ 2`_ 6 -
    WRITE_NOTE(D_so, ONE_FOURTH);           // 5
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF);  // 1`_
    WRITE_NOTE(D_re * 2, ONE_FOURTH_HALF);  // 2`_
    WRITE_NOTE(D_la, ONE_FOURTH * 2);       // 6 -

    // 好运带来了
    // 6 2` 1` 6_ 5_
    WRITE_NOTE(D_la, ONE_FOURTH);       // 6
    WRITE_NOTE(D_re * 2, ONE_FOURTH);   // 2`
    WRITE_NOTE(D_do * 2, ONE_FOURTH);   // 1`
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);  // 6_
    WRITE_NOTE(D_so, ONE_FOURTH_HALF);  // 5_

    // 喜和爱,
    // 2 5_ 6_ 3 -
    WRITE_NOTE(D_re, ONE_FOURTH);       // 2
    WRITE_NOTE(D_so, ONE_FOURTH_HALF);  // 5_
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);  // 6_
    WRITE_NOTE(D_mi, ONE_FOURTH * 2);   // 3

    // 好运来我们
    // 3 6_ 5_ 6 6_ 5_
    WRITE_NOTE(D_mi, ONE_FOURTH);                    // 3
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);               // 6_
    WRITE_NOTE(D_so, ONE_FOURTH_HALF);               // 5_
    WRITE_NOTE(D_la, ONE_FOURTH + ONE_FOURTH_HALF);  // 6 6_
    WRITE_NOTE(D_so, ONE_FOURTH_HALF);               // 5_

    // 好运来，
    // 6 2`_ 1`_ 2` -
    WRITE_NOTE(D_la, ONE_FOURTH);           // 6
    WRITE_NOTE(D_re * 2, ONE_FOURTH_HALF);  // 2`_
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF);  // 1`_
    WRITE_NOTE(D_re * 2, ONE_FOURTH * 2);   // 2`

    //迎着好运兴旺发达
    // 1`-_ 1`__ 1`_ 2`_ 3`_ 3`_ 2`_ 1`_
    WRITE_NOTE(
        D_do * 2, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF + ONE_FOURTH_HALF_HALF +
                      ONE_FOURTH_HALF);                       // 1`-_ 1`__ 1`_
    WRITE_NOTE(D_re * 2, ONE_FOURTH_HALF);                    // 2`_
    WRITE_NOTE(D_mi * 2, ONE_FOURTH_HALF + ONE_FOURTH_HALF);  // 3`_ 3`_
    WRITE_NOTE(D_re * 2, ONE_FOURTH_HALF);                    // 2`_
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF);                    // 1`_

    // 通四海。
    // 5 1`_ 6_ 6 -
    WRITE_NOTE(D_so, ONE_FOURTH);                        // 5
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF);               // 1`_
    WRITE_NOTE(D_la, ONE_FOURTH_HALF + ONE_FOURTH * 2);  // 6_ 6

    // 尾奏
    // 6 - - -
    WRITE_NOTE(D_la, ONE_FOURTH * 4);  // 6

    //6__ 5__ 3_ 2_ 2_ 2__ 1__ 2__ 3__ 5_ 5_
    WRITE_NOTE(D_la, ONE_FOURTH_HALF_HALF);                                      // 6__
    WRITE_NOTE(D_so, ONE_FOURTH_HALF_HALF);                                      // 5__
    WRITE_NOTE(D_mi, ONE_FOURTH_HALF);                                           // 3_
    WRITE_NOTE(D_re, ONE_FOURTH_HALF + ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 2_ 2_ 2__
    WRITE_NOTE(D_do, ONE_FOURTH_HALF_HALF);                                      // 1__
    WRITE_NOTE(D_re, ONE_FOURTH_HALF_HALF);                                      // 2__
    WRITE_NOTE(D_mi, ONE_FOURTH_HALF_HALF);                                      // 3__
    WRITE_NOTE(D_so, ONE_FOURTH_HALF + ONE_FOURTH_HALF);                         // 5_ 5_

    // 5__ 2__ 3__ 5__ 6_ 6_ 6__ 3__ 5__ 6__ 1`_ 1`_
    WRITE_NOTE(D_so, ONE_FOURTH_HALF_HALF);                                      // 5__
    WRITE_NOTE(D_re, ONE_FOURTH_HALF_HALF);                                      // 2__
    WRITE_NOTE(D_mi, ONE_FOURTH_HALF);                                           // 3__
    WRITE_NOTE(D_so, ONE_FOURTH_HALF);                                           // 5__
    WRITE_NOTE(D_la, ONE_FOURTH_HALF + ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 6_ 6_ 6__
    WRITE_NOTE(D_mi, ONE_FOURTH_HALF);                                           // 3__
    WRITE_NOTE(D_so, ONE_FOURTH_HALF);                                           // 5__
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);                                           // 6__
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF + ONE_FOURTH_HALF);                     // 1`_ 1`_

    // 1`__ 6__ 1`__ 2`__ 3`_ 3`_ 0 3`_ 3`_
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF_HALF);               // 1`__
    WRITE_NOTE(D_la, ONE_FOURTH_HALF_HALF);                   // 6__
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF_HALF);               // 1`__
    WRITE_NOTE(D_re * 2, ONE_FOURTH_HALF_HALF);               // 2`__
    WRITE_NOTE(D_mi * 2, ONE_FOURTH_HALF + ONE_FOURTH_HALF);  // 3`_ 3`_
    WRITE_NOTE(0, ONE_FOURTH_HALF);                           // 0
    WRITE_NOTE(D_mi * 2, ONE_FOURTH_HALF + ONE_FOURTH_HALF);  // 3`_ 3`_

    // 停顿一下
    WRITE_NOTE(0, ONE_FOURTH_HALF_HALF);

    // 叠个千纸鹤 再
    // 6-_ 6__ 1`_ 1`_ 6- 6_
    WRITE_NOTE(D_la, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF + ONE_FOURTH_HALF_HALF);  // 6-_ 6__
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF + ONE_FOURTH_HALF);                          // 1`_ 1`_
    WRITE_NOTE(D_la, ONE_FOURTH + ONE_FOURTH_HALF + ONE_FOURTH_HALF);                 // 6- 6_

    // 系个红飘带，
    // 5_ 3_ 5_ 1`_ 6 -
    WRITE_NOTE(D_so, ONE_FOURTH_HALF);      // 5_
    WRITE_NOTE(D_mi, ONE_FOURTH_HALF);      // 3_
    WRITE_NOTE(D_so, ONE_FOURTH_HALF);      // 5_
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF);  // 1`_
    WRITE_NOTE(D_la, ONE_FOURTH * 2);       // 6 -

    // 愿善良的人们
    // 6_ 1`_ 1`-_ 1`__ 1`_ 6_ 5
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);  // 6_
    WRITE_NOTE(
        D_do * 2, ONE_FOURTH_HALF + ONE_FOURTH + ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF +
                      ONE_FOURTH_HALF);  // 1`_ 1`-_ 1`__ 1`_
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);   // 6_
    WRITE_NOTE(D_so, ONE_FOURTH);        // 5

    // 天天好运来，你
    // 6_ 5_ 2_ 5_ 3- 3_
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);                                 // 6_
    WRITE_NOTE(D_so, ONE_FOURTH_HALF);                                 // 5_
    WRITE_NOTE(D_re, ONE_FOURTH_HALF);                                 // 2_
    WRITE_NOTE(D_so, ONE_FOURTH_HALF);                                 // 5_
    WRITE_NOTE(D_mi, ONE_FOURTH + ONE_FOURTH_HALF + ONE_FOURTH_HALF);  // 3- 3_

    // 勤劳生活美 你
    // 3_ 2_ 1_ 3_ 2- 3_
    WRITE_NOTE(D_mi, ONE_FOURTH_HALF);               // 3_
    WRITE_NOTE(D_re, ONE_FOURTH_HALF);               // 2_
    WRITE_NOTE(D_do, ONE_FOURTH_HALF);               // 1_
    WRITE_NOTE(D_mi, ONE_FOURTH_HALF);               // 3_
    WRITE_NOTE(D_re, ONE_FOURTH + ONE_FOURTH_HALF);  // 2-
    WRITE_NOTE(D_mi, ONE_FOURTH_HALF);               // 3_

    // 健康春常在，
    // 6_ 5_ 3_ 6_ 5 -
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);  // 6_
    WRITE_NOTE(D_so, ONE_FOURTH_HALF);  // 5_
    WRITE_NOTE(D_mi, ONE_FOURTH_HALF);  // 3_
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);  // 6_
    WRITE_NOTE(D_so, ONE_FOURTH * 2);   // 5 -

    // 你一生的忙碌为了
    // 6_ 1`_ 1`-_ 6__ 2`_ 2`_ 2`_ 1`_
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);                                               // 6_
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF + ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 1`_ 1`-_
    WRITE_NOTE(D_la, ONE_FOURTH_HALF_HALF);                                          // 6__
    WRITE_NOTE(D_re * 2, ONE_FOURTH_HALF + ONE_FOURTH_HALF + ONE_FOURTH_HALF);       // 2`_ 2`_ 2`_
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF);                                           // 1`_

    // 笑逐颜
    // 2/4 6 5_ 1`_
    WRITE_NOTE(D_la, ONE_FOURTH * 2);  // 6
    WRITE_NOTE(D_so, ONE_FOURTH);      // 5_
    WRITE_NOTE(D_do * 2, ONE_FOURTH);  // 1`_

    // 开
    // 6 - - -
    WRITE_NOTE(D_la, ONE_FOURTH * 4);  // 6

    // 停顿一下
    WRITE_NOTE(0, ONE_FOURTH_HALF_HALF);

    // 打个中国结 请
    // 6-_ 6__ 1`_ 1`_ 6- 6_
    WRITE_NOTE(D_la, ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF + ONE_FOURTH_HALF_HALF);  // 6-_ 6__
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF + ONE_FOURTH_HALF);                          // 1`_ 1`_
    WRITE_NOTE(D_la, ONE_FOURTH + ONE_FOURTH_HALF + ONE_FOURTH_HALF);                 // 6- 6_

    // 春风剪个彩，
    // 5_ 3_ 5_ 1`_ 6 -
    WRITE_NOTE(D_so, ONE_FOURTH_HALF);      // 5_
    WRITE_NOTE(D_mi, ONE_FOURTH_HALF);      // 3_
    WRITE_NOTE(D_so, ONE_FOURTH_HALF);      // 5_
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF);  // 1`_
    WRITE_NOTE(D_la, ONE_FOURTH * 2);       // 6 -

    // 愿祖国的日月
    // 6_ 1`_ 1`-_ 1`__ 1`_ 6_ 5
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);  // 6_
    WRITE_NOTE(
        D_do * 2, ONE_FOURTH_HALF + ONE_FOURTH + ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF +
                      ONE_FOURTH_HALF);  // 1`_ 1`-_ 1`__ 1`_
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);   // 6_
    WRITE_NOTE(D_so, ONE_FOURTH);        // 5

    // 年年好运来，你
    // 6_ 5_ 2_ 5_ 3- 3_
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);                                 // 6_
    WRITE_NOTE(D_so, ONE_FOURTH_HALF);                                 // 5_
    WRITE_NOTE(D_re, ONE_FOURTH_HALF);                                 // 2_
    WRITE_NOTE(D_so, ONE_FOURTH_HALF);                                 // 5_
    WRITE_NOTE(D_mi, ONE_FOURTH + ONE_FOURTH_HALF + ONE_FOURTH_HALF);  // 3- 3_

    // 凤舞太平年 你
    // 3_ 2_ 1_ 3_ 2- 3_
    WRITE_NOTE(D_mi, ONE_FOURTH_HALF);               // 3_
    WRITE_NOTE(D_re, ONE_FOURTH_HALF);               // 2_
    WRITE_NOTE(D_do, ONE_FOURTH_HALF);               // 1_
    WRITE_NOTE(D_mi, ONE_FOURTH_HALF);               // 3_
    WRITE_NOTE(D_re, ONE_FOURTH + ONE_FOURTH_HALF);  // 2-
    WRITE_NOTE(D_mi, ONE_FOURTH_HALF);               // 3_

    // 龙腾新时代，
    // 6_ 5_ 3_ 6_ 5 -
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);  // 6_
    WRITE_NOTE(D_so, ONE_FOURTH_HALF);  // 5_
    WRITE_NOTE(D_mi, ONE_FOURTH_HALF);  // 3_
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);  // 6_
    WRITE_NOTE(D_so, ONE_FOURTH * 2);   // 5 -

    // 幸福的家园迎来
    // 6_ 1`_ 1`-_ 6__ 2`_ 2`_ 2`_ 1`_
    WRITE_NOTE(D_la, ONE_FOURTH_HALF);                                               // 6_
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF + ONE_FOURTH_HALF + ONE_FOURTH_HALF_HALF);  // 1`_ 1`-_
    WRITE_NOTE(D_la, ONE_FOURTH_HALF_HALF);                                          // 6__
    WRITE_NOTE(D_re * 2, ONE_FOURTH_HALF + ONE_FOURTH_HALF + ONE_FOURTH_HALF);       // 2`_ 2`_ 2`_
    WRITE_NOTE(D_do * 2, ONE_FOURTH_HALF);                                           // 1`_

    // 百花盛
    // 2/4 6 5_ 1`_
    WRITE_NOTE(D_la, ONE_FOURTH * 2);  // 6
    WRITE_NOTE(D_so, ONE_FOURTH);      // 5_
    WRITE_NOTE(D_do * 2, ONE_FOURTH);  // 1`_

    // 开
    // 6 - - -
    WRITE_NOTE(D_la, ONE_FOURTH * 4);  // 6

    SLEEP_NOTE(1000);

    return MUSIC_INFO;
}
/*------------------------------ End of File ------------------------------*/
