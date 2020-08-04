/*
 * Sequences.h
 *
 *  Created on: 09 ���. 2015 �.
 *      Author: Kreyl
 */

#pragma once

#include "ChunkTypes.h"

const LedRGBChunk_t lsqStart[] = {
        {csSetup, 0, kRed},
        {csWait, 207},
        {csSetup, 0, kGreen},
        {csWait, 207},
        {csSetup, 0, kBlue},
        {csWait, 207},
        {csSetup, 0, {0,4,0}},
        {csEnd},
};

const LedRGBChunk_t lsqFailure[] = {
        {csSetup, 0, kRed},
        {csWait, 99},
        {csSetup, 0, kBlack},
        {csWait, 99},
        {csSetup, 0, kRed},
        {csWait, 99},
        {csSetup, 0, kBlack},
        {csWait, 99},
        {csSetup, 0, kRed},
        {csWait, 99},
        {csSetup, 0, kBlack},
        {csEnd}
};

const LedRGBChunk_t lsqSearch[] = {
        {csSetup, 0, kRed},
        {csWait, 99},
        {csSetup, 0, kBlack},
        {csEnd},
};


#if 1 // ============================= Beeper ==================================
#define BEEP_VOLUME     2   // Maximum 10

#if 1 // ==== Notes ====
#define La_2    880

#define Do_3    1047
#define Do_D_3  1109
#define Re_3    1175
#define Re_D_3  1245
#define Mi_3    1319
#define Fa_3    1397
#define Fa_D_3  1480
#define Sol_3   1568
#define Sol_D_3 1661
#define La_3    1720
#define Si_B_3  1865
#define Si_3    1976

#define Do_4    2093
#define Do_D_4  2217
#define Re_4    2349
#define Re_D_4  2489
#define Mi_4    2637
#define Fa_4    2794
#define Fa_D_4  2960
#define Sol_4   3136
#define Sol_D_4 3332
#define La_4    3440
#define Si_B_4  3729
#define Si_4    3951

// Length
#define OneSixteenth    90
#define OneEighth       (OneSixteenth * 2)
#define OneFourth       (OneSixteenth * 4)
#define OneHalfth       (OneSixteenth * 8)
#define OneWhole        (OneSixteenth * 16)
#endif

// Type, BEEP_VOLUME, freq
const BeepChunk_t bsqOn[] = {
        {csSetup, 10, 7000},
        {csEnd}
};

const BeepChunk_t bsqButton[] = {
        {csSetup, 1, 1975},
        {csWait, 54},
        {csSetup, 0},
        {csEnd}
};
const BeepChunk_t bsqBeepBeep[] = {
        {csSetup, BEEP_VOLUME, 1975},
        {csWait, 54},
        {csSetup, 0},
        {csWait, 54},
        {csSetup, BEEP_VOLUME, 1975},
        {csWait, 54},
        {csSetup, 0},
        {csEnd}
};

const BeepChunk_t bsqSearch[] = {
        {csSetup, BEEP_VOLUME, 1975},
        {csWait, 99},
        {csSetup, 0},
        {csEnd}
};

#if 1 // ==== Extensions ====
// Pill
const BeepChunk_t bsqBeepPillOk[] = {
        {csSetup, BEEP_VOLUME, Si_3},
        {csWait, 180},
        {csSetup, BEEP_VOLUME, Re_D_4},
        {csWait, 180},
        {csSetup, BEEP_VOLUME, Fa_D_4},
        {csWait, 180},
        {csSetup, 0},
        {csEnd}
};

const BeepChunk_t bsqBeepPillBad[] = {
        {csSetup, BEEP_VOLUME, Fa_4},
        {csWait, 180},
        {csSetup, BEEP_VOLUME, Re_4},
        {csWait, 180},
        {csSetup, BEEP_VOLUME, Si_3},
        {csWait, 180},
        {csSetup, 0},
        {csEnd}
};
#endif // ext
#endif // beeper

#if 1 // ============================== Vibro ==================================
#define VIBRO_VOLUME    100  // 1 to 100

#define VIBRO_SHORT_MS          99
#define VIBRO_REPEAT_PERIOD     1008

const BaseChunk_t vsqBrr[] = {
        {csSetup, VIBRO_VOLUME},
        {csWait, VIBRO_SHORT_MS},
        {csSetup, 0},
        {csWait, VIBRO_REPEAT_PERIOD},
        {csEnd}
};

const BaseChunk_t vsqBrrBrr[] = {
        {csSetup, VIBRO_VOLUME},
        {csWait, VIBRO_SHORT_MS},
        {csSetup, 0},
        {csWait, 99},
        {csSetup, VIBRO_VOLUME},
        {csWait, VIBRO_SHORT_MS},
        {csSetup, 0},
        {csWait, VIBRO_REPEAT_PERIOD},
        {csEnd}
};

const BaseChunk_t vsqBrrBrrBrr[] = {
        {csSetup, VIBRO_VOLUME},
        {csWait, VIBRO_SHORT_MS},
        {csSetup, 0},
        {csWait, 99},
        {csSetup, VIBRO_VOLUME},
        {csWait, VIBRO_SHORT_MS},
        {csSetup, 0},
        {csWait, 99},
        {csSetup, VIBRO_VOLUME},
        {csWait, VIBRO_SHORT_MS},
        {csSetup, 0},
        {csWait, VIBRO_REPEAT_PERIOD},
        {csEnd}
};

// ==== Health ====
const BaseChunk_t vsqIll[] = {
        {csSetup, VIBRO_VOLUME},
        {csWait, 999},
        {csSetup, 0},
        {csWait, 3600},
        {csSetup, VIBRO_VOLUME},
        {csWait, VIBRO_SHORT_MS},
        {csGoto, 2}
};

const BaseChunk_t vsqDeath[] = {
        {csSetup, VIBRO_VOLUME},
        {csWait, 999},
        {csSetup, 0},
        {csWait, 4005},
        {csSetup, VIBRO_VOLUME},
        {csWait, VIBRO_SHORT_MS},
        {csGoto, 2}
};

// Cataclysm
const BaseChunk_t vsqCataclysm[] = {
        {csSetup, VIBRO_VOLUME},
        {csWait, 999},
        {csSetup, 0},
        {csWait, 999},
        {csSetup, VIBRO_VOLUME},
        {csWait, VIBRO_SHORT_MS},
        {csSetup, 0},
        {csWait, 99},
        {csSetup, VIBRO_VOLUME},
        {csWait, VIBRO_SHORT_MS},
        {csGoto, 2}
};

#endif
