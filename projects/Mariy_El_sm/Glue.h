#include "qhsm.h"
#ifndef GLUE_H_
#define GLUE_H_

const uint32_t path_id = 1;
const uint32_t monster_id = 2;
const uint32_t rad_id = 3;

enum PlayerSignals {
    TICK_SEC_SIG = Q_USER_SIG,

    TIME_TICK_1M_SIG,
    TIME_TICK_1S_SIG,
    PILL_HEAL_SIG,
    PILL_RESET_SIG,
    PILL_GOD_SIG,
    GOD_BUTTON_LONGPRESS,
    DEAD_BUTTON_LONGPRESS,
    RAD_RECEIVED_SIG,
    MONSTER_SIGNAl_SIG,
    LAST_USER_SIG [[maybe_unused]]
};

#endif /* GLUE_H_ */
