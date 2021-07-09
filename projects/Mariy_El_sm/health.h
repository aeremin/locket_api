/*$file${.::health.h} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/
/*
* Model: Health.qm
* File:  ${.::health.h}
*
* This code has been generated by QM 4.5.1 (https://www.state-machine.com/qm).
* DO NOT EDIT THIS FILE MANUALLY. All your changes will be lost.
*
* This program is open source software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published
* by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
*/
/*$endhead${.::health.h} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
#ifndef health_h
#define health_h

#include "mariel_sm.h"
#include "api/eeprom.h"
#include "sm_data.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "qhsm.h"    /* include own framework tagunil version */


//Start of h code from diagram
const uint32_t SIMPLE = 0;
const uint32_t GOD_READY = 1;
const uint32_t GOD = 2;
const uint32_t DEAD = 3;
const uint32_t GOD_THRESHOLD_S=30;
const uint32_t PILL_RESET = 0;
const uint32_t PILL_HEAL = 1;
const uint32_t PILL_GOD = 2;
const uint32_t DEFAULT_DAMAGE = 1;


//End of h code from diagram


/*$declare${SMs::Health} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/
/*${SMs::Health} ...........................................................*/

typedef struct {
/* protected: */
    QHsm super;
    RadBehavior* SMBeh;

/* public: */
    QStateHandler StartState;
    Health_Variables vars;
    Eeprom* eeprom;
    Logger* logger;
} Health;

/* protected: */
QState Health_initial(Health * const me, QEvt const * const e);
QState Health_global(Health * const me, QEvt const * const e);
QState Health_alive(Health * const me, QEvt const * const e);
QState Health_god(Health * const me, QEvt const * const e);
QState Health_mortal(Health * const me, QEvt const * const e);
QState Health_god_ready(Health * const me, QEvt const * const e);
QState Health_simple(Health * const me, QEvt const * const e);
QState Health_dead(Health * const me, QEvt const * const e);

#ifdef DESKTOP
QState Health_final(Health * const me, QEvt const * const e);
#endif /* def DESKTOP */

/*$enddecl${SMs::Health} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/

static Health health; /* the only instance of the Health class */



typedef struct healthQEvt {
    QEvt super;
} healthQEvt;


extern QHsm * const the_health; /* opaque pointer to the health HSM */

/*$declare${SMs::Health_ctor} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/
/*${SMs::Health_ctor} ......................................................*/
void Health_ctor(RadBehavior *SMBeh, unsigned int State, Eeprom* eeprom, Logger* logger);
/*$enddecl${SMs::Health_ctor} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
#ifdef __cplusplus
}
#endif
#endif /* health_h */