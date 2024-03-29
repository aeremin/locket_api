/*$file${.::health.cpp} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/
/*
* Model: Health.qm
* File:  ${.::health.cpp}
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
/*$endhead${.::health.cpp} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
#include "qhsm.h"
#include "health.h"
#include "Glue.h"
#include "color.h"

//Q_DEFINE_THIS_FILE
/* global-scope definitions -----------------------------------------*/
QHsm * const the_health = (QHsm *) &health; /* the opaque pointer */

/*$skip${QP_VERSION} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/
/* Check for the minimum required QP version */
/*$endskip${QP_VERSION} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
/*$define${SMs::Health_ctor} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/
/*${SMs::Health_ctor} ......................................................*/
void Health_ctor(
    RadBehavior *SMBeh, Eeprom* eeprom, Logger* logger)
{
    Health *me = &health;
    me->eeprom = eeprom;
    me->logger = logger;
    me->SMBeh = SMBeh;
    unsigned int State = eeprom->Read<unsigned int>(offsetof(EepromMap, health_state));
    me->vars = Health_Variables::Load(eeprom);
    switch (State) {
        case SIMPLE: {
            me->StartState = (QStateHandler)&Health_simple;
            break;
        }
        case GOD_READY: {
            me->StartState = (QStateHandler)&Health_god_ready;
            break;
        }
        case GOD: {
            me->StartState = (QStateHandler)&Health_god;
            break;
        }
        case DEAD: {
            me->StartState = (QStateHandler)&Health_dead;
            break;
        }
        default:
            me->StartState = (QStateHandler)&Health_simple;
        }
    QHsm_ctor(&me->super, Q_STATE_CAST(&Health_initial));
}
/*$enddef${SMs::Health_ctor} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
/*$define${SMs::Health} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/
/*${SMs::Health} ...........................................................*/
/*${SMs::Health::SM} .......................................................*/
QState Health_initial(Health * const me, [[maybe_unused]] QEvt const * const e) {
    /*${SMs::Health::SM::initial} */
    return Q_TRAN(me->StartState);
}
/*${SMs::Health::SM::global} ...............................................*/
QState Health_global(Health * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {

#ifdef DESKTOP
        /*${SMs::Health::SM::global::TERMINATE} */
        case TERMINATE_SIG: {
            status_ = Q_TRAN(&Health_final);
            break;
        }
#endif /* def DESKTOP */

        default: {
            status_ = Q_SUPER(&QHsm_top);
            break;
        }
    }
    return status_;
}
/*${SMs::Health::SM::global::alive} ........................................*/
QState Health_alive(Health * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        /*${SMs::Health::SM::global::alive} */
        case Q_ENTRY_SIG: {
            me->logger->log("Entered state alive");
            me->SMBeh->StartTransmitForPath();
            me->SMBeh->SetColor(me->vars.GetHealthColor());
            status_ = Q_HANDLED();
            break;
        }
        /*${SMs::Health::SM::global::alive} */
        case Q_EXIT_SIG: {
            me->logger->log("Exited state alive");
            status_ = Q_HANDLED();
            break;
        }
        /*${SMs::Health::SM::global::alive::PILL_HEAL} */
        case PILL_HEAL_SIG: {
            me->vars.ResetHealth();
            me->SMBeh->SetColor(me->vars.GetHealthColor());
            me->SMBeh->MakePillUsed();
            status_ = Q_HANDLED();
            break;
        }
        /*${SMs::Health::SM::global::alive::MONSTER_SIGNAl} */
        case MONSTER_SIGNAl_SIG: {
            me->SMBeh->MonsterVibro();
            status_ = Q_HANDLED();
            break;
        }
        /*${SMs::Health::SM::global::alive::PILL_RESET} */
        case PILL_RESET_SIG: {
            me->vars.ResetHealth();
            me->SMBeh->SetColor(me->vars.GetHealthColor());
            status_ = Q_TRAN(&Health_simple);
            break;
        }
        default: {
            status_ = Q_SUPER(&Health_global);
            break;
        }
    }
    return status_;
}
/*${SMs::Health::SM::global::alive::god} ...................................*/
QState Health_god(Health * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        /*${SMs::Health::SM::global::alive::god} */
        case Q_ENTRY_SIG: {
            me->logger->log("Entered state god");
            me->SMBeh->SetColor(kWhite);
            me->SMBeh->GodVibro();
            me->vars.ResetCount();
            SaveHealthState(me->eeprom, GOD);
            status_ = Q_HANDLED();
            break;
        }
        /*${SMs::Health::SM::global::alive::god} */
        case Q_EXIT_SIG: {
            me->logger->log("Exited state god");
            me->vars.ReSetGodPause();
            me->SMBeh->GodVibro();
            status_ = Q_HANDLED();
            break;
        }
        /*${SMs::Health::SM::global::alive::god::TIME_TICK_1S} */
        case TIME_TICK_1S_SIG: {
            /*${SMs::Health::SM::global::alive::god::TIME_TICK_1S::[me->count>=GOD_THRESHOLD_S]} */
            if (me->vars.GetCount() >= GOD_THRESHOLD_S) {
                status_ = Q_TRAN(&Health_god_ready);
            }
            /*${SMs::Health::SM::global::alive::god::TIME_TICK_1S::[else]} */
            else {
                me->vars.IncrementCount();
                status_ = Q_HANDLED();
            }
            break;
        }
        default: {
            status_ = Q_SUPER(&Health_alive);
            break;
        }
    }
    return status_;
}
/*${SMs::Health::SM::global::alive::mortal} ................................*/
QState Health_mortal(Health * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        /*${SMs::Health::SM::global::alive::mortal} */
        case Q_ENTRY_SIG: {
            me->logger->log("Entered state mortal");
            status_ = Q_HANDLED();
            break;
        }
        /*${SMs::Health::SM::global::alive::mortal} */
        case Q_EXIT_SIG: {
            me->logger->log("Exited state mortal");
            status_ = Q_HANDLED();
            break;
        }
        /*${SMs::Health::SM::global::alive::mortal::RAD_RECEIVED} */
        case RAD_RECEIVED_SIG: {
            /*${SMs::Health::SM::global::alive::mortal::RAD_RECEIVED::[me->health<=e->damage]} */
            if (me->vars.GetHealth() <= DEFAULT_DAMAGE) {
                status_ = Q_TRAN(&Health_dead);
            }
            else {
                me->vars.DecreaseHealth(DEFAULT_DAMAGE);
                me->SMBeh->SetColor(me->vars.GetHealthColor());
                me->SMBeh->RadiationVibro();
                status_ = Q_UNHANDLED();
            }
            break;
        }
        /*${SMs::Health::SM::global::alive::mortal::DEAD_BUTTON_LONGPRESS} */
        case DEAD_BUTTON_LONGPRESS: {
            status_ = Q_TRAN(&Health_dead);
            break;
        }
        default: {
            status_ = Q_SUPER(&Health_alive);
            break;
        }
    }
    return status_;
}
/*${SMs::Health::SM::global::alive::mortal::god_ready} .....................*/
QState Health_god_ready(Health * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        /*${SMs::Health::SM::global::alive::mortal::god_ready} */
        case Q_ENTRY_SIG: {
            me->logger->log("Entered state god_ready");
            me->SMBeh->Flash(kWhite, me->vars.GetHealthColor());
            SaveHealthState(me->eeprom, GOD_READY);
            status_ = Q_HANDLED();
            break;
        }
        /*${SMs::Health::SM::global::alive::mortal::god_ready} */
        case Q_EXIT_SIG: {
            me->logger->log("Exited state god_ready");
            status_ = Q_HANDLED();
            break;
        }
        /*${SMs::Health::SM::global::alive::mortal::god_ready::TIME_TICK_1M} */
        case TIME_TICK_1M_SIG: {
            if (me->vars.GetGodPause() > 0) {
                me->vars.DecrementGodPause();
            }
            status_ = Q_HANDLED();
            break;
        }
        /*${SMs::Health::SM::global::alive::mortal::god_ready::GOD_BUTTON_LONGPRESS} */
        case GOD_BUTTON_LONGPRESS: {
            /*${SMs::Health::SM::global::alive::mortal::god_ready::MIDDLE_BUTTON_PR~::[me->god_pause==0]} */
            if (me->vars.GetGodPause() == 0) {
                status_ = Q_TRAN(&Health_god);
            }
            /*${SMs::Health::SM::global::alive::mortal::god_ready::MIDDLE_BUTTON_PR~::[else]} */
            else {
                me->SMBeh->Flash(kWhite, me->vars.GetHealthColor());
                status_ = Q_HANDLED();
            }
            break;
        }
        default: {
            status_ = Q_SUPER(&Health_mortal);
            break;
        }
    }
    return status_;
}
/*${SMs::Health::SM::global::alive::mortal::simple} ........................*/
QState Health_simple(Health * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        /*${SMs::Health::SM::global::alive::mortal::simple} */
        case Q_ENTRY_SIG: {
            me->logger->log("Entered state simple");
            SaveHealthState(me->eeprom, SIMPLE);
            status_ = Q_HANDLED();
            break;
        }
        /*${SMs::Health::SM::global::alive::mortal::simple} */
        case Q_EXIT_SIG: {
            me->logger->log("Exited state simple");
            status_ = Q_HANDLED();
            break;
        }
        /*${SMs::Health::SM::global::alive::mortal::simple::PILL_GOD} */
        case PILL_GOD_SIG: {
            me->vars.ZeroGodPause();
            status_ = Q_TRAN(&Health_god_ready);
            break;
        }
        default: {
            status_ = Q_SUPER(&Health_mortal);
            break;
        }
    }
    return status_;
}
/*${SMs::Health::SM::global::dead} .........................................*/
QState Health_dead(Health * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        /*${SMs::Health::SM::global::dead} */
        case Q_ENTRY_SIG: {
            me->logger->log("Entered state dead");
            me->SMBeh->SetColor(kRed);
            me->vars.ResetCount();
            SaveHealthState(me->eeprom, DEAD);
            me->SMBeh->StopTransmitForPath();
            me->SMBeh->DeathVibro();
            status_ = Q_HANDLED();
            break;
        }
        /*${SMs::Health::SM::global::dead} */
        case PILL_HEAL_SIG: {
            me->vars.ResetHealth();
            me->SMBeh->SetColor(me->vars.GetHealthColor());
            me->SMBeh->MakePillUsed();
            status_ = Q_TRAN(&Health_simple);
            break;
        }
        /*${SMs::Health::SM::global::dead} */
        case Q_EXIT_SIG: {
            me->logger->log("Exited state dead");
            status_ = Q_HANDLED();
            break;
        }
        /*${SMs::Health::SM::global::dead::PILL_RESET} */
        case PILL_RESET_SIG: {
            me->vars.ResetHealth();
            me->SMBeh->SetColor(me->vars.GetHealthColor());
            status_ = Q_TRAN(&Health_simple);
            break;
        }
        default: {
            status_ = Q_SUPER(&Health_global);
            break;
        }
    }
    return status_;
}

#ifdef DESKTOP
/*${SMs::Health::SM::final} ................................................*/
QState Health_final(Health * const me, QEvt const * const e) {
    QState status_;
    switch (e->sig) {
        /*${SMs::Health::SM::final} */
        case Q_ENTRY_SIG: {
            printf("Bye! Bye!");
            exit(0);
            status_ = Q_HANDLED();
            break;
        }
        default: {
            status_ = Q_SUPER(&QHsm_top);
            break;
        }
    }
    return status_;
}
#endif /* def DESKTOP */

/*$enddef${SMs::Health} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/


/*tranlated from diagrams code*/
