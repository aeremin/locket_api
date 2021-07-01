/*$file${.::ability.h} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/
/*
* Model: Ability.qm
* File:  ${.::ability.h}
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
/*$endhead${.::ability.h} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
#ifndef ability_h
#define ability_h
#include "api/eeprom.h"
#include "mariel_sm.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "qhsm.h"    /* include own framework tagunil version */

//Start of h code from diagram
#define ABILITY_THRESHOLD_S 30
#define ABILITY_PAUSE_M 15
const unsigned int SAVE_PAUSE = 60;

class Ability_Variables {
public:
    static Ability_Variables Load(Eeprom* eeprom) {
        Ability_Variables result{};
        result.eeprom = eeprom;
        result.ability_pause = eeprom->Read<unsigned int>(offsetof(Ability_Variables, ability_pause));
        result.count = eeprom->Read<unsigned int>(offsetof(Ability_Variables, count));
        result.ability = eeprom->Read<unsigned int>(offsetof(Ability_Variables, ability));
        return result;
    }

    void DecrementAbilityPause() {
        --ability_pause;
        SaveAbilityPause();
    }

    unsigned int GetAbilityPause() const {
        return ability_pause;
    }

    void SetAbility(unsigned int a) {
        ability = a;
        SaveAbility();
    }

    void ResetAbilityPause() {
        ability_pause = ABILITY_PAUSE_M;
        SaveAbilityPause();
    }

    void ResetCount() {
        count = 0;
        SaveAbilityPause();
    }

    void IncrementCount() {
        ++count;
        // Count changes every second, only persist it occasionally to prevent
        // too frequent eeprom writes.
        if (count % SAVE_PAUSE == 0) {
            SaveCount();
        }
    }

    unsigned int GetCount() const {
        return count;
    }

private:
    void SaveAbilityPause() {
        eeprom->Write(ability_pause, offsetof(Ability_Variables, ability_pause));
    }
    void SaveCount() {
        eeprom->Write(count, offsetof(Ability_Variables, count));
    }
    void SaveAbility() {
        eeprom->Write(ability, offsetof(Ability_Variables, ability));
    }

    unsigned int ability_pause;
    unsigned int count;
    unsigned int ability;
    Eeprom* eeprom;
};
//End of h code from diagram


/*$declare${SMs::Ability} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/
/*${SMs::Ability} ..........................................................*/
typedef struct {
/* protected: */
    QHsm super;
    RadBehavior* SMBeh;

    /* public: */
    Ability_Variables vars;
} Ability;

/* protected: */
QState Ability_initial(Ability * const me, QEvt const * const e);
QState Ability_global(Ability * const me, QEvt const * const e);
QState Ability_ability(Ability * const me, QEvt const * const e);
QState Ability_idle(Ability * const me, QEvt const * const e);
QState Ability_active(Ability * const me, QEvt const * const e);

#ifdef DESKTOP
QState Ability_final(Ability * const me, QEvt const * const e);
#endif /* def DESKTOP */

/*$enddecl${SMs::Ability} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/

static Ability ability; /* the only instance of the Ability class */



typedef struct abilityQEvt {
    QEvt super;
    unsigned int value;
} abilityQEvt;


extern QHsm * const the_ability; /* opaque pointer to the ability HSM */

/*$declare${SMs::Ability_ctor} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/
/*${SMs::Ability_ctor} .....................................................*/
void Ability_ctor(RadBehavior* SMBeh, Eeprom* eeprom);
/*$enddecl${SMs::Ability_ctor} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
#ifdef __cplusplus
}
#endif
#endif /* ability_h */