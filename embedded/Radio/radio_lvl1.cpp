/*
 * radio_lvl1.cpp
 *
 *  Created on: Nov 17, 2013
 *
 *      Author: kreyl
 */

#include "radio_lvl1.h"
#include "cc1101.h"
#include "uart.h"

#include "led.h"
#include "beeper.h"
#include "sequences_collection.h"

extern LedRGBwPower_t Led;
extern Beeper_t Beeper;

cc1101_t CC(CC_Setup0);

//#define DBG_PINS

#ifdef DBG_PINS
#define DBG_GPIO1   GPIOB
#define DBG_PIN1    10
#define DBG1_SET()  PinSetHi(DBG_GPIO1, DBG_PIN1)
#define DBG1_CLR()  PinSetLo(DBG_GPIO1, DBG_PIN1)
#define DBG_GPIO2   GPIOB
#define DBG_PIN2    9
#define DBG2_SET()  PinSetHi(DBG_GPIO2, DBG_PIN2)
#define DBG2_CLR()  PinSetLo(DBG_GPIO2, DBG_PIN2)
#else
#define DBG1_SET()
#define DBG1_CLR()
#endif

rLevel1_t g_radio_singleton;
extern int32_t ID;
void ProcessRCmd(const rPkt_t& packet);

static int8_t Rssi;

#if 1 // ================================ Task =================================
static THD_WORKING_AREA(warLvl1Thread, 256);
[[noreturn]]
static void rLvl1Thread(void *arg) {
    chRegSetThreadName("rLvl1");
    while(true) {
        // ==== TX if needed ====
        RMsg_t Msg = g_radio_singleton.RMsgQ.Fetch(TIME_IMMEDIATE);
        if(Msg.Cmd == R_MSG_TRANSMIT) {
            CC.SetTxPower(CC_PwrMinus20dBm);
            for(int i=0; i<4; i++) {
                CC.Recalibrate();
                CC.Transmit(g_radio_singleton.PktTx, g_radio_singleton.PktTxSize);
                chThdSleepMilliseconds(99);
            }
        }
        // ==== Rx ====
        CC.Recalibrate();

        rPkt_t PktRx;
        if(CC.Receive(360, &PktRx, RPKT_LEN, &Rssi) == retvOk) {
            Printf("From: %u; To: %u; TrrID: %u; PktID: %u; Cmd: %u; Rssi: %d\r\n", PktRx.From, PktRx.To, PktRx.TransmitterID, PktRx.PktID, PktRx.Cmd, Rssi);
//            Led.StartOrRestart(lsqBlinkB);
            ProcessRCmd(PktRx);
        }
    } // while true
}

void ProcessRCmd(const rPkt_t& PktRx) {
    rPkt_t PktTx;
    if(PktRx.To == ID) { // For us
        // Common preparations
        PktTx.From = ID;
        PktTx.To = PktRx.From;
        PktTx.TransmitterID = ID;
        PktTx.Cmd = rcmdPong;
        if(PktRx.PktID != PKTID_DO_NOT_RETRANSMIT) { // Increase PktID if not PKTID_DO_NOT_RETRANSMIT
            if(PktRx.PktID >= PKTID_TOP_VALUE) PktTx.PktID = 1;
            else PktTx.PktID = PktRx.PktID + 1;
        }
        else PktTx.PktID = PKTID_DO_NOT_RETRANSMIT;

        // Transmit pkt
        if(PktRx.PktID != PKTID_DO_NOT_RETRANSMIT) chThdSleepMilliseconds(999); // Let network to calm down
        CC.SetTxPower(CC_PwrPlus5dBm);
        systime_t Start = chVTGetSystemTimeX();
        while(true) {
            int32_t TxTime_ms = (int32_t)324 - (int32_t)(TIME_I2MS(chVTTimeElapsedSinceX(Start)));
            if(TxTime_ms <= 0) break;
            CC.Recalibrate();
            CC.Transmit(&PktTx, RPKT_LEN);
            //chThdSleepMilliseconds(6);
        }
    }
    else { // for everyone!
        switch(PktRx.Cmd) {
            case rcmdLocketDieAll:
                EvtQMain.SendNowOrExit(EvtMsg_t(evtIdShinePktMutant));
                break;

            default: break;
        } // switch
    } // else
}
#endif // task

void rLevel1_t::TryToSleep(uint32_t SleepDuration) {
    if(SleepDuration >= MIN_SLEEP_DURATION_MS) CC.EnterPwrDown();
    chThdSleepMilliseconds(SleepDuration);
}

#if 1 // ============================
uint8_t rLevel1_t::Init() {
    RMsgQ.Init();
    if(CC.Init() == retvOk) {
        CC.SetTxPower(CC_PwrMinus20dBm);
        CC.SetPktSize(RPKT_LEN);
        CC.SetChannel(1);
//        CC.EnterPwrDown();
        // Thread
        chThdCreateStatic(warLvl1Thread, sizeof(warLvl1Thread), HIGHPRIO, (tfunc_t)rLvl1Thread, NULL);
        return retvOk;
    }
    else return retvFail;
}


#endif
