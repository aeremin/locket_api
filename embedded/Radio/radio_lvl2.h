#ifndef LOCKET_API_RADIO_LVL2_H
#define LOCKET_API_RADIO_LVL2_H

#include <limits>

#include "kl_lib.h"
#include "ch.h"
#include "cc1101.h"
#include "MsgQ.h"
#include "shell.h"

struct RadioPacketMetadata {
    uint8_t ID;
    uint8_t CycleN;
    uint8_t TimeSrcID;
};

template<typename TRadioPacket>
struct FullRadioPacket {
    FullRadioPacket() = default;
    explicit FullRadioPacket(const TRadioPacket& payload): payload(payload), metadata(), payload_present(true) {};

    TRadioPacket payload{};
    RadioPacketMetadata metadata{};
    bool payload_present = false;
};

#define RCHNL_EACH_OTH  7

// Change if CC settings changed, too. Do not touch until you know what you do.
#define HWTIM_CLK_FREQ_HZ       (27000000UL / 192UL) // 27MHz / 192, set in CC settings
#define HWTIM_TICK_DUR_us       (1000000UL / HWTIM_CLK_FREQ_HZ)

#define ID_MAX 50

// Feel-Each-Other related
#define RCYCLE_CNT              4
#define RSLOT_CNT               (ID_MAX+1) // Max count of lockets in system
#define RSLOT_DURATION_us       2200UL // Measured time is 1540us, but there must be reserve to deal with timer's jitter
#define RSLOT_DURATION_tics     (RSLOT_DURATION_us / HWTIM_TICK_DUR_us)
#define END_OF_PKT_SHIFT_tics   235U // Measured experimentally. This is time at transmitter's side when transmission is done. We have to set same time at ours side.

#define SCYCLES_TO_KEEP_TIMESRC 4   // After that amount of supercycles, TimeSrcID become self ID

template <typename TRadioPacket>
class RadioLevel2 {
public:
    uint8_t Init();

    void ProcessEvent();

    uint8_t SetBeaconPacketTo(const TRadioPacket& packet) {
        return incoming_message_queue_.SendNowOrExit({IncomingMessageType::SetBeacon, packet});
    }

    uint8_t ClearBeaconPacket() {
        return incoming_message_queue_.SendNowOrExit({IncomingMessageType::ClearBeacon, {}});
    }

    uint8_t TransmitOnce(const TRadioPacket& packet) {
        return incoming_message_queue_.SendNowOrExit({IncomingMessageType::TransmitOnce, packet});
    }

    std::pair<TRadioPacket, int8_t> FetchReceivedPacket() {
        return received_packets_.Fetch(TIME_IMMEDIATE);
    }

    void AddReceivedPacket(const TRadioPacket& packet, int8_t rssi) {
        incoming_message_queue_.SendNowOrExitI({IncomingMessageType::PacketReceived, {}});
        received_packets_.SendNowOrExitI({packet, rssi});
    }

    void ScheduleSleep() {
        if(state_ != State::Idle) {
            incoming_message_queue_.SendNowOrExitI({IncomingMessageType::TryToSleep, {}});
        }
    }

    void ScheduleTransmission() {
        incoming_message_queue_.SendNowOrExitI({IncomingMessageType::TryToTransmit, {}});
    }

    void ScheduleReception() {
        if(state_ != State::Rx) {
            incoming_message_queue_.SendNowOrExitI({IncomingMessageType::TryToReceive, {}});
        }
    }

    void SetPower(uint8_t power) {
        transmission_power_ = power;
    }

private:
    enum class IncomingMessageType: uint8_t {
        TryToReceive,
        TryToTransmit,
        TryToSleep,
        PacketReceived,
        SetBeacon,
        ClearBeacon,
        TransmitOnce,
    };
    struct IncomingMessage {
        IncomingMessageType type;
        TRadioPacket packet;
    };

    enum class State {Idle, Rx, Tx};

    volatile State state_ = State::Idle;
    std::optional<TRadioPacket> packet_to_transmit_once_ = std::nullopt;
    int8_t repeats_left = 0;
    std::optional<TRadioPacket> packet_to_beacon_ = std::nullopt;
    uint8_t transmission_power_ = 0; // 0 means "do not change"
    EvtMsgQ_t<std::pair<TRadioPacket, int8_t>, R_MSGQ_LEN> received_packets_{};
    EvtMsgQ_t<IncomingMessage, 2 * R_MSGQ_LEN> incoming_message_queue_{};
};

struct Config {
    uint8_t ID = 0;
};

Config g_config;

static Timer_t IHwTmr(TIM9);

class IRadioTime {
public:
    virtual void AdjustI(const RadioPacketMetadata& metadata) = 0;
    virtual void IOnTimerI() = 0;
    volatile uint8_t CycleN = 0, TimeSlot = 0;
    uint16_t TimeSrcId = std::numeric_limits<uint16_t>::max();
};

template<typename TRadioPacketType>
class RadioTime_t: public IRadioTime {
private:
    uint16_t TimeSrcTimeout = 0;
public:
    RadioLevel2<TRadioPacketType>* radio_;

    explicit RadioTime_t(RadioLevel2<TRadioPacketType>* radio): radio_(radio) {}

    void IncTimeSlot() {
        TimeSlot++;
        if(TimeSlot >= RSLOT_CNT) {  // Increment cycle
            TimeSlot = 0;
            CycleN++;
            if(CycleN >= RCYCLE_CNT) { // New supercycle begins
                CycleN = 0;
                radio_->ScheduleReception();
                // Check TimeSrc timeout
                if(TimeSrcTimeout >= SCYCLES_TO_KEEP_TIMESRC) TimeSrcId = g_config.ID;
                else TimeSrcTimeout++;
            }
        }
    }

    void AdjustI(const RadioPacketMetadata& metadata) override {
        if(metadata.TimeSrcID <= TimeSrcId) { // Adjust time if theirs TimeSrc <= OursTimeSrc
            CycleN = metadata.CycleN;
            TimeSlot = metadata.ID;
            TimeSrcId = metadata.TimeSrcID;
            TimeSrcTimeout = 0; // Reset Time Src Timeout
            IHwTmr.SetCounter(END_OF_PKT_SHIFT_tics);
        }
    }

    void IOnTimerI() override {
        IncTimeSlot();
        // Tx if now is our timeslot
        if(TimeSlot == g_config.ID) radio_->ScheduleTransmission();
            // Not our timeslot
        else {
            if(CycleN == 0) { // Enter RX if not yet
                radio_->ScheduleReception();
            }
            else { // CycleN != 0, enter sleep
                radio_->ScheduleSleep();
            }
        }
    }
};

IRadioTime* radio_time = nullptr;

template<typename TRadioPacket>
void RxCallback(void* user_data) {
    auto* radio_instance = static_cast<RadioLevel2<TRadioPacket>*>(user_data);
    FullRadioPacket<TRadioPacket> PktRx;
    int8_t rssi;
    if(CC.ReadFIFO(&PktRx, &rssi, sizeof(FullRadioPacket<TRadioPacket>)) == retvOk) {  // if pkt successfully received
        radio_time->AdjustI(PktRx.metadata);
        if (PktRx.payload_present) {
            radio_instance->AddReceivedPacket(PktRx.payload, rssi);
        }
    }
}

// RadioTime HW Timer IRQ. Calls OnTimer method.
extern "C"
void VectorA4() {
    // Printf("VectorA4\n");
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    TIM9->SR &= ~TIM_SR_UIF;
    radio_time->IOnTimerI();
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

template <typename TRadioPacket>
[[noreturn]]
static void rLvl2Thread(void* arg) {
    auto* radio_instance = static_cast<RadioLevel2<TRadioPacket>*>(arg);
    chRegSetThreadName("rLvl2");

    while(true) {
        radio_instance->ProcessEvent();
    }
}

template <typename TRadioPacket>
uint8_t RadioLevel2<TRadioPacket>::Init() {
    static THD_WORKING_AREA(warLvl2Thread, 256);

    incoming_message_queue_.Init();
    received_packets_.Init();

    if(CC.Init() == retvOk) {
        Printf("Init ok\n");
        CC.SetPktSize(sizeof(FullRadioPacket<TRadioPacket>));
        CC.DoIdleAfterTx();
        CC.SetChannel(RCHNL_EACH_OTH);
        CC.SetTxPower(CC_PwrMinus10dBm);
        CC.SetBitrate(CCBitrate500k);

        auto rt = new RadioTime_t<TRadioPacket>(this);
        rt->TimeSrcId = g_config.ID;
        radio_time = rt;

        // Setup HW timer: it generates IRQ at timeslot end
        // TIM9 Ch1. This is clock input. CC generates stable clock at GDO2: CLK_XOSC/192 (27MHz / 192 = 140625 Hz)
        PinSetupAlterFunc(GPIOA, 2, omPushPull, pudNone, AF3);
        IHwTmr.Init();
        IHwTmr.SetTopValue(RSLOT_DURATION_tics);
        IHwTmr.SelectSlaveMode(smExternal);
        IHwTmr.SetTriggerInput(tiTI1FP1);
        // Timer is clocked by external clock, provided by CC
        IHwTmr.SetupInput1(0b01, Timer_t::pscDiv1, rfRising);
        IHwTmr.EnableIrq(TIM9_IRQn, IRQ_PRIO_HIGH);
        IHwTmr.EnableIrqOnUpdate();

        // Thread
        chThdCreateStatic(warLvl2Thread, sizeof(warLvl2Thread), HIGHPRIO, rLvl2Thread<TRadioPacket>, this);
        chSysLock();
        IHwTmr.Enable(); // Start timer
        chSysUnlock();
        return retvOk;
    }
    else return retvFail;
}

template<typename TRadioPacket>
void RadioLevel2<TRadioPacket>::ProcessEvent() {
    auto message = incoming_message_queue_.Fetch(TIME_INFINITE);
    switch(message.type) {
        case IncomingMessageType::TryToTransmit: {
            CC.EnterIdle();
            state_ = State::Tx;
            FullRadioPacket<TRadioPacket> PktTx;
            if (packet_to_transmit_once_) {
                PktTx = FullRadioPacket<TRadioPacket>(packet_to_transmit_once_.value());
                if (--repeats_left == 0) {
                    packet_to_transmit_once_ = std::nullopt;
                }
            } else if (packet_to_beacon_) {
                PktTx = FullRadioPacket<TRadioPacket>(packet_to_beacon_.value());
            }
            PktTx.metadata.ID = g_config.ID;
            PktTx.metadata.CycleN = radio_time->CycleN;
            PktTx.metadata.TimeSrcID = radio_time->TimeSrcId;
            if(transmission_power_ != 0) {
                CC.SetTxPower(transmission_power_);
                transmission_power_ = 0;
            }
            CC.Recalibrate();
            CC.Transmit(&PktTx, sizeof(FullRadioPacket<TRadioPacket>));
        } break;

        case IncomingMessageType::TryToReceive:
            state_ = State::Rx;
            CC.ReceiveAsync(RxCallback<TRadioPacket>, this);
            break;

        case IncomingMessageType::TryToSleep:
            state_ = State::Idle;
            CC.EnterIdle();
            break;

        case IncomingMessageType::PacketReceived:
            state_ = State::Idle;
            EvtQMain.SendNowOrExit({evtIdRadioCmd});
            break;

        case IncomingMessageType::SetBeacon:
            packet_to_beacon_ =  message.packet;
            break;

        case IncomingMessageType::ClearBeacon:
            packet_to_beacon_ = std::nullopt;
            break;

        case IncomingMessageType::TransmitOnce:
            packet_to_transmit_once_ =  message.packet;
            repeats_left = RCYCLE_CNT + 1;
            break;
    } // switch

}

#endif //LOCKET_API_RADIO_LVL2_H
