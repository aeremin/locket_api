//
// Created by juice on 15.09.2021.
//

#ifndef LOCKET_API_7STRIDES_BEHAVIOUR_H
#define LOCKET_API_7STRIDES_BEHAVIOUR_H

#include "rx_table.h"
#include "behavior.h"


uint8_t const LOCKET_RX = 1;
uint8_t const LOCKET_TX = 2;
uint8_t const LOCKET_RXTX = 3;


class SevenStridesBehavior: public Behavior<IdOnlyState, IdOnlyState> {
public:
    using Behavior::Behavior;

    void OnStarted() override;
    void EverySecond() override;
    void OnRadioPacketReceived(const IdOnlyState& packet, int8_t rssi) override;
    void OnButtonPressed(uint16_t button_index, bool long_press) override;
    void OnDipSwitchChanged(uint16_t dip_value_mask) override;
    void OnUartCommand(UartCommand& command) override;

private:
    uint32_t rx_seconds_counter = 0;
    uint32_t tx_seconds_counter = 0;
    int8_t rssi_level = -120;
    RxTable<IdOnlyState> rx_table;
    uint32_t LocketType = LOCKET_RX;
    RadioPowerLevel RangeLevel = RadioPowerLevel::MINUS_6_DBM;
    bool working = false;
};

RadioPowerLevel TypeToEnum(uint8_t level);

#endif //LOCKET_API_7STRIDES_BEHAVIOUR_H
