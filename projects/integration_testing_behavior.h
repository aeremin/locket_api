#ifndef LOCKET_API_INTEGRATION_TESTING_BEHAVIOR_H
#define LOCKET_API_INTEGRATION_TESTING_BEHAVIOR_H

#include "pill_manager.h"
#include "rx_table.h"
#include "behavior.h"


class IntegrationTestingBehavior: public Behavior<IdOnlyState, IdOnlyState> {
public:
    enum class Mode {
        kAnnoying = 0,
        kNotAnnoying = 1,
    };

    using Behavior::Behavior;

    void OnStarted() override;
    void EverySecond() override;
    void OnPillConnected(PillManager<IdOnlyState>* manager) override;
    void OnPillDisconnected() override;
    void OnDipSwitchChanged(uint16_t dip_value_mask) override;
    void OnButtonPressed(uint16_t button_index) override;
    void OnRadioPacketReceived(const IdOnlyState& packet) override;

private:
    PillManager<IdOnlyState>* pill_manager = nullptr;
    uint32_t seconds_counter = 0;
    Mode mode = Mode::kAnnoying; // If true, beep and vibrate every bunch of seconds.
    RxTable<IdOnlyState> rx_table;
};

#endif //LOCKET_API_INTEGRATION_TESTING_BEHAVIOR_H