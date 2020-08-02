#include <utility.h>
#include "basic_behavior.h"

void BasicBehavior::OnStarted() {
    std::cout << "Started execution!" << std::endl;
    led->StartOrRestart(RgbLedSequence());
}

void BasicBehavior::EverySecond() {
    ++seconds_counter;
    if (seconds_counter % 30 == 0) {
        std::cout << "30 seconds passed" << std::endl;
    }
}

void BasicBehavior::OnPillConnected(PillManager<IdOnlyState>* manager) {
    pill_manager = manager;
    std::cout << "Read value " << pill_manager->ReadPill().id << " from pill" << std::endl;
}

void BasicBehavior::OnPillDisconnected() {
    std::cout << "Pill was disconnected" << std::endl;
    pill_manager = nullptr;
}

void BasicBehavior::OnDipSwitchChanged(uint16_t dip_value_mask) {
    std::cout << "DIP switch changed to "
              << GetSwitchState(dip_value_mask, 1)
              << GetSwitchState(dip_value_mask, 2)
              << GetSwitchState(dip_value_mask, 3)
              << GetSwitchState(dip_value_mask, 4)
              << GetSwitchState(dip_value_mask, 5)
              << GetSwitchState(dip_value_mask, 6)
              << GetSwitchState(dip_value_mask, 7)
              << GetSwitchState(dip_value_mask, 8)
              << std::endl;
}
