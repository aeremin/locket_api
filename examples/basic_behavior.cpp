#include "basic_behavior.h"

void BasicBehavior::OnStarted() {
    std::cout << "Started execution!" << std::endl;
    led->StartOrRestart(RgbLedSequence());
}

void BasicBehavior::EverySecond() {
    std::cout << "Second passed" << std::endl;
}

void BasicBehavior::OnPillConnected(PillManager<IdOnlyState>* manager) {
    pill_manager = manager;
}

void BasicBehavior::OnPillDisconnected() {
    pill_manager = nullptr;
}
