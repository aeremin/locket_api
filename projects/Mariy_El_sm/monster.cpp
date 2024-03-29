//
// Created by juice on 10.07.2021.
//
#include "sequences.h"
#include "monster.h"
#include "utility.h"
#include "Glue.h"

const LedRGBChunk StartOnceLedSequence[] = {
        {{ChunkType::kSetup, {0}}, kLightBlue},
        {{ChunkType::kWait, {1000}}},
        {{ChunkType::kGoto, {2}}}
};

void MonsterBehavior::OnStarted() {
    // мы считаем, что здесь вызвался OnDipSwitchChanged
    led->StartOrRestart(StartOnceLedSequence);
    vibro->StartOrRestart(kBrrBrrBrr);
    radio->SetBeaconPacket({monster_id});
}

void MonsterBehavior::OnDipSwitchChanged(uint16_t dip_value_mask) {
    uint8_t first_range = GetSwitchState(dip_value_mask, 5);
    uint8_t second_range = GetSwitchState(dip_value_mask, 6);
    uint8_t third_range = GetSwitchState(dip_value_mask, 7);
    uint8_t fourth_range = GetSwitchState(dip_value_mask, 8);
    uint8_t range = 8 * first_range + 4 * second_range + 2 * third_range + fourth_range;
    if (range > 11) {
        range = 11;
    }
    range_level = IdToRadioEnum(range);
    radio->SetPowerLevel(range_level);
    logger->log("DIP switch changed to %d%d%d%d%d%d%d%d",
                GetSwitchState(dip_value_mask, 1),
                GetSwitchState(dip_value_mask, 2),
                GetSwitchState(dip_value_mask, 3),
                GetSwitchState(dip_value_mask, 4),
                GetSwitchState(dip_value_mask, 5),
                GetSwitchState(dip_value_mask, 6),
                GetSwitchState(dip_value_mask, 7),
                GetSwitchState(dip_value_mask, 8));
}