#ifndef LOCKET_API_RAD_BEHAVIOR_H
#define LOCKET_API_RAD_BEHAVIOR_H

#include "pill_manager.h"
#include "rx_table.h"
#include "behavior.h"



class RadBehavior: public Behavior<IdOnlyState, IdOnlyState> {
public:
    using Behavior::Behavior;

    void OnStarted() override;
    void EverySecond() override;
    void OnButtonPressed(uint16_t button_index, bool long_press) override;
    void OnPillConnected(PillManager<IdOnlyState>* manager) override;
    void OnPillDisconnected() override;
    void OnRadioPacketReceived(const IdOnlyState& packet, int8_t rssi) override;
    void OnDipSwitchChanged(uint16_t dip_value_mask) override;

    //methods for logic
    void StartTransmitForPath();
    void StopTransmitForPath();
    void SetColor(Color color);
    void MonsterVibro();
    void MakePillUsed();
    void RadiationVibro();
    void Flash(Color color_new, Color color_old);
    void DeathVibro();
    void GodVibro();

private:
    PillManager<IdOnlyState>* pill_manager = nullptr;
    RxTable<IdOnlyState> rx_table;
    RadioPowerLevel range_level = RadioPowerLevel::MINUS_15_DBM;
    const uint32_t used_pill_id = 10;
    unsigned int s_counter = 0;
    unsigned int radio_counter = 0;
    unsigned int monster_s_counter = 0;
    unsigned int rad_s_counter = 0;

};

#endif //LOCKET_API_SM_BEHAVIOR_H
