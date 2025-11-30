#include "BatteryState.h"

void BatteryState::update_energy_remaining(double delta_energy) {
    energy_remaining += delta_energy;
}