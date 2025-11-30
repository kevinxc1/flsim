#include "Motor.h"
#include <cmath>

double Motor::power_consumed(double angular_speed, double torque) const {
    // Calculate mechanical power: P = ω × τ
    double mechanical_power = angular_speed * torque;

    // In this simplified motor model, only hysteresis loss is added
    // Eddy current effects are already incorporated in the torque-speed relationship
    return mechanical_power + hysteresis_loss;
}