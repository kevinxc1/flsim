#include "Motor.h"
#include <cmath>

double Motor::power_consumed(double angular_speed, double torque) const {
     
    double mechanical_power = angular_speed * torque;

     
     
    double eddy_current_loss = eddy_current_loss_coefficient * angular_speed;

    return mechanical_power + hysteresis_loss + eddy_current_loss;
}