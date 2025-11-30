#include "Battery.h"
#include <cmath>
#include <algorithm>

double Battery::state_of_charge(double energy_remaining) const {
    return energy_remaining / energy_capacity;
}

double Battery::current_voltage(double state_of_charge) const {
     
    return min_voltage + state_of_charge * (max_voltage - min_voltage);
}

std::optional<double> Battery::power_loss(double net_power_demanded, double state_of_charge) const {
     
    double open_circuit_voltage = current_voltage(state_of_charge);

     
     
     

    double current;
    if (net_power_demanded >= 0) {
         
        double discriminant = open_circuit_voltage * open_circuit_voltage + 4 * pack_resistance * net_power_demanded;
        if (discriminant < 0) {
            return std::nullopt;
        }
        current = (-open_circuit_voltage + std::sqrt(discriminant)) / (2 * pack_resistance);
    } else {
         
        double P_abs = -net_power_demanded;
        double discriminant = open_circuit_voltage * open_circuit_voltage - 4 * pack_resistance * P_abs;
        if (discriminant < 0) {
            return std::nullopt;
        }
        current = (open_circuit_voltage - std::sqrt(discriminant)) / (2 * pack_resistance);
    }

     
    return current * current * pack_resistance;
}