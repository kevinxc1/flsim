#include "Battery.h"
#include <cmath>
#include <algorithm>

double Battery::state_of_charge(double energy_remaining) const {
    return energy_remaining / energy_capacity;
}

double Battery::current_voltage(double state_of_charge) const {
    // Linear interpolation between min and max voltage based on state of charge
    return min_voltage + state_of_charge * (max_voltage - min_voltage);
}

std::optional<double> Battery::power_loss(double net_power_demanded, double state_of_charge) const {
    // Calculate the open-circuit voltage at this state of charge
    double open_circuit_voltage = current_voltage(state_of_charge);

    // Solve for current using the quadratic relationship between power, voltage, and resistance
    // For discharging (P > 0): P = I * V_oc + I²R (power includes losses)
    // For charging (P < 0): |P| = I * V_oc - I²R (stored power is input minus losses)

    double current;
    if (net_power_demanded >= 0) {
        // Discharging: I²R + I*V_oc - P = 0
        double discriminant = open_circuit_voltage * open_circuit_voltage + 4 * pack_resistance * net_power_demanded;
        if (discriminant < 0) {
            return std::nullopt;
        }
        current = (-open_circuit_voltage + std::sqrt(discriminant)) / (2 * pack_resistance);
    } else {
        // Charging: I²R - I*V_oc + |P| = 0
        double P_abs = -net_power_demanded;
        double discriminant = open_circuit_voltage * open_circuit_voltage - 4 * pack_resistance * P_abs;
        if (discriminant < 0) {
            return std::nullopt;
        }
        current = (open_circuit_voltage - std::sqrt(discriminant)) / (2 * pack_resistance);
    }

    // Power loss due to internal resistance: P_loss = I² * R
    return current * current * pack_resistance;
}