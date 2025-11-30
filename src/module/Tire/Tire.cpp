#include "Tire.h"
#include <cmath>

double Tire::rolling_resistance(double tire_load, double vehicle_speed, std::optional<double> tire_pressure) const {
     
    double pressure = tire_pressure.value_or(tire_pressure_at_stc);
    
     
     
     
     
     
     
    
     
    double vehicle_speed_kmh = vehicle_speed * 3.6;
    
    double pressure_term = std::pow(pressure, alpha);
    double load_term = std::pow(tire_load, beta);
    double speed_term = a + b * vehicle_speed_kmh + c * vehicle_speed_kmh * vehicle_speed_kmh;
    
    return pressure_term * load_term * speed_term;
}