#include "Tire.h"
#include <cmath>

double Tire::rolling_resistance(double tire_load, double vehicle_speed, std::optional<double> tire_pressure) const {
    // Use the provided tire pressure or fall back to standard conditions pressure
    double pressure = tire_pressure.value_or(tire_pressure_at_stc);
    
    // SAE J2452 formula: RR = P^α × Z^β × (a + b×V + c×V²)
    // Where:
    // P = tire inflation pressure (kPa)
    // Z = applied load/vehicle weight (N) 
    // V = vehicle speed (converted to km/h based on coefficient units)
    // α, β, a, b, c = coefficients
    
    // Convert vehicle speed from m/s to km/h (multiply by 3.6)
    double vehicle_speed_kmh = vehicle_speed * 3.6;
    
    double pressure_term = std::pow(pressure, alpha);
    double load_term = std::pow(tire_load, beta);
    double speed_term = a + b * vehicle_speed_kmh + c * vehicle_speed_kmh * vehicle_speed_kmh;
    
    return pressure_term * load_term * speed_term;
}