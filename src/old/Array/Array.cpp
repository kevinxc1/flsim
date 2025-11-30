#include "Array.h"

double Array::power_in(double irradiance) const {
    // Power = Area * Efficiency * Irradiance
    // Efficiency is given as percentage, so divide by 100
    return array_area * (array_efficiency / 100.0) * irradiance;
}