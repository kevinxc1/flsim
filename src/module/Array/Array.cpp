#include "Array.h"

double Array::power_in(double irradiance) const {
     
     
    return array_area * (array_efficiency / 100.0) * irradiance;
}