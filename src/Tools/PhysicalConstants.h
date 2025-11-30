#ifndef MINISIM_PHYSICALCONSTANTS_H
#define MINISIM_PHYSICALCONSTANTS_H

#include "Conversions.h"

namespace physical_constants {

	// STC values
	static constexpr double STC_IRRADIANCE = 1000.0;                                           // W/m^2
	static constexpr double STC_TEMPERATURE = 25.0;                                            // C
	static constexpr double STC_TEMPERATURE_KELVIN = celsius_to_kelvin(STC_TEMPERATURE);       // K
	static constexpr double STC_RECIPROCAL_TEMPERATURE_KELVIN = 1.0 / STC_TEMPERATURE_KELVIN;  // 1/K

	// Physical Constants
	static constexpr double ELEMENTARY_CHARGE = 1.60217663e-19;
	static constexpr double BOLTZMANN_CONSTANT = 1.380649e-23;

	// Semiconductor Physics
	namespace semiconductors {
		// https://www.sciencedirect.com/science/article/pii/0031891467900626
		namespace si {
			static constexpr double ZERO_KELVIN_BAND_GAP_ENERGY_EV = 1.1557;
			static constexpr double BAND_GAP_ENERGY_ALPHA_COEFFICIENT = 7.021e-4;
			static constexpr double BAND_GAP_ENERGY_BETA_COEFFICIENT = 1108;
		}  // namespace si
		namespace ga_as {
			static constexpr double ZERO_KELVIN_BAND_GAP_ENERGY_EV = 1.5216;
			static constexpr double BAND_GAP_ENERGY_ALPHA_COEFFICIENT = 8.871e-4;
			static constexpr double BAND_GAP_ENERGY_BETA_COEFFICIENT = 572;
		}  // namespace ga_as
	}  // namespace semiconductors

	// Derived Constants
	static constexpr double ELEMENTARY_CHARGE_OVER_BOLTZMANN_CONSTANT = ELEMENTARY_CHARGE / BOLTZMANN_CONSTANT;
	static constexpr double BOLTZMANN_CONSTANT_OVER_ELEMENTARY_CHARGE = BOLTZMANN_CONSTANT / ELEMENTARY_CHARGE;

}  // namespace physical_constants

#endif  // MINISIM_PHYSICALCONSTANTS_H
