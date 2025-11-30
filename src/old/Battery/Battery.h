#ifndef MINISIM_BATTERY_H
#define MINISIM_BATTERY_H

#include <cassert>
#include <optional>

class Battery {
   public:
	/// @note @p maximum_voltage must be greater than or equal to @p minimum_voltage
	Battery(double energy_capacity, double pack_resistance, double minimum_voltage, double maximum_voltage)
		: energy_capacity(energy_capacity),
		  pack_resistance(pack_resistance),
		  min_voltage(minimum_voltage),
		  max_voltage(maximum_voltage) {
			assert(maximum_voltage >= minimum_voltage);
		  }

	/// @brief Calculates the state of charge of the battery, given its current stored energy.
	///
	/// @param energy_remaining (Wh) the energy remaining in the battery
    /// @return unitless state of charge [0,1] relative to the maximum capacity of the battery.
	double state_of_charge(double energy_remaining) const;

	/// @brief Calculates the current voltage of the battery, given its state of charge
	///
	/// @param state_of_charge (fraction [0, 1]) the remaining energy in the battery, relative to
	/// its maximum capacity
	double current_voltage(double state_of_charge) const;

	/// @brief Calculates the power loss due to battery resistance given the power demanded, as well
	/// as the current state of charge of the battery.
	///
	/// @param net_power_demanded the power demanded from the battery (positive means power out,
	/// negative means power in)
	/// @param state_of_charge (fraction [0, 1]) the remaining energy in the battery, relative to
	/// its maximum capacity
	/// @returns (W) Power Loss due to battery resistance. If the situation is physically impossible,
	/// returns an std::nullopt instead (e.g. the square root of a negative number).
	std::optional<double> power_loss(double net_power_demanded, double state_of_charge) const;

	/// @returns (Wh) The energy capacity of the battery
	// clang-format off
	inline double get_capacity() const { return energy_capacity; }
	// clang-format on

   private:
	/// @brief (Wh) the amount of energy the battery stores.
	double energy_capacity;

	/// @brief (Ohms) the electrical resistance internal to the pack
	double pack_resistance;

	/// @brief (V) the minimum voltage of our battery, when it's at zero remaining energy
	double min_voltage;

	/// @brief (V) the maximum voltage of our battery, when it's at full capacity
	double max_voltage;
};

#endif  // MINISIM_BATTERY_H
