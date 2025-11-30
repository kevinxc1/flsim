#ifndef MINISIM_BATTERYSTATE_H
#define MINISIM_BATTERYSTATE_H

class BatteryState {
   public:
	BatteryState(double energy_capacity) : energy_remaining(energy_capacity) {}

	/// @brief Updates the energy remaining.
	///
	/// @todo In a later iteration of training, update the interface to be called @p energy_outflow.
	///
	/// @param delta_energy (Wh) The change in energy to update. Positive means energy is gained,
	/// negative means energy is lost.
	void update_energy_remaining(double delta_energy);

	/// @returns The energy remaining
	// clang-format off
	inline double get_energy_remaining() const { return energy_remaining; }
	// clang-format on

   private:
	/// @brief (Wh) the amount of energy remaining in the battery.
	double energy_remaining;
};

#endif  // MINISIM_BATTERYSTATE_H