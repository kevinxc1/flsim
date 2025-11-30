#ifndef MINISIM_TIRE_H
#define MINISIM_TIRE_H

#include <optional>

/// @brief A struct containing the SAE J2452 Coefficients for Tire Model
/// construction.
///
/// The reason that this exists is because, especially when there are constants
/// this random, it becomes much much easier to make a mistake when using them.
/// Fortunately, structs allow 1) designated initialization (C++17) and 2)
/// manually setting each variable. This gives us the option to be very explicit
/// when initializing.
struct SaeJ2452Coefficients {
	double alpha;
	double beta;
	double a;
	double b;
	double c;
};

class Tire {
   public:
	Tire(double alpha, double beta, double a, double b, double c, double pressure_at_stc)
		: alpha(alpha), beta(beta), a(a), b(b), c(c), tire_pressure_at_stc(pressure_at_stc) {}

	Tire(SaeJ2452Coefficients coefficients, double pressure_at_stc)
		: alpha(coefficients.alpha),
		  beta(coefficients.beta),
		  a(coefficients.a),
		  b(coefficients.b),
		  c(coefficients.c),
		  tire_pressure_at_stc(pressure_at_stc) {}

	/// @brief Calculates the rolling resistance of a singular tire using the SAE J2452 Standard.
	///
	/// @param tire_load (+N) the load on the tire. This must be non-negative.
	/// @param vehicle_speed (+m/s) the velocity of the vehicle (and thus, the wheel). This must
	/// be non-negative
	/// @param tire_pressure (+kPa, Optional) the internal tire pressure, if different from STC.
	/// This must be non=negative.
	/// @return rolling resistance (N) for tire.
    ///
	/// @note Be careful with your units! Pay attention to what you get in and what you actually
	/// need. You might need to reference an SAE J2452 datasheet to see what units the coefficients
	/// are tuned for, and adjust the code. Assume that we always operate with coefficients tuned
	/// to the same units.
	///
	double rolling_resistance(
		double tire_load, double vehicle_speed, std::optional<double> tire_pressure = std::nullopt) const;

   private:
	/// @brief one of the SAE J2452 Coefficients
	double alpha;
	/// @brief one of the SAE J2452 Coefficients
	double beta;
	/// @brief one of the SAE J2452 Coefficients
	double a;
	/// @brief one of the SAE J2452 Coefficients
	double b;
	/// @brief one of the SAE J2452 Coefficients
	double c;

	/// @brief the tire pressure under standard conditions (e.g. STC, not
	/// impacted by external conditions)
	double tire_pressure_at_stc;
};

#endif
