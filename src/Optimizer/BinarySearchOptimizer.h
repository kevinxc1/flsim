#ifndef MINISIM_BINARYSEARCHOPTIMIZER_H
#define MINISIM_BINARYSEARCHOPTIMIZER_H

#include <optional>

#include "Optimizer.h"
#include "RaceConfig/RaceSchedule/RaceSchedule.h"
#include "RaceConfig/Route/Route.h"
#include "RaceConfig/Weather/Weather.h"
#include "SolarCar/SolarCar.h"

class BinarySearchOptimizer : public Optimizer {
   public:
	explicit BinarySearchOptimizer(
		const SolarCar& car, const Weather& weather, const Route& route, const RaceSchedule& schedule);

	std::optional<Optimizer::OptimizationOutput> optimize_race() const override;

   private:
	const SolarCar& car;
	const Weather& weather;
	const Route& route;
	const RaceSchedule& schedule;
	/// The minimum speed we want to go at.
	static constexpr double minimum_speed = 5;  // mps
	/// The maximum speed we're allowed to go at.
	static constexpr double maximum_speed = 50;  // mps
	/// The precision we're searching until.
	///
	/// In other words, once our speed changes would be less than 0.1 meters per
	/// second, stop searching.
	///
	/// @note There may be situations in which the speed you settle on is too
	/// fast, because of the precision. Be sure to explicitly check for that and
	/// fall back to a speed lower by @p precision if necessary.
	static constexpr double precision = 0.1;
};

#endif  // MINISIM_BINARYSEARCHOPTIMIZER_H
