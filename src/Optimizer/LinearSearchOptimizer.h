#ifndef MINISIM_LINEARSEARCHOPTIMIZER_H
#define MINISIM_LINEARSEARCHOPTIMIZER_H

#include <optional>

#include "Optimizer.h"
#include "RaceConfig/RaceSchedule/RaceSchedule.h"
#include "RaceConfig/Route/Route.h"
#include "RaceConfig/Weather/Weather.h"
#include "SolarCar/SolarCar.h"

class LinearSearchOptimizer : public Optimizer {
   public:
	explicit LinearSearchOptimizer(
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
	/// The precision we're searching at.
	static constexpr double speed_step = 0.1;
};

#endif  // MINISIM_LINEARSEARCHOPTIMIZER_H
