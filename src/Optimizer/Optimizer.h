#ifndef MINISIM_OPTIMIZER_H
#define MINISIM_OPTIMIZER_H

#include <memory>
#include <optional>
#include <string_view>

#include "RaceConfig/RaceSchedule/RaceSchedule.h"
#include "RaceConfig/Route/Route.h"
#include "RaceConfig/Weather/Weather.h"
#include "SolarCar/SolarCar.h"

class Optimizer {
   public:
	Optimizer() = default;

	// Make sure that the destructor is virtual so that the correct destructor is called when deleting an Optimizer
	virtual ~Optimizer() = default;
	Optimizer(Optimizer&&) = default;
	Optimizer(const Optimizer&) = default;
	Optimizer& operator=(Optimizer&&) = default;
	Optimizer& operator=(const Optimizer&) = default;

	struct OptimizationOutput {
		double racetime;
		double speed;
	};

	/// Using a heuristic, optimizes the entire race.
	/// @returns The optimized output result.
	virtual std::optional<OptimizationOutput> optimize_race() const = 0;

	// Factory Methods
	/// Creates an Optimizer from a Config File.
	///
	/// @param [in] optimizer_type The type of optimizer to create.
	/// @param [in] solarcar The solar car we're racing.
	/// @param [in] weather The weather forecast for the race.
	/// @param [in] route The route we're racinga long.
	/// @param [in] schedule The schedule that we're racing with.
	///
	/// @returns The created optimizer.
	static std::unique_ptr<const Optimizer> create_optimizer(std::string_view optimizer_type, const SolarCar& solarcar,
		const Weather& weather, const Route& route, const RaceSchedule& schedule);
};

#endif  // MINISIM_OPTIMIZER_H
