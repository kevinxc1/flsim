#include "LinearSearchOptimizer.h"

#include <optional>

#include "RaceRunner/RaceRunner.h"

LinearSearchOptimizer::LinearSearchOptimizer(
	const SolarCar& car, const Weather& weather, const Route& route, const RaceSchedule& schedule)
	: car(car), weather(weather), route(route), schedule(schedule) {}

std::optional<Optimizer::OptimizationOutput> LinearSearchOptimizer::optimize_race() const {
	 
	 

	double best_speed = 0;
	double best_racetime = 0;

	 
	for (double speed = minimum_speed; speed <= maximum_speed; speed += speed_step) {
		const auto racetime_opt = RaceRunner::calculate_racetime(car, route, weather, schedule, speed);

		 
		if (racetime_opt.has_value()) {
			best_speed = speed;
			best_racetime = racetime_opt.value();
		}
	}

	 
	if (best_speed == 0) {
		return std::nullopt;
	}

	return OptimizationOutput{best_racetime, best_speed};
}
