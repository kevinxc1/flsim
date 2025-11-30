#include "BinarySearchOptimizer.h"

#include <optional>

#include "RaceRunner/RaceRunner.h"

BinarySearchOptimizer::BinarySearchOptimizer(
	const SolarCar& car, const Weather& weather, const Route& route, const RaceSchedule& schedule)
	: car(car), weather(weather), route(route), schedule(schedule) {}

std::optional<Optimizer::OptimizationOutput> BinarySearchOptimizer::optimize_race() const {
	 
	 

	double low = minimum_speed;
	double high = maximum_speed;
	double best_speed = 0;
	double best_racetime = 0;

	 
	while (high - low > precision) {
		const double mid = (low + high) / 2.0;

		const auto racetime_opt = RaceRunner::calculate_racetime(car, route, weather, schedule, mid);

		if (racetime_opt.has_value()) {
			 
			best_speed = mid;
			best_racetime = racetime_opt.value();
			low = mid;
		} else {
			 
			high = mid;
		}
	}

	 
	if (best_speed == 0) {
		return std::nullopt;
	}

	 
	 
	auto verification = RaceRunner::calculate_racetime(car, route, weather, schedule, best_speed);
	if (!verification.has_value()) {
		 
		best_speed -= precision;
		auto fallback = RaceRunner::calculate_racetime(car, route, weather, schedule, best_speed);

		if (!fallback.has_value()) {
			 
			return std::nullopt;
		}

		best_racetime = fallback.value();
	}

	return OptimizationOutput{best_racetime, best_speed};
}
