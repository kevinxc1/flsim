#include "Optimizer.h"

#include <exception>
#include <iostream>
#include <memory>
#include <string_view>

#include "BinarySearchOptimizer.h"
#include "LinearSearchOptimizer.h"
#include "RaceConfig/RaceSchedule/RaceSchedule.h"
#include "RaceConfig/Route/Route.h"
#include "RaceConfig/Weather/Weather.h"
#include "SolarCar/SolarCar.h"

namespace {
	enum class OptimizerType {
		LinearSearchOptimizer,
		BinarySearchOptimizer,
	};

	OptimizerType get_optimizer_type(const std::string_view name) {
		if (name == "linear") {
			return OptimizerType::LinearSearchOptimizer;
		}
		if (name == "binary") {
			return OptimizerType::BinarySearchOptimizer;
		}
		std::cerr << "Invalid Optimizer Type: " << name << "\n";
		throw std::exception();
	}
}   

std::unique_ptr<const Optimizer> Optimizer::create_optimizer(const std::string_view optimizer_type,
	const SolarCar& solarcar, const Weather& weather, const Route& route, const RaceSchedule& schedule) {
	const auto type = get_optimizer_type(optimizer_type);

	switch (type) {
		case OptimizerType::LinearSearchOptimizer: {
			return std::make_unique<LinearSearchOptimizer>(solarcar, weather, route, schedule);
		}
		case OptimizerType::BinarySearchOptimizer: {
			return std::make_unique<BinarySearchOptimizer>(solarcar, weather, route, schedule);
		}
	}
	assert(false);
	return nullptr;
}
