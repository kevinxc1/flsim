#ifndef MINISIM_RACECONFIG_H
#define MINISIM_RACECONFIG_H

#include <optional>

#include "RaceConfig/RaceSchedule/RaceSchedule.h"
#include "RaceConfig/SolarPosition/SolarPosition.h"
#include "Route/Route.h"
#include "SolarCar/SolarCar.h"
#include "Weather/Weather.h"

struct RaceConfig {
	const SolarCar car;
	const Route route;
	const Weather weather;
	const RaceSchedule schedule;
};

#endif  // MINISIM_RACECONFIG_H
