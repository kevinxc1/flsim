#ifndef MINISIM_RACERUNNER_H
#define MINISIM_RACERUNNER_H

#include <optional>

#include "RaceConfig/RaceSchedule/RaceSchedule.h"
#include "RaceConfig/Route/Route.h"
#include "RaceConfig/Weather/Weather.h"
#include "SolarCar/SolarCar.h"

namespace RaceRunner {
	/// @brief Calculates the Watt-hours of energy gained while static charging. This means the energy gained while not
	/// moving and simply charging.
	///
	/// This is going to be used when we reach a checkpoint, are charging at the beginning of the day (before racing
	/// begins), or are charging at the end of the day (after racing ends).
	///
	/// @note This should use time increments of 5 minutes (or 300 seconds). Every 300 seconds,
	/// weather data will be resampled to stretch calculations.
	///
	/// @requires start_time < end_time.
	///
	/// @param [in] car The car that will be running the race.
	/// @param [in] weather The weather capturing weather data ranging from @p start_time through to @p end_time.
	/// @param [in] weather_station The weather station representing the current position of the car.
	/// @param [in] start_time The time to start static charging calculation.
	/// @param [in] end_time The time to end static charging calculation.
	/// @returns (Wh) the total energy gained by static charging.
	double calculate_static_charging_gain(
		const SolarCar& car, const Weather& weather, double weather_station, double start_time, double end_time);

	/// @brief Calculates the total racetime of a race with the given parameters, traveling at a constant speed.
	///
	/// Each Race Day is divided into up to four stages:
	///
	/// 1. Morning Charging (BoD Charging)
	/// 2. Racing
	/// 3. Checkpoints (Control Stops)
	/// 4. Evening Charging (EoD Charging)
	///
	/// This function should reflect that framework.
	///
	/// The Race Starts on Schedule Day 0. We do not charge on the first day. Every subsequent day starts at @p
	/// morning_charging_start_time, charges until @p morning_charging_end_time, races from @race_start_time to
	/// @p race_end_time, and evening charges from @p evening_charging_start_time to @p evening_charge_end_time.
	///
	/// The Route End Condition explains whether or not the route ends in a Control Stop. If the Route ends in a
	/// checkpoint, complete the checkpoint before continuing with the rest of the program.
	///
	/// Assume there is no difference between Racing and Marshaling Route Segments.
	///
	/// @param [in] car The car that will be running the race.
	/// @param [in] route The route to drive the car on.
	/// @param [in] weather The weather capturing weather data from the dates encolsed in @p schedule.
	/// @param [in] schedule The schedule of the race.
	/// @param [in] speed (@p speed > 0) The speed of the car to race at.
	/// @returns (seconds) The total time it takes to complete the given Route, including control stops. If
	/// the car runs out of energy before finishing the race, returns std::nullopt.
	std::optional<double> calculate_racetime(
		const SolarCar& car, const Route& route, const Weather& weather, const RaceSchedule& schedule, double speed);
};  // namespace RaceRunner

#endif  // MINISIM_RACERUNNER_H
