#ifndef MINISIM_RACESEGMENTRUNNER_H
#define MINISIM_RACESEGMENTRUNNER_H

#include "RaceConfig/Route/RouteSegment.h"
#include "RaceConfig/Weather/WeatherDataPoint.h"
#include "SolarCar/SolarCar.h"

class RaceSegmentRunner {
   public:
	explicit RaceSegmentRunner(const SolarCar& solar_car) : car(solar_car) {};

	/// @brief Calculates the resistive force the car experiences over a certain stretch.
	///
	/// @param car The solar car.
	/// @param route_segment The Route Segment the car is driving on.
	/// @param weather_data The Weather data at the time the car is driving.
	/// @param speed (m/s) The requested speed for the car to drive at.
	/// @note Assume the car has three wheels. For simplicity, also assume the car's 
    /// weight is uniform across all three wheels.
	/// @returns (N) The resistive force the car experiences, in Newtons.
	double calculate_resistive_force(
		const RouteSegment& route_segment, const WeatherDataPoint& weather_data, double speed) const;

	/// @brief Calculate the power required to drive the given segment at the given speed.
	/// @hint There are 3 components you should include here
	/// @param route_segment The Route Segment the car is driving on.
	/// @param weather_data The Weather data at the time the car is driving.
	/// @param speed (m/s) The requested speed for the car to drive at.
	///
	/// @returns (W) The power out the car demands to travel the given speed over the route.
	double calculate_power_out(
		const RouteSegment& route_segment, const WeatherDataPoint& weather_data, double speed) const;

	/// @brief Calculate the power brought in by power-generating components of the car.
	///
	/// @param route_segment The Route Segment the car is driving on.
	/// @param weather_data The Weather data at the time the car is driving.
	///
	/// @returns (W) The power in the car generates while traveling over the given route segment.
	double calculate_power_in(const RouteSegment& route_segment, const WeatherDataPoint& weather_data) const;

	/// @brief Calculate the net power used during this segment.
	///
	/// @note a negative power means the car is using power, while a positive one is gaining power.
	///
	/// Hint: Don't forget to take Battery Losses into account here--they only depend on net power (pre-loss), not power
	/// out.
	///
	/// @param route_segment The Route Segment the car is driving on.
	/// @param weather_data The Weather data at the time the car is driving.
	/// @param state_of_charge The current state of charge of the battery.
	/// @param speed (m/s) The requested speed for the car to drive at.
	///
	/// @returns (W) The net power that the car draws (or gains) over the given segment. A negative net power means the
	/// car is demanding power, while a positive one represents gaining power. Returns std::nullopt if the speed
	/// demanded puts the car in an impossible physical position (e.g. requires the square root of a negative number).
	std::optional<double> calculate_power_net(const RouteSegment& route_segment, const WeatherDataPoint& weather_data,
		double state_of_charge, double speed) const;

   private:
	SolarCar car;
};

#endif  // MINISIM_RACESEGMENTRUNNER_H
