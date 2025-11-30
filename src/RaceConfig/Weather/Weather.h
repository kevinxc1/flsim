#ifndef MINISIM_WEATHER_H
#define MINISIM_WEATHER_H

#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "RaceConfig/WeatherStations/WeatherStations.h"
#include "WeatherConstants.h"
#include "WeatherDataPoint.h"
#include "alglib/interpolation.h"

/// This class encapsulates all weather data and construction of splines (which predict data in between our known
/// discrete data points
class Weather {
   public:
	Weather() = default;

	/// @brief Construct a new Weather object
	/// @param weatherFile the path to the weather file
	/// @param weather_station_coordinates the coordinates of the weather stations
	Weather(std::string_view weather_file, const WeatherStations& weather_station);

	/// @brief Construct a new Weather object from multiple weather files and merge them together
	Weather(std::span<const std::string> weather_files, const WeatherStations& weather_station_coordinates);

	/// @brief get the weather data point at the given weather group and time
	/// @param weather_station the weather group as a decimal
	/// @param time the time
	/// @return WeatherDataPoint the weather data point at the given weather group and time
	WeatherDataPoint get_weather_at(double weather_station, double time) const;

	/// @brief get the weather data point at the given weather group and time segment
	/// @param weather_station the weather group as a decimal
	/// @param start_time the start time
	/// @param end_time the end time
	/// @return WeatherDataPoint the weather data point at the given weather group and time segment
	WeatherDataPoint get_weather_during(double weather_station, double start_time, double end_time) const;

   private:
	struct SplineAndStartTime {
		double start_time;
		alglib::spline2dinterpolant weather_spline;
	};
	/// spline interpolant for all weather data
	std::vector<SplineAndStartTime> weather_splines;

	/// the number of weather groups
	int num_weather_groups;
};

#endif  // MINISIM_WEATHER_H
