#ifndef MINISIM_WEATHERSTATION_H
#define MINISIM_WEATHERSTATION_H

#include <string_view>
#include <vector>

#include "DataClasses/GeographicalCoordinate.h"

class WeatherStations {
   public:
	explicit WeatherStations(std::string_view weather_stations_file);
	explicit WeatherStations(std::vector<GeographicalCoordinate> weather_stations)
		: weather_stations(std::move(weather_stations)){};
	WeatherStations() = default;

	size_t size() const;
	GeographicalCoordinate operator[](size_t index) const;
	GeographicalCoordinate get(size_t index) const;

   private:
	std::vector<GeographicalCoordinate> weather_stations;
};

#endif  // MINISIM_WEATHERSTATION_H
