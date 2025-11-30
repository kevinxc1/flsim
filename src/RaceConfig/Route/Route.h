#ifndef MINISIM_ROUTE_H
#define MINISIM_ROUTE_H

#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "RaceConfig/WeatherStations/WeatherStations.h"
#include "RouteSegment.h"

/// @brief A wrapper class for a Route the car is taking.
class Route {
   public:
	explicit Route(std::string_view route_file, WeatherStations weather_stations);
	Route() = default;

	size_t get_num_segments() const;
	size_t get_num_weather_stations() const;
	RouteSegment get_segment(size_t index) const;
	RouteSegment operator[](size_t index) const;
	double get_total_distance() const;
	/// @return The distance between two segments, excluding the last segment
	double get_distance_between(size_t index1, size_t index2) const;

	WeatherStations weather_stations;

	/// @return A pointer to a const version of segments
	const std::vector<RouteSegment>* get_segments() const;
	std::span<const RouteSegment> get_segments_span() const;

	static std::vector<GeographicalCoordinate> parse_weather_stations(std::string_view weatherStationsFile);

   private:
	std::vector<RouteSegment> segments;
	double total_distance = 0;
};

#endif  // MINISIM_ROUTE_H
