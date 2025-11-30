#include "WeatherStations.h"

#include <cstddef>
#include <string_view>

#include "WeatherStationConstants.h"
#include "csv/csv.h"

WeatherStations::WeatherStations(std::string_view weather_stations_file) {
	io::CSVReader<weather::stations::NUM_COLUMNS_STATIONS_FILE> stations_csv(weather_stations_file.data());

	stations_csv.read_header(io::ignore_extra_column,
		weather::stations::CN_STATION_NAME.data(),       
		weather::stations::CN_STATION_ID.data(),         
		weather::stations::CN_STATION_LATITUDE.data(),   
		weather::stations::CN_STATION_LONGITUDE.data()   
	);

	GeographicalCoordinate coord = {};
	std::string _name;
	int id;

	while (stations_csv.read_row(_name, id, coord.latitude, coord.longitude)) {
		weather_stations.push_back(coord);
	}
}

size_t WeatherStations::size() const {
	return weather_stations.size();
}

GeographicalCoordinate WeatherStations::operator[](size_t index) const {
	return weather_stations[index];
}

GeographicalCoordinate WeatherStations::get(size_t index) const {
	return weather_stations[index];
}
