add_library(weather_stations "")

target_sources(
	weather_stations
	PRIVATE
		WeatherStations.cpp
		WeatherStationConstants.h
	PUBLIC
		WeatherStations.h
)

target_include_directories(
	weather_stations
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/RaceConfig
)

target_link_libraries(weather_stations PRIVATE tools)
