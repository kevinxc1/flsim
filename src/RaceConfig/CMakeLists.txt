add_library(raceconfig INTERFACE)

add_subdirectory(RaceSchedule)
add_subdirectory(WeatherStations)
add_subdirectory(Route)
add_subdirectory(Weather)
add_subdirectory(SolarPosition)

target_sources(raceconfig INTERFACE RaceConfig.h)

target_link_libraries(
	raceconfig
	INTERFACE
		raceschedule
		weather_stations
		route
		weather
		solar_position
)
