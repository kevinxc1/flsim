add_library(weather "")

target_sources(
	weather
	PRIVATE
		Weather.cpp
		WeatherDataPoint.cpp
	PUBLIC
		Weather.h
		WeatherConstants.h
		WeatherDataPoint.h
)

target_link_libraries(
	weather
	PRIVATE
		tools
		weather_stations
)
