add_library(route "")

target_sources(
	route
	PRIVATE
		Route.cpp
	PUBLIC
		RouteConstants.h
		RouteSegment.h
		Route.h
)

target_link_libraries(
	route
	PRIVATE
		tools
		weather_stations
)
