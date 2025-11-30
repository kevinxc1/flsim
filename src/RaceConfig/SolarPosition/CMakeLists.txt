add_library(solar_position "")

target_sources(solar_position PRIVATE SolarPosition.cpp PUBLIC SolarPosition.h)

target_include_directories(
	solar_position
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/RaceConfig
)

target_link_libraries(
	solar_position
	PRIVATE
		tools
		solpos
)
