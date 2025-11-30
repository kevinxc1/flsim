add_library(racerunner STATIC)

target_sources(racerunner PUBLIC RaceRunner.h PRIVATE RaceRunner.cpp)

target_link_libraries(
	racerunner
	PRIVATE
		race_segment_runner
		solarcar
		weather
		route
		weather_stations

)

# add_executable(racerunner_test_gen racerunner_test_gen.cpp)
# target_link_libraries(
# 	racerunner_test_gen
# 	PRIVATE
# 		racerunner
# 		tools
# 		raceschedule
# 		solarcar
# 		weather
# 		route
# 		weather_stations
# )

add_executable(racerunner_tests RaceRunnerTests.cpp)
target_link_libraries(
	racerunner_tests
	PRIVATE
		racerunner
		raceschedule
		solarcar
		weather
		route
		weather_stations
		root_tool
		Catch2::Catch2WithMain
)

catch_discover_tests(racerunner_tests)
