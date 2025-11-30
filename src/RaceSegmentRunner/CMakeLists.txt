add_library(race_segment_runner STATIC)

target_sources(
	race_segment_runner
	PUBLIC
		RaceSegmentRunner.h
	PRIVATE
		RaceSegmentRunner.cpp
)

target_link_libraries(
	race_segment_runner
	PUBLIC
		solarcar
		route
		weather
)

# add_executable(race_segment_runner_test_gen race_segment_runner_test_gen.cpp)
# target_link_libraries(race_segment_runner_test_gen PRIVATE race_segment_runner)

add_executable(race_segment_runner_tests RaceSegmentRunnerTests.cpp)
target_link_libraries(
	race_segment_runner_tests
	PRIVATE
		race_segment_runner
		Catch2::Catch2WithMain
)

catch_discover_tests(race_segment_runner_tests)
