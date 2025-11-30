add_library(tire STATIC)

target_sources(tire PUBLIC Tire.h PRIVATE Tire.cpp)

# add_executable(tire_test_gen tire_test_gen.cpp)
# target_link_libraries(tire_test_gen PUBLIC tire)

add_executable(tire_tests TireTests.cpp)
target_link_libraries(
	tire_tests
	PRIVATE
		tire
		Catch2::Catch2WithMain
)

catch_discover_tests(tire_tests)
