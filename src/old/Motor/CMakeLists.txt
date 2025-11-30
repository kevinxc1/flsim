add_library(motor STATIC)

target_sources(motor PUBLIC Motor.h PRIVATE Motor.cpp)

# add_executable(motor_test_gen motor_test_gen.cpp)
# target_link_libraries(motor_test_gen PUBLIC motor)

add_executable(motor_tests MotorTests.cpp)
target_link_libraries(
	motor_tests
	PRIVATE
		motor
		Catch2::Catch2WithMain
)

catch_discover_tests(motor_tests)
